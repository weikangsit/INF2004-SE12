/** @file    telemetry.c
 *  @brief   Telemetry Publisher task (Buddy 1) - lowest priority (FR6).
 *
 *  Any task reports an event with telemetry_post_event(): the event is
 *  packed at once (1-4 bytes, proto.h) and queued on telemetry_q. Every
 *  100 ms this task forwards the queued events, refreshes the live fault
 *  bits and publishes one 18-byte telemetry frame through comms. The
 *  frame is built from the packed shared state record (vehicle_state.h),
 *  so only integer unit conversions are needed here.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <string.h>

#include "port.h"

#include "barcode_map.h"
#include "car_config.h"
#include "comms.h"
#include "fault.h"
#include "imu_terrain.h"
#include "safety.h"
#include "systime.h"
#include "telemetry.h"
#include "ultrasonic_scan.h"
#include "vehicle_state.h"

#define MS_PER_TICK          (100U)    /* uptime field resolution        */
#define UPTIME_MASK          (0xFFFFU)
#define RANGE_MAX_CM         (510U)    /* 511 is reserved for no echo    */
#define MM_PER_CM            (10)
#define HALF_DEG_PER_DEG     (2U)
#define FULL_CIRCLE_DEG      (360U)
#define QUEUE_DEPTH          (16U)
#define MBF_SLOT_BYTES       (8U)      /* 4-byte header + up to 4 bytes  */

static ID g_telemetry_q = 0;

static void    live_faults_refresh (uint16_t front_cm);
static void    frame_fill (proto_telem_t * p_telem, uint16_t front_cm);
static int16_t div_round (int32_t value, int32_t divisor);
static void    event_queue_put (uint8_t const * p_frame, uint32_t len);

/*!
 * @brief  Create the event queue. Call from usermain() before the tasks
 *         start.
 */
void
telemetry_init (void)
{
    T_CMBF create;

    (void) memset(&create, 0, sizeof(create));
    create.mbfatr  = TA_TFIFO;
    create.bufsz   = (SZ) (QUEUE_DEPTH * MBF_SLOT_BYTES);      /* 128 B   */
    create.maxmsz  = (INT) PROTO_EVENT_MAX_LEN;
    g_telemetry_q  = tk_cre_mbf(&create);
}

/*!
 * @brief  Report an event. Safe to call from any task.
 *
 *  An EV_FAULT also sets its bit in the shared fault mask, and a failed
 *  encoder self-test sets FC_ENCODER, so the 10 Hz frame always carries
 *  every fault reported since the last reset.
 *
 * @param  kind    Event kind.
 * @param  code    Kind-specific code (see proto_event_t).
 * @param  value1  Kind-specific value 1.
 * @param  value2  Kind-specific value 2.
 */
void
telemetry_post_event (proto_event_t kind, uint8_t code, int16_t value1,
                      int16_t value2)
{
    uint8_t        frame[PROTO_EVENT_MAX_LEN];
    uint32_t const len = proto_pack_event(kind, code, value1, value2, frame);

    /* FC_COUNT is 16, so it fits the uint8_t comparison unchanged. */
    if ((EV_FAULT == kind) && (code < (uint8_t) FC_COUNT))
    {
        /* code < FC_COUNT, so it is a valid fault_code_t. */
        fault_set((fault_code_t) code);
    }
    else if ((EV_ENC_CAL == kind) && (0U != code))
    {
        fault_set(FC_ENCODER);
    }
    else
    {
        /* No fault bit for this event. */
    }

    if (0U != len)
    {
        event_queue_put(frame, len);
    }
}

/*!
 * @brief  Telemetry Publisher task body (priority PRI_TELEM, 100 ms).
 * @param  stacd    Start code (unused).
 * @param  p_exinf  Extended information (unused).
 */
void
telemetry_task (INT stacd, void * p_exinf)
{
    uint64_t next_us = time_us_64();

    (void) stacd;
    (void) p_exinf;

    for (;;)
    {
        uint8_t        event[PROTO_EVENT_MAX_LEN];
        uint8_t        frame[PROTO_TELEM_LEN];
        proto_telem_t  telem;
        uint16_t       front_cm  = US_RANGE_NO_ECHO;
        INT            event_len = 0;

        systime_period_wait(&next_us, PERIOD_TELEM_MS);
        safety_heartbeat(SAFETY_HB_TELEM);

        /* Step 1: forward every queued event. */
        event_len = tk_rcv_mbf(g_telemetry_q, event, TMO_POL);
        while (event_len > 0)
        {
            /* event_len is 1..PROTO_EVENT_MAX_LEN here. */
            (void) comms_publish(COMMS_TOPIC_EVENT, event,
                                 (uint32_t) event_len);
            event_len = tk_rcv_mbf(g_telemetry_q, event, TMO_POL);
        }

        /* Step 2: refresh live faults, then publish one frame. */
        front_cm = ultrasonic_scan_front_cm();
        live_faults_refresh(front_cm);
        frame_fill(&telem, front_cm);
        (void) comms_publish(COMMS_TOPIC_TELEMETRY, frame,
                             proto_pack_telem(&telem, frame));

        /* Step 3: the late-telemetry warning has now been reported once. */
        fault_clear(FC_TELEM_LATE);
    }
}

/*!
 * @brief  Set or clear the faults that describe live sensor / link state.
 * @param  front_cm  Latest front ultrasonic range (US_RANGE_NO_ECHO).
 */
static void
live_faults_refresh (uint16_t front_cm)
{
    static fault_code_t const codes[] = { FC_IMU, FC_NO_ECHO, FC_WIFI_LOSS };
    bool const                b_active[] =
    {
        !imu_terrain_is_ok(),
        (US_RANGE_NO_ECHO == front_cm),
        !comms_is_online()
    };
    uint32_t                  idx = 0U;

    for (idx = 0U; idx < (sizeof(codes) / sizeof(codes[0])); idx++)
    {
        if (b_active[idx])
        {
            fault_set(codes[idx]);
        }
        else
        {
            fault_clear(codes[idx]);
        }
    }
}

/*!
 * @brief  Fill the telemetry values (wire units) from the shared state.
 * @param  p_telem   Output values; proto_pack_telem() clamps each field.
 * @param  front_cm  Latest front ultrasonic range (US_RANGE_NO_ECHO).
 */
static void
frame_fill (proto_telem_t * p_telem, uint16_t front_cm)
{
    vehicle_state_t state;
    uint32_t        status  = 0U;
    uint32_t        heading = 0U;

    vehicle_state_read(&state);

    if (safety_is_latched())
    {
        status |= PROTO_ST_LATCHED;
    }

    if (barcode_map_is_loaded())
    {
        status |= PROTO_ST_CFG_MQTT;
    }

    /* 0..719 half degrees -> 0..359 degrees, rounded; 360 wraps to 0. */
    heading = ((uint32_t) state.heading_half_deg + 1U) / HALF_DEG_PER_DEG;
    if (heading >= FULL_CIRCLE_DEG)
    {
        heading -= FULL_CIRCLE_DEG;
    }

    /* The shared record already uses the telemetry units for most fields
     * (same bit order for line and barcode); the narrowing casts keep
     * values that proto_pack_telem() clamps again to each field width. */
    p_telem->state          = state.nav_state;
    p_telem->motion_class   = state.motion_class;
    p_telem->line_bits      = state.ir_black;
    p_telem->barcode        = state.last_barcode;
    p_telem->fault_mask     = fault_get_mask();
    p_telem->status         = (uint8_t) status;
    p_telem->speed_l_cms    = div_round(state.speed_l_mms, MM_PER_CM);
    p_telem->speed_r_cms    = div_round(state.speed_r_mms, MM_PER_CM);
    p_telem->duty_l_pct     = state.duty_l_pct;
    p_telem->duty_r_pct     = state.duty_r_pct;
    p_telem->pitch_half_deg = state.pitch_half_deg;
    p_telem->roll_deg       = state.roll_deg;
    p_telem->heading_deg    = (uint16_t) heading;
    p_telem->turn_rate_dps  = state.turn_rate_dps;
    p_telem->range_cm       = (US_RANGE_NO_ECHO == front_cm) ?
                              PROTO_RANGE_NO_ECHO :
                              ((front_cm > RANGE_MAX_CM) ?
                               (uint16_t) RANGE_MAX_CM : front_cm);
    p_telem->distance_cm    = div_round(state.dist_mm, MM_PER_CM);
    p_telem->hump_half_mm   = state.hump_peak_half_mm;
    p_telem->uptime_ticks   = (uint16_t) ((systime_now_ms() / MS_PER_TICK) &
                                          UPTIME_MASK);
}

/*!
 * @brief  Divide, rounding half away from zero, limited to int16_t.
 * @param  value    Dividend.
 * @param  divisor  Positive divisor.
 * @return Rounded quotient.
 */
static int16_t
div_round (int32_t value, int32_t divisor)
{
    int32_t const half     = divisor / 2;
    int32_t const quotient = (value >= 0) ? ((value + half) / divisor) :
                                            ((value - half) / divisor);
    int32_t const limited  = (quotient > INT16_MAX) ? INT16_MAX :
                             ((quotient < INT16_MIN) ? INT16_MIN : quotient);

    /* limited is within the int16_t range. */
    return ((int16_t) limited);
}

/*!
 * @brief  Queue an event; when the queue is full the oldest event is
 *         dropped so the newest one is kept (Tribunal 1 decision).
 * @param  p_frame  Packed event.
 * @param  len      Event length, 1..PROTO_EVENT_MAX_LEN.
 */
static void
event_queue_put (uint8_t const * p_frame, uint32_t len)
{
    uint8_t   discard[PROTO_EVENT_MAX_LEN];
    INT const msg_len = (INT) len;    /* <= PROTO_EVENT_MAX_LEN: fits INT */

    if (E_OK != tk_snd_mbf(g_telemetry_q, p_frame, msg_len, TMO_POL))
    {
        (void) tk_rcv_mbf(g_telemetry_q, discard, TMO_POL);
        (void) tk_snd_mbf(g_telemetry_q, p_frame, msg_len, TMO_POL);
    }
}

/*** end of file ***/

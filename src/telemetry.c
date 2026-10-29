/** @file    telemetry.c
 *  @brief   Telemetry Publisher task (Buddy 1) - lowest priority (FR6).
 *
 *  Any task reports an event with telemetry_post_event(): the event is
 *  packed at once (1-4 bytes, proto.h) and queued on telemetry_q. Every
 *  100 ms this task forwards the queued events, refreshes the live fault
 *  bits and publishes one 18-byte telemetry frame through comms.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <math.h>

#include "port.h"

#include "app.h"
#include "barcode_map.h"
#include "comms.h"
#include "fault.h"
#include "imu.h"
#include "telemetry.h"
#include "ultrasonic.h"

#define CENTI_PER_UNIT       (100.0f)  /* m -> cm, ratio -> percent      */
#define HALVES_PER_UNIT      (2.0f)    /* deg -> 0.5 deg, mm -> 0.5 mm   */
#define UNIT_SCALE           (1.0f)    /* already in wire units          */
#define FULL_CIRCLE_DEG      (360)
#define MS_PER_TICK          (100U)    /* uptime field resolution        */
#define UPTIME_MASK          (0xFFFFU)
#define RANGE_MAX_CM         (510.0f)  /* 511 is reserved for no echo    */
#define RANGE_NO_ECHO_CM     (900.0f)  /* us_front_cm() reports 999      */
#define SCALE_LIMIT          (32767.0f) /* keeps lroundf() in int16 range */
#define LINE_POS_LEFT        (-1)      /* line_pos: line under left     */
#define LINE_POS_CENTRE      (0)
#define LINE_POS_RIGHT       (1)

static void     live_faults_refresh (float32_t front_cm);
static void     frame_fill (proto_telem_t * p_telem, float32_t front_cm);
static uint8_t  line_bits_get (motion_state_t const * p_state);
static uint8_t  barcode_bits_get (char letter);
static int32_t  scale_round (float32_t value, float32_t factor);
static void     event_queue_put (uint8_t const * p_frame, uint32_t len);

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
        uint8_t       event[PROTO_EVENT_MAX_LEN];
        uint8_t       frame[PROTO_TELEM_LEN];
        proto_telem_t telem;
        float32_t     front_cm  = 0.0f;
        INT           event_len = 0;

        period_wait(&next_us, PERIOD_TELEM_MS);
        heartbeat(HB_TELEM);

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
        front_cm = us_front_cm();
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
 * @param  front_cm  Latest front ultrasonic range (999 = no echo).
 */
static void
live_faults_refresh (float32_t front_cm)
{
    static fault_code_t const codes[] = { FC_IMU, FC_NO_ECHO, FC_WIFI_LOSS };
    bool const                b_active[] =
    {
        !imu_is_ok(),
        (front_cm >= RANGE_NO_ECHO_CM),
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
 * @param  front_cm  Latest front ultrasonic range (999 = no echo).
 */
static void
frame_fill (proto_telem_t * p_telem, float32_t front_cm)
{
    motion_state_t state;
    int32_t        heading = 0;
    uint32_t       status  = 0U;

    state_read(&state);

    if (safety_is_latched())
    {
        status |= PROTO_ST_LATCHED;
    }

    if (barcode_map_is_loaded())
    {
        status |= PROTO_ST_CFG_MQTT;
    }

    heading = scale_round(state.heading_deg, UNIT_SCALE);
    if (heading >= FULL_CIRCLE_DEG)
    {
        heading -= FULL_CIRCLE_DEG;               /* 359.6 rounds to 360   */
    }

    /* Each narrowing cast below keeps a value that proto_pack_telem()
     * clamps again to the field width, so out-of-range input is safe. */
    p_telem->state          = state.sys_state;
    p_telem->motion_class   = state.motion_class;
    p_telem->line_bits      = line_bits_get(&state);
    p_telem->barcode        = barcode_bits_get(state.last_barcode);
    p_telem->fault_mask     = fault_get_mask();
    p_telem->status         = (uint8_t) status;
    p_telem->speed_l_cms    = (int16_t) scale_round(state.speed_l_mps,
                                                    CENTI_PER_UNIT);
    p_telem->speed_r_cms    = (int16_t) scale_round(state.speed_r_mps,
                                                    CENTI_PER_UNIT);
    p_telem->duty_l_pct     = (int16_t) scale_round(state.duty_l,
                                                    CENTI_PER_UNIT);
    p_telem->duty_r_pct     = (int16_t) scale_round(state.duty_r,
                                                    CENTI_PER_UNIT);
    p_telem->pitch_half_deg = (int16_t) scale_round(state.pitch_deg,
                                                    HALVES_PER_UNIT);
    p_telem->roll_deg       = (int16_t) scale_round(state.roll_deg,
                                                    UNIT_SCALE);
    p_telem->heading_deg    = (uint16_t) heading;
    p_telem->turn_rate_dps  = (int16_t) scale_round(state.turn_rate_dps,
                                                    UNIT_SCALE);
    p_telem->range_cm       = (front_cm >= RANGE_NO_ECHO_CM) ?
                              PROTO_RANGE_NO_ECHO :
                              (uint16_t) scale_round(fminf(front_cm,
                                                           RANGE_MAX_CM),
                                                     UNIT_SCALE);
    p_telem->distance_cm    = (int16_t) scale_round(state.dist_m,
                                                    CENTI_PER_UNIT);
    p_telem->hump_half_mm   = (uint16_t) scale_round(state.hump_peak_mm,
                                                     HALVES_PER_UNIT);
    p_telem->uptime_ticks   = (uint16_t) ((now_ms() / MS_PER_TICK) &
                                          UPTIME_MASK);
}

/*!
 * @brief  The two digital line sensors as two bits (1 = black).
 *
 *  The Line task publishes pos (-1 / 0 / +1) and line_seen; together they
 *  give back both raw sensor levels:
 *  pos -1 left only, +1 right only, 0 + seen both, 0 + not seen neither.
 *
 * @param  p_state  Snapshot of the shared state record.
 * @return PROTO_LINE_LEFT / PROTO_LINE_RIGHT bits.
 */
static uint8_t
line_bits_get (motion_state_t const * p_state)
{
    bool const b_both = (LINE_POS_CENTRE == p_state->line_pos) &&
                        (p_state->line_seen);
    uint8_t    bits   = 0U;

    if ((LINE_POS_LEFT == p_state->line_pos) || b_both)
    {
        bits |= PROTO_LINE_LEFT;
    }

    if ((LINE_POS_RIGHT == p_state->line_pos) || b_both)
    {
        bits |= PROTO_LINE_RIGHT;
    }

    return (bits);
}

/*!
 * @brief  Last barcode as 3 bits: valid flag + (letter - 'A') for A-D.
 * @param  letter  Last decoded character (0 = none).
 * @return PROTO_BAR_VALID | index, or 0 for none or a letter outside A-D.
 */
static uint8_t
barcode_bits_get (char letter)
{
    uint8_t bits = 0U;

    if ((letter >= 'A') && (letter <= 'D'))
    {
        /* letter - 'A' is 0..3 here, so the result fits 3 bits. */
        bits = (uint8_t) (PROTO_BAR_VALID | (uint32_t) (letter - 'A'));
    }

    return (bits);
}

/*!
 * @brief  Multiply and round to the nearest integer.
 * @param  value   Value to scale.
 * @param  factor  Wire units per unit of value.
 * @return Rounded result, limited to the int16_t range; 0 for a NaN or
 *         infinite input (Barr-C 5.4.b.v).
 */
static int32_t
scale_round (float32_t value, float32_t factor)
{
    float32_t const scaled = value * factor;
    int32_t         result = 0;

    /* Clamp first so lroundf() never overflows; the packer clamps again
     * to each field's real width. */
    if (isfinite(scaled))
    {
        /* Clamped to +/-32767 first, so the result fits an int32_t. */
        result = (int32_t) lroundf(fmaxf(fminf(scaled, SCALE_LIMIT),
                                         -SCALE_LIMIT));
    }

    return (result);
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

/** @file    sensor_line.c
 *  @brief   Line / Barcode task (Buddy 3), see sensor_line.h.
 *
 *  Line region: CALIBRATE -> FOLLOW. Calibration (operator swipes black
 *  under each sensor within CAL_WINDOW_MS) only gates the power-on
 *  self-test; the reading is published in every state, so a missed
 *  calibration can never silently stop line following.
 *  Barcode region: barcode_poll() (barcode.c) every period.
 *
 *  Each sensor's black reading is held for LINE_BLACK_HOLD_MS to bridge
 *  white flicker at the line edge; junction detection and calibration
 *  use the raw reading, so the hold cannot make a false junction.
 *
 *  Shared data: the newest reading and the last-black time are single
 *  32-bit words with this task as the only writer; an aligned word load
 *  or store is a single access, so readers on either core never see a
 *  torn value.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <string.h>

#include "port.h"

#include "barcode.h"
#include "car_config.h"
#include "safety.h"
#include "sensor_line.h"
#include "systime.h"
#include "vehicle_state.h"

#define LATEST_BITS_MASK     (0x3U)
#define LATEST_STAMP_SHIFT   (16U)
#define STAMP_MASK           (0xFFFFU)
#define BOTH_BLACK           (VEHICLE_IR_LEFT | VEHICLE_IR_RIGHT)
#define ALL_EVENTS           (SENSOR_LINE_EVT_JUNCTION |                  \
                              SENSOR_LINE_EVT_REGAINED |                  \
                              SENSOR_LINE_EVT_SENSOR_FAIL)

typedef enum
{
    LS_CALIBRATE = 0,
    LS_FOLLOW
} line_state_t;

/** Black-hold state of one sensor. */
typedef struct
{
    uint32_t last_black_ms;
    bool     b_seen;         /* has seen black at least once              */
} black_hold_t;

static ID                g_evt_flg       = 0;
static volatile uint32_t g_latest        = 0U;    /* bits | stamp << 16   */
static volatile uint32_t g_last_black_ms = 0U;
static volatile bool     g_b_cal_request = true;  /* calibrate at boot    */
static volatile bool     g_b_ready       = false;

static black_hold_t      g_hold[2];             /* left, right          */

static uint8_t sensors_read (void);
static uint8_t black_hold (uint8_t raw, uint32_t now_ms);
static void    reading_publish (uint8_t bits, uint8_t raw, uint8_t prev_raw,
                                uint8_t prev_bits);

/*!
 * @brief  Create the event flag and the barcode queue. Call from
 *         usermain() before the tasks start.
 */
void
sensor_line_init (void)
{
    T_CFLG create;

    (void) memset(&create, 0, sizeof(create));
    create.flgatr  = TA_TFIFO | TA_WMUL;
    create.iflgptn = 0U;
    g_evt_flg      = tk_cre_flg(&create);
    barcode_init();
}

/*!
 * @brief  Line / Barcode task body (priority PRI_LINE, 5 ms).
 * @param  stacd    Start code (unused).
 * @param  p_exinf  Extended information (unused).
 */
void
sensor_line_task (INT stacd, void * p_exinf)
{
    uint64_t     next_us   = time_us_64();
    line_state_t state     = LS_CALIBRATE;
    uint32_t     cal_start = systime_now_ms();
    uint8_t      cal_seen  = 0U;
    uint8_t      prev_bits = 0U;
    uint8_t      prev_raw  = 0U;

    (void) stacd;
    (void) p_exinf;

    g_last_black_ms = systime_now_ms();

    for (;;)
    {
        uint8_t raw  = 0U;
        uint8_t bits = 0U;

        systime_period_wait(&next_us, PERIOD_LINE_MS);
        safety_heartbeat(SAFETY_HB_LINE);

        raw  = sensors_read();
        bits = black_hold(raw, systime_now_ms());
        reading_publish(bits, raw, prev_raw, prev_bits);
        prev_bits = bits;
        prev_raw  = raw;

        if (g_b_cal_request)
        {
            g_b_cal_request = false;
            g_b_ready       = false;
            cal_seen        = 0U;
            cal_start       = systime_now_ms();
            state           = LS_CALIBRATE;
        }

        if (LS_CALIBRATE == state)
        {
            /* bits is 0..3, so the OR stays within 2 bits. */
            cal_seen = (uint8_t) (cal_seen | raw);

            if ((BOTH_BLACK == cal_seen) && (0U == raw))
            {
                g_b_ready = true;                 /* both seen, now white */
                state     = LS_FOLLOW;
            }
            else if ((systime_now_ms() - cal_start) > CAL_WINDOW_MS)
            {
                (void) tk_set_flg(g_evt_flg, SENSOR_LINE_EVT_SENSOR_FAIL);
                state = LS_FOLLOW;                /* keep following       */
            }
            else
            {
                /* Waiting for the operator's swipe. */
            }
        }

        barcode_poll();
    }
}

/*!
 * @brief  Newest line reading (Motion Control).
 * @param  p_bits  Output VEHICLE_IR_* bits.
 * @return True if the reading is at most LINE_MAX_AGE_MS old.
 */
bool
sensor_line_read (uint8_t * p_bits)
{
    uint32_t const word  = g_latest;              /* one atomic load      */
    uint16_t const stamp = (uint16_t) ((word >> LATEST_STAMP_SHIFT) &
                                       STAMP_MASK);

    /* Masked to 2 bits. */
    *p_bits = (uint8_t) (word & LATEST_BITS_MASK);

    return (systime_stamp_age(stamp) <= LINE_MAX_AGE_MS);
}

/*!
 * @brief  Time since either sensor last saw black (line-lost check).
 * @return Milliseconds.
 */
uint32_t
sensor_line_black_age_ms (void)
{
    return (systime_now_ms() - g_last_black_ms);
}

/*!
 * @brief  Take and clear the pending line events (Navigation FSM).
 * @return SENSOR_LINE_EVT_* bits.
 */
uint32_t
sensor_line_events_take (void)
{
    UINT pattern = 0U;

    (void) tk_wai_flg(g_evt_flg, ALL_EVENTS, TWF_ORW | TWF_CLR, &pattern,
                      TMO_POL);

    return ((uint32_t) pattern);
}

/*!
 * @brief  Start a new calibration window (GP21 short press in READY).
 */
void
sensor_line_request_calibration (void)
{
    g_b_cal_request = true;
}

/*!
 * @brief  Report whether calibration has passed.
 * @return True once both sensors have seen black and then white.
 */
bool
sensor_line_is_ready (void)
{
    return (g_b_ready);
}

/*!
 * @brief  Read the two line sensors.
 * @return VEHICLE_IR_* bits (1 = black).
 */
static uint8_t
sensors_read (void)
{
    uint32_t bits = 0U;

    if (IR_BLACK_LEVEL == (port_gpio_get(PIN_IR_L) ? 1U : 0U))
    {
        bits |= VEHICLE_IR_LEFT;
    }

    if (IR_BLACK_LEVEL == (port_gpio_get(PIN_IR_R) ? 1U : 0U))
    {
        bits |= VEHICLE_IR_RIGHT;
    }

    /* At most two bits set. */
    return ((uint8_t) bits);
}

/*!
 * @brief  Hold each sensor's black reading for LINE_BLACK_HOLD_MS.
 * @param  raw     This raw reading, VEHICLE_IR_* bits.
 * @param  now_ms  Current time.
 * @return Held reading, VEHICLE_IR_* bits.
 */
static uint8_t
black_hold (uint8_t raw, uint32_t now_ms)
{
    static uint8_t const masks[2] = { VEHICLE_IR_LEFT, VEHICLE_IR_RIGHT };
    uint32_t             held     = 0U;
    uint32_t             idx      = 0U;

    for (idx = 0U; idx < 2U; idx++)
    {
        if (0U != (raw & masks[idx]))
        {
            g_hold[idx].last_black_ms = now_ms;
            g_hold[idx].b_seen        = true;
        }

        if (g_hold[idx].b_seen &&
            ((now_ms - g_hold[idx].last_black_ms) < LINE_BLACK_HOLD_MS))
        {
            held |= masks[idx];
        }
    }

    /* At most two bits set. */
    return ((uint8_t) held);
}

/*!
 * @brief  Publish a reading: newest word, last-black time, events, and
 *         the shared state record (only when the reading changes).
 * @param  bits       This reading, after the black hold.
 * @param  raw        This raw reading (junction detection).
 * @param  prev_raw   Previous raw reading.
 * @param  prev_bits  Previous held reading.
 */
static void
reading_publish (uint8_t bits, uint8_t raw, uint8_t prev_raw,
                 uint8_t prev_bits)
{
    uint32_t const stamp = systime_stamp();
    UINT           events = 0U;

    g_latest = ((uint32_t) bits & LATEST_BITS_MASK) |
               (stamp << LATEST_STAMP_SHIFT);

    if (0U != bits)
    {
        g_last_black_ms = systime_now_ms();
    }

    if ((0U != bits) && (0U == prev_bits))
    {
        events |= SENSOR_LINE_EVT_REGAINED;
    }

    if ((BOTH_BLACK == raw) && (BOTH_BLACK != prev_raw))
    {
        events |= SENSOR_LINE_EVT_JUNCTION;
    }

    if (0U != events)
    {
        (void) tk_set_flg(g_evt_flg, events);
    }

    if (bits != prev_bits)
    {
        vehicle_state_set_line(bits);
    }
}

/*** end of file ***/

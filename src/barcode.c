/** @file    barcode.c
 *  @brief   Barcode reader (Buddy 3), see barcode.h.
 *
 *  Tribunal 1 (Buddy 3): the GPIO interrupt only time-stamps edges into a
 *  bounded ring (hal.c); everything else runs here, in the Line / Barcode
 *  task:
 *   1. Collect: raw edges pass through the glitch filter (bar_filter.c),
 *      which removes the chatter a slowly passing edge causes. A capture
 *      starts on a white-to-black edge.
 *   2. End: once the barcode has fully passed the sensor. While the car
 *      drives: white for BAR_QUIET_MM of travel (so a slow car cannot cut
 *      a read short; BAR_STALL_TIMEOUT_US if it stops for good). On a
 *      hand swipe or open loop: white for BAR_QUIET_MS after at least
 *      BAR_EDGES_MIN edges, or BAR_IDLE_TIMEOUT_US without an edge.
 *   3. Check: a read during which the ring overflowed lost edges and is
 *      rejected. Otherwise code39.c accepts only a complete '*X*' symbol
 *      with a clear wide / narrow ratio in every character. Bars are
 *      measured by the distance the sensor travelled (encoder counts
 *      stamped on every edge), so speed changes during a read do not
 *      matter (Tribunal 1: "if the car changes speed mid-read the widths
 *      come out wrong"); by time when the car did not move far enough
 *      (hand swipe) or drives open loop.
 *   4. Post: the letter is looked up in the table (barcode_map.c); only
 *      a letter in the table is queued, as its 2-bit slot (bar_cmd_q).
 *  Every read is reported (event BARCODE). With BAR_LOG the element
 *  widths go to debug/barcode and a status report to event BAR_STATUS
 *  (Tribunal 1: save the timestamp and decode logs).
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <string.h>
#include <tk/tkernel.h>

#include "port.h"

#include "bar_filter.h"
#include "barcode.h"
#include "barcode_map.h"
#include "car_config.h"
#include "code39.h"
#include "hal.h"
#include "motion_control.h"
#include "motor_driver.h"
#include "proto.h"
#include "systime.h"
#include "telemetry.h"
#include "vehicle_state.h"

#define LEVEL_BIT            (BAR_FILTER_LEVEL_BIT)
#define QUEUE_DEPTH          (4U)
#define MBF_SLOT_BYTES       (8U)      /* 4-byte header + 1 byte, rounded */
#define COLLECT_CHUNK        (16U)     /* edges taken from the ring at once */
#define COLLECT_CHUNKS_MAX   (BAR_RING_LEN / COLLECT_CHUNK)  /* per period */
#define LETTER_FIRST         ('A')
#define US_PER_MS            (1000U)
#define STATUS_READS_MAX     (15U)     /* 4-bit reads field              */
#define STATUS_COUNT_MAX     (255U)    /* 8-bit count fields             */
#define SUB_PER_COUNT        (16.0f)   /* bar_filter_positions() units   */
#define MM_PER_M             (1000.0f)
#define TENTHS_PER_MM        (10.0f)
#define PI_F                 (3.14159265f)
/* 0.1 mm of travel per 1/16 encoder count. */
#define TENTHS_PER_SUB       ((PI_F * WHEEL_DIAMETER_M * MM_PER_M *           \
                               TENTHS_PER_MM) /                              \
                              (ENC_COUNTS_PER_REV * SUB_PER_COUNT))
#define POS_SHIFT_DIV        (2.0f)    /* positions are shifted left by 1 */
#define BARDBG_WIDTH_MAX     (2047.0f) /* 11-bit width field             */
#define ROUND_HALF           (0.5f)
#define YAW_SCALE            (1000.0f)
#define SETUP_LEN            (3U)      /* bar_filter_positions() setup:  */
#define SETUP_SIGN_L         (0U)      /* left encoder sign              */
#define SETUP_SIGN_R         (1U)      /* right encoder sign             */
#define SETUP_YAW            (2U)      /* sensor offset / track x 1000   */
/* 1/16 encoder counts per mm of travel. */
#define SUB_PER_MM           ((ENC_COUNTS_PER_REV * SUB_PER_COUNT) /         \
                              (PI_F * WHEEL_DIAMETER_M * MM_PER_M))

/** Running totals for the status report (task only). */
typedef struct
{
    uint32_t reads;          /* captures finished                         */
    uint32_t glitches;       /* edges removed by the glitch filter        */
    uint32_t sent_reads;     /* totals at the last report                 */
    uint32_t sent_glitches;
    uint32_t sent_edges;
    uint32_t sent_dropped;
    uint32_t sent_ms;
} bar_stats_t;

static ID          g_slot_q    = 0;
static uint32_t    g_edges[BAR_EDGE_BUF_LEN];          /* task only        */
static int32_t     g_cl[BAR_EDGE_BUF_LEN];   /* encoder counts per edge    */
static int32_t     g_cr[BAR_EDGE_BUF_LEN];
static uint32_t    g_pos[BAR_EDGE_BUF_LEN];  /* travel, decoder format     */
static uint32_t    g_n_edges   = 0U;                   /* 0 = waiting      */
/* A read is in progress (set by the task, read by Motion Control). */
static volatile bool g_b_reading = false;
static uint32_t    g_dropped0  = 0U;   /* ring drops when the read began   */
static uint8_t     g_dbg_seq   = 0U;                   /* task only        */
static bar_stats_t g_stats;                            /* task only        */

static void     edges_collect (void);
static bool     capture_is_done (void);
static uint32_t travel_since (uint32_t idx, int32_t const * p_setup);
static bool     travel_setup (int32_t * p_setup);
static void     capture_finish (void);
static uint8_t  capture_decode (bool * p_b_distance);
static void     capture_send (uint8_t letter, bool b_distance);
static void     status_send (void);
static int16_t  count_field (uint32_t count, uint32_t max);

/*!
 * @brief  Create the slot queue. Call from usermain() before the tasks
 *         start.
 */
void
barcode_init (void)
{
    T_CMBF create;

    (void) memset(&create, 0, sizeof(create));
    (void) memset(&g_stats, 0, sizeof(g_stats));
    create.mbfatr = TA_TFIFO;
    create.bufsz  = (SZ) (QUEUE_DEPTH * MBF_SLOT_BYTES);       /* 32 B    */
    create.maxmsz = (INT) sizeof(uint8_t);
    g_slot_q      = tk_cre_mbf(&create);
}

/*!
 * @brief  Collect new edges and finish a capture when it is complete.
 *         Called by the Line / Barcode task every period.
 */
void
barcode_poll (void)
{
    edges_collect();

    if (capture_is_done())
    {
        capture_finish();
    }

    g_b_reading = (g_n_edges > 0U);

    if ((0 != BAR_LOG) &&
        ((systime_now_ms() - g_stats.sent_ms) >= BAR_STATUS_MS))
    {
        g_stats.sent_ms = systime_now_ms();
        status_send();
    }
}

/*!
 * @brief  Take one barcode, if any (Navigation FSM).
 * @param  p_slot  Output slot 0..3 (= action wire code, PROTO_ACT_*).
 * @return True if a barcode was taken.
 */
bool
barcode_take (uint8_t * p_slot)
{
    return (tk_rcv_mbf(g_slot_q, p_slot, TMO_POL) > 0);
}

/*!
 * @brief  Report whether a barcode is being read: from the read's first
 *         edge until it is decoded or rejected. A stray edge counts too
 *         and ends after BAR_QUIET_MM of white. Safe from any task.
 * @return True while a read is in progress.
 */
bool
barcode_is_reading (void)
{
    return (g_b_reading);
}

/*!
 * @brief  Move the edges from the interrupt ring through the glitch
 *         filter into the capture, COLLECT_CHUNK at a time (bounded
 *         stack), at most one ring's worth per period (bounded time: a
 *         chattering sensor must not hold up the line reading that shares
 *         this task; the rest waits for the next period).
 */
static void
edges_collect (void)
{
    uint32_t fresh[COLLECT_CHUNK];
    int32_t  fresh_cl[COLLECT_CHUNK];
    int32_t  fresh_cr[COLLECT_CHUNK];
    uint32_t chunks = 1U;
    uint32_t count  = hal_bar_take(fresh, fresh_cl, fresh_cr, COLLECT_CHUNK);

    while (count > 0U)
    {
        uint32_t idx = 0U;

        for (idx = 0U; idx < count; idx++)
        {
            uint32_t const before = g_n_edges;

            g_n_edges = bar_filter_add(g_edges, g_n_edges, BAR_EDGE_BUF_LEN,
                                       fresh[idx], BAR_GLITCH_US);

            if (g_n_edges < before)
            {
                g_stats.glitches++;
            }
            else if (g_n_edges > before)
            {
                /* Kept: keep its distance stamp with it. */
                g_cl[before] = fresh_cl[idx];
                g_cr[before] = fresh_cr[idx];

                if (0U == before)
                {
                    g_dropped0 = hal_bar_dropped();   /* a new read begins */
                }
            }
            else
            {
                /* Ignored (no level change, or the read is full). */
            }
        }

        if (chunks < COLLECT_CHUNKS_MAX)
        {
            count = hal_bar_take(fresh, fresh_cl, fresh_cr, COLLECT_CHUNK);
            chunks++;
        }
        else
        {
            count = 0U;                           /* next period          */
        }
    }
}

/*!
 * @brief  Decide whether the capture is complete (rules:
 *         bar_filter_is_done(), tested in web/test_proto.py).
 * @return True once the barcode has fully passed the sensor.
 */
static bool
capture_is_done (void)
{
    /* Constant expressions; the travel limits are a few thousand 1/16
     * counts, so the casts keep them. */
    static bar_end_limits_t const limits =
    {
        (uint32_t) BAR_MIN_TRAVEL_COUNTS * (uint32_t) SUB_PER_COUNT,
        (uint32_t) (BAR_QUIET_MM * SUB_PER_MM),
        BAR_STALL_TIMEOUT_US,
        BAR_EDGES_MIN,
        BAR_QUIET_MS * US_PER_MS,
        BAR_IDLE_TIMEOUT_US
    };
    bool b_done = false;

    if (g_n_edges > 0U)
    {
        uint32_t const  last = g_edges[g_n_edges - 1U];
        bar_end_state_t state;
        int32_t         setup[SETUP_LEN];

        state.n_edges = g_n_edges;
        /* Edges were collected before this time was read, so the
         * difference is never negative. */
        state.idle_us = port_time_us_32() - (last & ~LEVEL_BIT);
        state.moved   = 0U;
        state.quiet   = 0U;
        /* A long WIDE bar (level still black) must not end the read. */
        state.b_white = (0U == (last & LEVEL_BIT));

        if (travel_setup(setup))
        {
            state.moved = travel_since(0U, setup);
            state.quiet = travel_since(g_n_edges - 1U, setup);
        }

        b_done = bar_filter_is_done(&state, &limits);
    }

    return (b_done);
}

/*!
 * @brief  Sensor travel from one edge of the capture to now.
 * @param  idx      Edge (below g_n_edges).
 * @param  p_setup  travel_setup() values.
 * @return Distance, 1/16 counts (reversing counts too).
 */
static uint32_t
travel_since (uint32_t idx, int32_t const * p_setup)
{
    int32_t const moved = bar_filter_travel(
        motor_driver_count_left() - g_cl[idx],
        motor_driver_count_right() - g_cr[idx], p_setup);

    /* |moved| is a few thousand within one read, so it fits. */
    return ((uint32_t) ((moved < 0) ? -moved : moved));
}

/*!
 * @brief  Fill the bar_filter_positions() setup: encoder signs and the
 *         sensor's offset.
 * @param  p_setup  Output, SETUP_LEN values { sign L, sign R, yaw x 1000 }.
 * @return False if the encoders are not used (open loop): travel is then
 *         meaningless.
 */
static bool
travel_setup (int32_t * p_setup)
{
    bool const b_closed = !motion_control_is_open_loop();

    p_setup[SETUP_SIGN_L] = 1;
    p_setup[SETUP_SIGN_R] = 1;
    motion_control_encoder_signs(&p_setup[SETUP_SIGN_L],
                                 &p_setup[SETUP_SIGN_R]);
    /* Offset / track x 1000: a few hundred at most, fits. */
    p_setup[SETUP_YAW] = (int32_t) ((BAR_SENSOR_Y_MM * YAW_SCALE) /
                                    (TRACK_WIDTH_M * MM_PER_M));

    return (b_closed);
}

/*!
 * @brief  Check and decode the capture, look its letter up in the table,
 *         queue the slot (or report a letter not in the table, or an
 *         unreadable barcode), and wait for the next one.
 */
static void
capture_finish (void)
{
    bool          b_distance = false;
    uint8_t const letter     = capture_decode(&b_distance);
    /* n_edges <= BAR_EDGE_BUF_LEN (64), so it fits an int16_t. */
    int16_t const edges      = (int16_t) g_n_edges;

    g_stats.reads++;

    if (0 != BAR_LOG)
    {
        capture_send(letter, b_distance);
    }

    if (CODE39_NONE != letter)
    {
        /* letter is 0..25, so 'A' + letter is a valid character. */
        char const                 code   = (char) (LETTER_FIRST + letter);
        barcode_map_action_t const action = barcode_map_get(code);

        if (BA_NONE != action)
        {
            /* BA_STRAIGHT..BA_UTURN follow wire codes 0..3, so the slot
             * fits the 2-bit symbol field. */
            uint8_t const slot = (uint8_t) ((uint32_t) action -
                                            (uint32_t) BA_STRAIGHT);

            (void) tk_snd_mbf(g_slot_q, &slot, (INT) sizeof(slot), TMO_POL);
            vehicle_state_set_barcode((uint8_t) (VEHICLE_BAR_VALID | slot));
            telemetry_post_event(EV_BARCODE, slot, 0, edges);
        }
        else
        {
            /* The letter itself goes out on debug/barcode (BAR_LOG). */
            telemetry_post_event(EV_BARCODE, PROTO_BAR_EV_UNKNOWN, 0, edges);
        }
    }
    else
    {
        telemetry_post_event(EV_BARCODE, PROTO_BAR_EV_UNREADABLE, 0, edges);
    }

    g_n_edges = 0U;
}

/*!
 * @brief  Decode the capture: by the sensor's travel when the car moved
 *         at least BAR_MIN_TRAVEL_COUNTS with working encoders, else by
 *         time (hand swipe, open loop). A read that lost edges to a full ring
 *         is rejected (Tribunal 1: decide what happens on overflow).
 * @param  p_b_distance  Output: true if the travel was used.
 * @return Letter 0..25, or CODE39_NONE.
 */
static uint8_t
capture_decode (bool * p_b_distance)
{
    uint8_t letter = CODE39_NONE;

    *p_b_distance = false;

    if (hal_bar_dropped() == g_dropped0)
    {
        int32_t setup[SETUP_LEN];

        if (travel_setup(setup))
        {
            if (bar_filter_positions(g_edges, g_cl, g_cr, g_n_edges, setup,
                                     g_pos) >=
                ((uint32_t) BAR_MIN_TRAVEL_COUNTS * (uint32_t) SUB_PER_COUNT))
            {
                *p_b_distance = true;
                letter = code39_decode(g_pos, g_n_edges,
                                       BAR_MIN_WIDE_RATIO_X10);
            }
        }

        /* By time only without distance: while the car drives the speed
         * is never steady (0 of 156 wobbly emulator reads decode by
         * time), so a retry by time could only add a wrong letter. */
        if (!*p_b_distance)
        {
            letter = code39_decode(g_edges, g_n_edges,
                                   BAR_MIN_WIDE_RATIO_X10);
        }
    }

    return (letter);
}

/*!
 * @brief  Send the captured element widths and the decoded letter to the
 *         web app (BAR_LOG), PROTO_BARDBG_WIDTHS widths per frame: with a
 *         full capture, one frame per Code 39 character (9 elements, then
 *         the inter-character gap). The widths are the ones the decoder
 *         used: sensor travel (0.1 mm) or time (0.25 ms).
 * @param  letter      Decoded letter 0..25, or CODE39_NONE.
 * @param  b_distance  True to send travel, false for time.
 */
static void
capture_send (uint8_t letter, bool b_distance)
{
    uint32_t const * const p_src = b_distance ? g_pos : g_edges;
    proto_bar_debug_t dbg;
    uint8_t           frame[PROTO_BARDBG_MAX_LEN];
    uint32_t const    n_widths = (g_n_edges > 0U) ? (g_n_edges - 1U) : 0U;
    uint32_t          done     = 0U;

    (void) memset(&dbg, 0, sizeof(dbg));
    dbg.seq        = g_dbg_seq;
    dbg.b_distance = b_distance;
    dbg.b_letter   = (CODE39_NONE != letter);
    dbg.letter   = dbg.b_letter ? letter : 0U;

    do
    {
        uint32_t idx = 0U;

        dbg.count = 0U;

        for (idx = 0U; (idx < PROTO_BARDBG_WIDTHS) && (done < n_widths);
             idx++)
        {
            uint32_t const width = (p_src[done + 1U] & ~LEVEL_BIT) -
                                   (p_src[done] & ~LEVEL_BIT);

            if (b_distance)
            {
                /* width is in 1/16 counts shifted left by one (bit 0
                 * holds the level). Saturated to the 11-bit field, so
                 * the cast keeps it (a backward step shows as maximum). */
                float32_t const tenths = ((float32_t) width / POS_SHIFT_DIV) *
                                         TENTHS_PER_SUB;

                dbg.widths[idx] = (uint16_t) ((tenths < BARDBG_WIDTH_MAX) ?
                                              (tenths + ROUND_HALF) :
                                              BARDBG_WIDTH_MAX);
            }
            else
            {
                /* An element is shorter than BAR_IDLE_TIMEOUT_US plus one
                 * period (~505 ms = 2020 units), so it fits a uint16_t. */
                dbg.widths[idx] = (uint16_t) ((width +
                                               (PROTO_BARDBG_UNIT_US / 2U)) /
                                              PROTO_BARDBG_UNIT_US);
            }
            dbg.count++;
            done++;
        }

        dbg.b_last = (done >= n_widths);
        telemetry_post_debug(frame, proto_pack_bar_debug(&dbg, frame));
        dbg.chunk++;
    } while (!dbg.b_last);

    /* Masked to 3 bits, so it fits a uint8_t. */
    g_dbg_seq = (uint8_t) (((uint32_t) g_dbg_seq + 1U) &
                           PROTO_BARDBG_SEQ_MASK);
}

/*!
 * @brief  Report the barcode sensor once per BAR_STATUS_MS (BAR_LOG): its
 *         level now, whether the ring overflowed, and per period the raw
 *         edges, the glitches removed and the reads finished, so each
 *         step of the chain can be seen.
 */
static void
status_send (void)
{
    uint32_t const edges   = hal_bar_edges();
    uint32_t const dropped = hal_bar_dropped();
    uint32_t const black   = (IR_BLACK_LEVEL ==
                              (port_gpio_get(PIN_IR_BC) ? 1U : 0U)) ? 1U :
                                                                      0U;
    uint32_t const lost    = (dropped != g_stats.sent_dropped) ? 1U : 0U;
    /* Unsigned differences of running totals wrap cleanly. */
    uint32_t const reads   = g_stats.reads - g_stats.sent_reads;
    uint32_t const reads_sat = (reads > STATUS_READS_MAX) ? STATUS_READS_MAX :
                                                            reads;

    /* code: bit 0 black, bit 1 ring overflowed, bits 2..5 reads (6 bits,
     * so it fits a uint8_t). */
    telemetry_post_event(EV_BAR_STATUS,
                         (uint8_t) ((reads_sat << 2) | (lost << 1) | black),
                         count_field(edges - g_stats.sent_edges,
                                     STATUS_COUNT_MAX),
                         count_field(g_stats.glitches - g_stats.sent_glitches,
                                     STATUS_COUNT_MAX));

    g_stats.sent_edges    = edges;
    g_stats.sent_dropped  = dropped;
    g_stats.sent_reads    = g_stats.reads;
    g_stats.sent_glitches = g_stats.glitches;
}

/*!
 * @brief  Saturate a count for an event field.
 * @param  count  Count.
 * @param  max    Largest value of the field (at most 32767).
 * @return count, or max if larger.
 */
static int16_t
count_field (uint32_t count, uint32_t max)
{
    /* The result is at most max <= 32767, so it fits an int16_t. */
    return ((int16_t) ((count > max) ? max : count));
}

/*** end of file ***/

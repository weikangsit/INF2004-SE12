/** @file    barcode.c
 *  @brief   Barcode reader (Buddy 3), see barcode.h.
 *
 *  Tribunal 1 (Buddy 3): the GPIO interrupt only time-stamps edges into a
 *  bounded ring (hal.c); everything else runs here, in the Line / Barcode
 *  task:
 *   1. Collect: raw edges pass through the glitch filter (bar_filter.c),
 *      which removes the chatter a slowly passing edge causes. A capture
 *      starts on a white-to-black edge.
 *   2. End: after BAR_EDGES_FULL clean edges once the level has been
 *      steady for BAR_GLITCH_US (so a trailing glitch cannot end it), or
 *      after BAR_IDLE_TIMEOUT_US without an edge.
 *   3. Check: a read during which the ring overflowed lost edges and is
 *      rejected. Otherwise code39.c accepts only a complete '*X*' symbol
 *      with a clear wide / narrow ratio in every character.
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
#include "proto.h"
#include "systime.h"
#include "telemetry.h"
#include "vehicle_state.h"

#define LEVEL_BIT            (BAR_FILTER_LEVEL_BIT)
#define QUEUE_DEPTH          (4U)
#define MBF_SLOT_BYTES       (8U)      /* 4-byte header + 1 byte, rounded */
#define COLLECT_CHUNK        (16U)     /* edges taken from the ring at once */
#define LETTER_FIRST         ('A')
#define STATUS_READS_MAX     (15U)     /* 4-bit reads field              */
#define STATUS_COUNT_MAX     (255U)    /* 8-bit count fields             */

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
static uint32_t    g_n_edges   = 0U;                   /* 0 = waiting      */
static uint32_t    g_dropped0  = 0U;   /* ring drops when the read began   */
static uint8_t     g_dbg_seq   = 0U;                   /* task only        */
static bar_stats_t g_stats;                            /* task only        */

static void     edges_collect (void);
static bool     capture_is_done (void);
static void     capture_finish (void);
static void     capture_send (uint8_t letter);
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
 * @brief  Move the edges from the interrupt ring through the glitch
 *         filter into the capture, COLLECT_CHUNK at a time (bounded
 *         stack).
 */
static void
edges_collect (void)
{
    uint32_t fresh[COLLECT_CHUNK];
    uint32_t count = hal_bar_take(fresh, COLLECT_CHUNK);

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
            else if ((0U == before) && (1U == g_n_edges))
            {
                g_dropped0 = hal_bar_dropped();   /* a new read begins     */
            }
            else
            {
                /* Edge added to the read, or ignored. */
            }
        }

        count = hal_bar_take(fresh, COLLECT_CHUNK);
    }
}

/*!
 * @brief  Decide whether the capture is complete.
 * @return True after BAR_EDGES_FULL clean edges and BAR_GLITCH_US of
 *         steady level, or after BAR_IDLE_TIMEOUT_US without an edge.
 */
static bool
capture_is_done (void)
{
    bool b_done = false;

    if (g_n_edges > 0U)
    {
        /* Edges were collected before this time was read, so the
         * difference is never negative. */
        uint32_t const idle = port_time_us_32() -
                              (g_edges[g_n_edges - 1U] & ~LEVEL_BIT);

        b_done = ((g_n_edges >= BAR_EDGES_FULL) && (idle >= BAR_GLITCH_US)) ||
                 (idle > BAR_IDLE_TIMEOUT_US);
    }

    return (b_done);
}

/*!
 * @brief  Check and decode the capture, look its letter up in the table,
 *         queue the slot (or report a letter not in the table, or an
 *         unreadable barcode), and wait for the next one.
 */
static void
capture_finish (void)
{
    /* Edges lost in the ring during this read make the widths wrong
     * (Tribunal 1: decide what happens on overflow): reject the read. */
    bool const    b_lost = (hal_bar_dropped() != g_dropped0);
    uint8_t const letter = b_lost ? CODE39_NONE :
                           code39_decode(g_edges, g_n_edges,
                                         BAR_MIN_WIDE_RATIO_X10);
    /* n_edges <= BAR_EDGE_BUF_LEN (64), so it fits an int16_t. */
    int16_t const edges  = (int16_t) g_n_edges;

    g_stats.reads++;

    if (0 != BAR_LOG)
    {
        capture_send(letter);
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
 * @brief  Send the captured element widths and the decoded letter to the
 *         web app (BAR_LOG), PROTO_BARDBG_WIDTHS widths per frame: with a
 *         full capture, one frame per Code 39 character (9 elements, then
 *         the inter-character gap).
 * @param  letter  Decoded letter 0..25, or CODE39_NONE.
 */
static void
capture_send (uint8_t letter)
{
    proto_bar_debug_t dbg;
    uint8_t           frame[PROTO_BARDBG_MAX_LEN];
    uint32_t const    n_widths = (g_n_edges > 0U) ? (g_n_edges - 1U) : 0U;
    uint32_t          done     = 0U;

    (void) memset(&dbg, 0, sizeof(dbg));
    dbg.seq      = g_dbg_seq;
    dbg.b_letter = (CODE39_NONE != letter);
    dbg.letter   = dbg.b_letter ? letter : 0U;

    do
    {
        uint32_t idx = 0U;

        dbg.count = 0U;

        for (idx = 0U; (idx < PROTO_BARDBG_WIDTHS) && (done < n_widths);
             idx++)
        {
            uint32_t const width_us = (g_edges[done + 1U] & ~LEVEL_BIT) -
                                      (g_edges[done] & ~LEVEL_BIT);

            /* An element is shorter than BAR_IDLE_TIMEOUT_US plus one
             * period (~505 ms = 2020 units), so it fits a uint16_t. */
            dbg.widths[idx] = (uint16_t) ((width_us +
                                           (PROTO_BARDBG_UNIT_US / 2U)) /
                                          PROTO_BARDBG_UNIT_US);
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

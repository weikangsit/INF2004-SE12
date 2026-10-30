/** @file    barcode.c
 *  @brief   Barcode reader (Buddy 3), see barcode.h.
 *
 *  BAR_WAIT -> BAR_CAPTURE -> DECODE. The GPIO interrupt only time-stamps
 *  edges into a ring buffer (hal.c); this code runs in the Line / Barcode
 *  task. Capture starts on a white-to-black edge and ends after
 *  BAR_EDGES_FULL edges or BAR_IDLE_TIMEOUT_US without an edge. Edges
 *  beyond BAR_EDGE_BUF_LEN are dropped (the read then fails cleanly).
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <string.h>
#include <tk/tkernel.h>

#include "port.h"

#include "barcode.h"
#include "barcode_map.h"
#include "car_config.h"
#include "code39.h"
#include "hal.h"
#include "proto.h"
#include "systime.h"
#include "telemetry.h"
#include "vehicle_state.h"

#define LEVEL_BIT            (1U)      /* bit 0 of an edge = level after  */
#define QUEUE_DEPTH          (4U)
#define MBF_SLOT_BYTES       (8U)      /* 4-byte header + 1 byte, rounded */
#define LETTER_FIRST         ('A')
#define STATUS_COUNT_MAX     (15U)     /* 4-bit reads field              */
#define STATUS_VALUE_MAX     (255U)    /* 8-bit count fields             */

typedef enum
{
    BAR_WAIT = 0,
    BAR_CAPTURE
} bar_state_t;

static ID          g_slot_q   = 0;
static bar_state_t g_state    = BAR_WAIT;              /* task only        */
static uint32_t    g_edges[BAR_EDGE_BUF_LEN];          /* task only        */
static uint32_t    g_n_edges  = 0U;                    /* task only        */

static uint8_t     g_dbg_seq  = 0U;                    /* task only        */
static uint32_t    g_reads    = 0U;                    /* task only        */
static uint32_t    g_status_ms = 0U;                   /* task only        */
static uint32_t    g_prev_irqs  = 0U;                  /* task only        */
static uint32_t    g_prev_kept  = 0U;                  /* task only        */
static uint32_t    g_prev_reads = 0U;                  /* task only        */

static void edges_collect (void);
static void capture_finish (void);
static void capture_send (uint8_t letter);
static void status_send (void);

/*!
 * @brief  Create the letter queue. Call from usermain() before the tasks
 *         start.
 */
void
barcode_init (void)
{
    T_CMBF create;

    (void) memset(&create, 0, sizeof(create));
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

    if (BAR_CAPTURE == g_state)
    {
        uint32_t const last = g_edges[g_n_edges - 1U] & ~LEVEL_BIT;
        uint32_t const idle = port_time_us_32() - last;

        if ((g_n_edges >= BAR_EDGES_FULL) || (idle > BAR_IDLE_TIMEOUT_US))
        {
            capture_finish();
        }
    }

    if ((0 != BAR_LOG) &&
        ((systime_now_ms() - g_status_ms) >= BAR_STATUS_MS))
    {
        g_status_ms = systime_now_ms();
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
 * @brief  Move edges from the interrupt ring buffer into the capture.
 */
static void
edges_collect (void)
{
    uint32_t       fresh[BAR_EDGE_BUF_LEN];
    uint32_t const count = hal_bar_take(fresh, BAR_EDGE_BUF_LEN);
    uint32_t       idx   = 0U;

    for (idx = 0U; idx < count; idx++)
    {
        if (BAR_WAIT == g_state)
        {
            if (0U != (fresh[idx] & LEVEL_BIT))        /* white -> black   */
            {
                g_edges[0] = fresh[idx];
                g_n_edges  = 1U;
                g_state    = BAR_CAPTURE;
            }
        }
        else if (g_n_edges < BAR_EDGE_BUF_LEN)
        {
            g_edges[g_n_edges] = fresh[idx];
            g_n_edges++;
        }
        else
        {
            /* Overflow: drop the edge; the decode will reject the read. */
        }
    }
}

/*!
 * @brief  Decode the captured symbol, look its letter up in the table,
 *         queue the slot (or report a letter not in the table, or an
 *         unreadable barcode), and wait for the next one.
 */
static void
capture_finish (void)
{
    uint8_t const letter = code39_decode(g_edges, g_n_edges,
                                         BAR_MIN_WIDE_RATIO_X10);
    /* n_edges <= BAR_EDGE_BUF_LEN (64), so it fits an int16_t. */
    int16_t const edges  = (int16_t) g_n_edges;

    g_reads++;

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

    g_state   = BAR_WAIT;
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
 *         level now, and per period the pin interrupts, the edges kept
 *         and the reads finished, so each step of the chain can be seen.
 */
static void
status_send (void)
{
    uint32_t       irqs   = 0U;
    uint32_t       kept   = 0U;
    uint32_t const reads  = g_reads - g_prev_reads;
    uint32_t const reads_sat = (reads > STATUS_COUNT_MAX) ? STATUS_COUNT_MAX :
                                                            reads;
    uint32_t const black  = (IR_BLACK_LEVEL ==
                             (port_gpio_get(PIN_IR_BC) ? 1U : 0U)) ? 1U : 0U;
    uint32_t       d_irqs = 0U;
    uint32_t       d_kept = 0U;

    hal_bar_stats(&irqs, &kept);
    d_irqs = irqs - g_prev_irqs;                  /* wraps cleanly        */
    d_kept = kept - g_prev_kept;
    g_prev_irqs  = irqs;
    g_prev_kept  = kept;
    g_prev_reads = g_reads;

    /* Saturated first, so the narrowing casts keep the values (5 bits for
     * the code, 8 bits for the counts); the packer clamps again. */
    telemetry_post_event(EV_BAR_STATUS,
                         (uint8_t) ((reads_sat << 1) | black),
                         (int16_t) ((d_irqs > STATUS_VALUE_MAX) ?
                                    STATUS_VALUE_MAX : d_irqs),
                         (int16_t) ((d_kept > STATUS_VALUE_MAX) ?
                                    STATUS_VALUE_MAX : d_kept));
}

/*** end of file ***/

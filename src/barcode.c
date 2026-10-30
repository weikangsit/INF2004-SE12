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
#include "car_config.h"
#include "code39.h"
#include "hal.h"
#include "proto.h"
#include "telemetry.h"
#include "vehicle_state.h"

#define LEVEL_BIT            (1U)      /* bit 0 of an edge = level after  */
#define QUEUE_DEPTH          (4U)
#define MBF_SLOT_BYTES       (8U)      /* 4-byte header + 1 byte, rounded */
#define LETTER_FIRST         ('A')
#define UNREADABLE           ('?')

typedef enum
{
    BAR_WAIT = 0,
    BAR_CAPTURE
} bar_state_t;

static ID          g_letter_q = 0;
static bar_state_t g_state    = BAR_WAIT;              /* task only        */
static uint32_t    g_edges[BAR_EDGE_BUF_LEN];          /* task only        */
static uint32_t    g_n_edges  = 0U;                    /* task only        */

static void edges_collect (void);
static void capture_finish (void);

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
    g_letter_q    = tk_cre_mbf(&create);
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
}

/*!
 * @brief  Take one decoded letter, if any (Navigation FSM).
 * @param  p_letter  Output letter index, 0..3 for A..D.
 * @return True if a letter was taken.
 */
bool
barcode_take (uint8_t * p_letter)
{
    return (tk_rcv_mbf(g_letter_q, p_letter, TMO_POL) > 0);
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
 * @brief  Decode the captured symbol, queue the letter or report an
 *         unreadable barcode, and wait for the next one.
 */
static void
capture_finish (void)
{
    uint8_t const letter = code39_decode(g_edges, g_n_edges,
                                         BAR_MIN_WIDE_RATIO_X10);

    if (CODE39_NONE != letter)
    {
        /* letter is 0..3, so the packed value fits 3 bits. */
        (void) tk_snd_mbf(g_letter_q, &letter, (INT) sizeof(letter),
                          TMO_POL);
        vehicle_state_set_barcode((uint8_t) (VEHICLE_BAR_VALID | letter));
    }
    else
    {
        /* n_edges <= BAR_EDGE_BUF_LEN (64), so it fits an int16_t. */
        telemetry_post_event(EV_BARCODE, (uint8_t) UNREADABLE, -1,
                             (int16_t) g_n_edges);
    }

    g_state   = BAR_WAIT;
    g_n_edges = 0U;
}

/*** end of file ***/

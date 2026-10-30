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
#include <tm/tmonitor.h>

#include "port.h"

#include "barcode.h"
#include "barcode_map.h"
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
#define CHAR_STRIDE          (10U)     /* 9 elements + inter-char gap     */
#define NO_LETTER            ('-')

typedef enum
{
    BAR_WAIT = 0,
    BAR_CAPTURE
} bar_state_t;

static ID          g_slot_q   = 0;
static bar_state_t g_state    = BAR_WAIT;              /* task only        */
static uint32_t    g_edges[BAR_EDGE_BUF_LEN];          /* task only        */
static uint32_t    g_n_edges  = 0U;                    /* task only        */

static void edges_collect (void);
static void capture_finish (void);
static void capture_log (void);

/* Action names by wire code (PROTO_ACT_*), for the console log. */
static char const * const g_action_names[PROTO_ACTION_COUNT] =
{
    "straight", "left", "right", "U-turn"
};

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

    if (0 != BAR_LOG)
    {
        capture_log();
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
            tm_printf((UB *) "barcode: read %c -> %s\n", (int) code,
                      g_action_names[slot]);
        }
        else
        {
            /* Only the slot goes over MQTT; the letter is logged here. */
            tm_printf((UB *) "barcode: read %c, not in the table\n",
                      (int) code);
            barcode_log_table("in use");
            telemetry_post_event(EV_BARCODE, PROTO_BAR_EV_UNKNOWN, 0, edges);
        }
    }
    else
    {
        tm_printf((UB *) "barcode: unreadable (%d edges; a full '*X*' "
                  "symbol has %d)\n", (int) edges, (int) BAR_EDGES_FULL);
        telemetry_post_event(EV_BARCODE, PROTO_BAR_EV_UNREADABLE, 0, edges);
    }

    g_state   = BAR_WAIT;
    g_n_edges = 0U;
}

/*!
 * @brief  Print the barcode table on the USB console.
 * @param  p_why  Short note printed with it (e.g. "received").
 */
void
barcode_log_table (char const * p_why)
{
    uint32_t const table = barcode_map_table();
    char           letters[PROTO_ACTION_COUNT];
    uint32_t       action = 0U;

    for (action = 0U; action < PROTO_ACTION_COUNT; action++)
    {
        uint32_t const code = (table >> (PROTO_CFG_LETTER_SHIFT +
                                         (PROTO_CFG_LETTER_BITS * action))) &
                              PROTO_CFG_LETTER_MASK;

        /* A valid entry holds 0..25, so 'A' + code is a letter. */
        letters[action] = (0U != (table & (1U << action))) ?
                          (char) ((uint32_t) LETTER_FIRST + code) : NO_LETTER;
    }

    tm_printf((UB *) "barcode table %s (%s): straight %c, left %c, "
              "right %c, U-turn %c\n", p_why,
              barcode_map_is_loaded() ? "from web" : "built-in",
              (int) letters[PROTO_ACT_STRAIGHT], (int) letters[PROTO_ACT_LEFT],
              (int) letters[PROTO_ACT_RIGHT], (int) letters[PROTO_ACT_UTURN]);
}

/*!
 * @brief  Print the captured element widths on the USB console, one line
 *         per Code 39 character (9 elements, then the inter-character
 *         gap). In a clean read each line has 3 wide values, clearly
 *         (1.5x or more) above its 6 narrow ones.
 */
static void
capture_log (void)
{
    uint32_t idx = 0U;

    tm_printf((UB *) "barcode: %d edges, element widths (us):",
              (int) g_n_edges);

    for (idx = 1U; idx < g_n_edges; idx++)
    {
        uint32_t const width = (g_edges[idx] & ~LEVEL_BIT) -
                               (g_edges[idx - 1U] & ~LEVEL_BIT);

        /* Element idx-1 starts a character every CHAR_STRIDE elements. */
        if (1U == (idx % CHAR_STRIDE))
        {
            tm_printf((UB *) "\n ");
        }

        /* Widths are below BAR_IDLE_TIMEOUT_US (150 ms), so they fit. */
        tm_printf((UB *) " %d", (int) width);
    }

    tm_printf((UB *) "\n");
}

/*** end of file ***/

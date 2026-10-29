/**
 * @file    line_barcode.c
 * @brief   Line / Barcode Task (Buddy 3) - Section 6.1.
 *
 *  Line region : CALIBRATE -> FOLLOW_LINE
 *                both white = straight ahead, one black = steer toward it,
 *                both black = junction. Both-white never counts as "lost".
 *  Barcode     : BAR_WAIT -> BAR_CAPTURE -> DECODE  (barcode sensor only;
 *                the car reacts only to a fully decoded '*X*' symbol)
 *
 *  line_pos_t.pos convention: -1 = line under the LEFT sensor (steer
 *  left), +1 = line under the RIGHT sensor (steer right), 0 = centred.
 */
#include <string.h>

#include "port.h"
#include "app.h"
#include "hal.h"
#include "line_barcode.h"

typedef enum { LS_IDLE = 0, LS_CALIBRATE, LS_FOLLOW, LS_ERROR } line_state_t;
typedef enum { BS_WAIT = 0, BS_CAPTURE } bar_state_t;

#define C39_ELEMENTS   (9U)
#define C39_SYMBOL_EL  (29U)

typedef struct
{
    char        ch;
    const char *p_pattern;   /* bar,space,bar,... 1 = wide */
} c39_entry_t;

/* Code 39 start/stop and A-Z. Element order: bar, space, bar, ... bar.
 * Reversed '*' reads as 'P', so a backwards scan can never pass the
 * forward start/stop check - it fails over to the reversed decode.    */
static const c39_entry_t g_c39[] =
{
    { '*', "010010100" },
    { 'A', "100001001" }, { 'B', "001001001" }, { 'C', "101001000" },
    { 'D', "000011001" }, { 'E', "100011000" }, { 'F', "001011000" },
    { 'G', "000001101" }, { 'H', "100001100" }, { 'I', "001001100" },
    { 'J', "000011100" }, { 'K', "100000011" }, { 'L', "001000011" },
    { 'M', "101000010" }, { 'N', "000010011" }, { 'O', "100010010" },
    { 'P', "001010010" }, { 'Q', "000000111" }, { 'R', "100000110" },
    { 'S', "001000110" }, { 'T', "000010110" }, { 'U', "110000001" },
    { 'V', "011000001" }, { 'W', "111000000" }, { 'X', "010010001" },
    { 'Y', "110010000" }, { 'Z', "011010000" }
};

static volatile bool g_cal_request = true;   /* calibrate at boot     */
static volatile bool g_ready;

void line_request_calibration(void) { g_cal_request = true; }
bool line_is_ready(void)            { return g_ready; }

static bool ir_black(uint32_t pin)
{
    return (port_gpio_get(pin) == IR_BLACK_LEVEL);
}

/* ------------------------------------------------------------------ */
/* Code 39                                                             */
/* ------------------------------------------------------------------ */
static char c39_decode_char(const uint32_t *p_w)
{
    char     pat[C39_ELEMENTS + 1U];
    uint32_t i;
    uint32_t k;
    uint32_t n;

    /* mark the 3 widest of 9 as wide - independent of speed */
    (void)memset(pat, '0', C39_ELEMENTS);
    pat[C39_ELEMENTS] = '\0';
    for (k = 0U; k < 3U; k++)
    {
        uint32_t best = 0U;
        uint32_t best_i = 0U;

        for (i = 0U; i < C39_ELEMENTS; i++)
        {
            if ((pat[i] == '0') && (p_w[i] > best))
            {
                best = p_w[i];
                best_i = i;
            }
        }
        pat[best_i] = '1';
    }

    for (n = 0U; n < (sizeof(g_c39) / sizeof(g_c39[0])); n++)
    {
        if (0 == strcmp(pat, g_c39[n].p_pattern))
        {
            return g_c39[n].ch;
        }
    }
    return '\0';
}

static char c39_decode_symbol(const uint32_t *p_w)
{
    char a = c39_decode_char(&p_w[0]);
    char b = c39_decode_char(&p_w[10]);   /* [9] = inter-char gap  */
    char c = c39_decode_char(&p_w[20]);   /* [19] = inter-char gap */

    if ((a == '*') && (c == '*') && (b >= 'A') && (b <= 'Z'))
    {
        return b;
    }
    return '\0';
}

/* Returns 'A'..'Z' or '\0'. Tries forward, then reversed read. */
static char barcode_decode(const uint32_t *p_ts, uint32_t n_edges)
{
    uint32_t w[C39_SYMBOL_EL];
    uint32_t r[C39_SYMBOL_EL];
    uint32_t i;
    char     ch;

    if (n_edges < (C39_SYMBOL_EL + 1U))
    {
        return '\0';
    }
    for (i = 0U; i < C39_SYMBOL_EL; i++)
    {
        w[i] = (p_ts[i + 1U] & ~1U) - (p_ts[i] & ~1U);
        r[C39_SYMBOL_EL - 1U - i] = w[i];
    }
    ch = c39_decode_symbol(w);
    if ('\0' == ch)
    {
        ch = c39_decode_symbol(r);
    }
    return ch;
}

/* ------------------------------------------------------------------ */
/* Task                                                                */
/* ------------------------------------------------------------------ */
void line_barcode_task(INT stacd, void *exinf)
{
    uint64_t     next_us = time_us_64();
    line_state_t ls = LS_IDLE;
    bar_state_t  bs = BS_WAIT;
    int8_t       prev_pos = 0;
    uint32_t     pos_since_ms = now_ms();
    bool         was_seen = false;
    bool         was_junction = false;
    uint32_t     t_state_ms = 0U;
    uint8_t      cal_seen = 0U;  /* bit0 L, bit1 R black seen */
    uint32_t     edges[BAR_EDGE_BUF_LEN];
    uint32_t     n_edges = 0U;
    uint32_t     tmp[BAR_EDGE_BUF_LEN];

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        bool    l;
        bool    rr;

        period_wait(&next_us, PERIOD_LINE_MS);
        heartbeat(HB_LINE);
        l  = ir_black(PIN_IR_L);
        rr = ir_black(PIN_IR_R);

        /* raw sensor readings for telemetry, published in every state:
         * one bit per sensor (Change Requirement 1)                      */
        state_lock();
        state_raw()->ir_black = (uint8_t)((l ? LINE_IR_LEFT : 0U) |
                                          (rr ? LINE_IR_RIGHT : 0U));
        state_unlock();

        if (g_cal_request)
        {
            g_cal_request = false;
            g_ready = false;
            cal_seen = 0U;
            t_state_ms = now_ms();
            ls = LS_CALIBRATE;
        }

        /* ---------------- line region ---------------- */
        /* Calibration only gates the power-on self-test (g_ready). The
         * line position is computed and sent to Motion Control in EVERY
         * state, so a missed or repeated calibration can never silently
         * turn line following off.                                      */
        switch (ls)
        {
            case LS_CALIBRATE:
                /* operator swipes a black patch under each LINE sensor.
                 * The barcode sensor is not part of this: it only ever
                 * reads barcodes (barcode region below).               */
                if (l)                      { cal_seen |= 0x01U; }
                if (rr)                     { cal_seen |= 0x02U; }
                if ((0x03U == cal_seen) && !l && !rr)
                {
                    g_ready = true;
                    ls = LS_FOLLOW;
                }
                else if ((now_ms() - t_state_ms) > CAL_WINDOW_MS)
                {
                    if (!g_ready)
                    {
                        (void)tk_set_flg(g_line_evt_flg, LEVT_SENSOR_FAIL);
                    }
                    ls = LS_FOLLOW;    /* keep following regardless */
                }
                break;

            case LS_FOLLOW:
            case LS_IDLE:
            case LS_ERROR:
            default:
                break;
        }

        {
                /* The two sensors straddle the line:
                 *   both white  -> centred: drive straight ahead (pos 0)
                 *   left black  -> pos -1, Motion steers LEFT (slower)
                 *   right black -> pos +1, Motion steers RIGHT (slower)
                 *   (direction set by LINE_STEER_DIR in car_config.h)
                 *   both black  -> junction                              */
                line_pos_t msg;

                msg.junction  = l && rr;
                msg.line_seen = l || rr;
                msg.pos = (l && !rr) ? -1 : ((rr && !l) ? 1 : 0);

                if (msg.line_seen && !was_seen)
                {
                    (void)tk_set_flg(g_line_evt_flg, LEVT_REGAINED);
                }
                if (msg.junction && !was_junction)
                {
                    (void)tk_set_flg(g_line_evt_flg, LEVT_JUNCTION);
                }
                was_seen = msg.line_seen;
                was_junction = msg.junction;

                if (msg.pos != prev_pos)
                {
                    prev_pos = msg.pos;
                    pos_since_ms = now_ms();
                }
                msg.state_ms = now_ms() - pos_since_ms;
                (void)tk_snd_mbf(g_line_pos_q, &msg, (INT)sizeof(msg), TMO_POL);

                state_lock();
                state_raw()->line_pos  = msg.pos;
                state_raw()->line_seen = msg.line_seen;
                state_unlock();
        }

        /* ---------------- barcode region ---------------- */
        {
            uint32_t n = hal_bar_take(tmp, BAR_EDGE_BUF_LEN);
            uint32_t i;

            for (i = 0U; i < n; i++)
            {
                if (BS_WAIT == bs)
                {
                    if (1U == (tmp[i] & 1U))     /* white -> black */
                    {
                        edges[0] = tmp[i];
                        n_edges = 1U;
                        bs = BS_CAPTURE;
                    }
                }
                else if (n_edges < BAR_EDGE_BUF_LEN)
                {
                    edges[n_edges] = tmp[i];
                    n_edges++;
                }
                else
                {
                    /* overflow: drop */
                }
            }

            if (BS_CAPTURE == bs)
            {
                uint32_t idle = port_time_us_32() - (edges[n_edges - 1U] & ~1U);

                if ((n_edges >= BAR_EDGES_FULL) || (idle > BAR_IDLE_TIMEOUT_US))
                {
                    char ch = barcode_decode(edges, n_edges);

                    if ('\0' != ch)
                    {
                        bar_cmd_t cmd = ch;

                        (void)tk_snd_mbf(g_bar_cmd_q, &cmd, (INT)sizeof(cmd), TMO_POL);
                        state_lock();
                        state_raw()->last_barcode = ch;
                        state_unlock();
                    }
                    else
                    {
                        telemetry_post_event(EV_BARCODE, (uint8_t)'?', -1, (int16_t)n_edges);
                    }
                    bs = BS_WAIT;
                    n_edges = 0U;
                }
            }
        }
    }
}

/** @file    stub_kernel.c  (TEST STUB)
 *  @brief   Fake micro T-Kernel (event flags, message buffers, mutexes), a
 *           fake clock and tm_printf(), for tests that run on the PC.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "port.h"
#include "stub_kernel.h"

#define MAX_FLAGS    (16)
#define MAX_MBFS     (16)
#define MAX_MSGS     (64)
#define MAX_MSG_LEN  (64)
#define LOG_LEN      (16384)
#define MBF_HEADER   (4)        /* T-Kernel stores a 4-byte length first */
#define MBF_ROUND    (4)

typedef struct
{
    bool b_used;
    UINT pattern;
} flag_t;

typedef struct
{
    INT     len;
    uint8_t data[MAX_MSG_LEN];
} msg_t;

typedef struct
{
    bool  b_used;
    SZ    bufsz;
    INT   maxmsz;
    SZ    used_bytes;
    int   head;
    int   count;
    msg_t msgs[MAX_MSGS];
} mbf_t;

static flag_t   g_flags[MAX_FLAGS + 1];
static mbf_t    g_mbfs[MAX_MBFS + 1];
static int      g_mtx_count = 0;
static uint64_t g_now_us    = 0U;
static char     g_log[LOG_LEN];

bool stub_gpio[32];
int  stub_nonpoll_calls = 0;

void
stub_kernel_reset (void)
{
    (void) memset(g_flags, 0, sizeof(g_flags));
    (void) memset(g_mbfs, 0, sizeof(g_mbfs));
    (void) memset(g_log, 0, sizeof(g_log));
    (void) memset(stub_gpio, 0, sizeof(stub_gpio));
    g_mtx_count        = 0;
    g_now_us           = 0U;
    stub_nonpoll_calls = 0;
}

void
stub_time_set_ms (uint32_t ms)
{
    g_now_us = (uint64_t) ms * 1000U;
}

void
stub_time_advance_ms (uint32_t ms)
{
    g_now_us += (uint64_t) ms * 1000U;
}

bool
stub_log_contains (char const * p_text)
{
    return (NULL != strstr(g_log, p_text));
}

/* ---------------------------------------------------------- clock, port */
uint64_t
time_us_64 (void)
{
    return (g_now_us);
}

uint32_t
port_time_us_32 (void)
{
    return ((uint32_t) g_now_us);
}

void
port_busy_wait_us (uint32_t delay_us)
{
    g_now_us += delay_us;
}

bool
port_gpio_get (uint32_t pin)
{
    return ((pin < 32U) ? stub_gpio[pin] : false);
}

/* -------------------------------------------------------------- tm_printf */
int
tm_printf (UB const * p_format, ...)
{
    char    line[256];
    va_list args;
    int     len = 0;

    va_start(args, p_format);
    len = vsnprintf(line, sizeof(line), (char const *) p_format, args);
    va_end(args);

    if ((strlen(g_log) + strlen(line) + 1U) < sizeof(g_log))
    {
        (void) strcat(g_log, line);
    }

    if (NULL != getenv("CAR_TEST_VERBOSE"))
    {
        (void) fputs(line, stdout);
    }

    return (len);
}

/* ------------------------------------------------------------ event flags */
ID
tk_cre_flg (T_CFLG const * p_cflg)
{
    ID id = 0;
    ID k  = 0;

    for (k = 1; (k <= MAX_FLAGS) && (0 == id); k++)
    {
        if (!g_flags[k].b_used)
        {
            g_flags[k].b_used  = true;
            g_flags[k].pattern = p_cflg->iflgptn;
            id                 = k;
        }
    }

    return ((0 != id) ? id : E_ID);
}

ER
tk_set_flg (ID flgid, UINT setptn)
{
    ER result = E_ID;

    if ((flgid > 0) && (flgid <= MAX_FLAGS) && g_flags[flgid].b_used)
    {
        g_flags[flgid].pattern |= setptn;
        result = E_OK;
    }

    return (result);
}

ER
tk_clr_flg (ID flgid, UINT clrptn)
{
    ER result = E_ID;

    if ((flgid > 0) && (flgid <= MAX_FLAGS) && g_flags[flgid].b_used)
    {
        g_flags[flgid].pattern &= clrptn;     /* keeps the 1 bits */
        result = E_OK;
    }

    return (result);
}

ER
tk_wai_flg (ID flgid, UINT waiptn, UINT wfmode, UINT * p_flgptn, TMO tmout)
{
    ER result = E_ID;

    if (TMO_POL != tmout)
    {
        stub_nonpoll_calls++;
    }

    if ((flgid > 0) && (flgid <= MAX_FLAGS) && g_flags[flgid].b_used)
    {
        UINT const now = g_flags[flgid].pattern;

        if (0U != (now & waiptn))                 /* OR wait */
        {
            *p_flgptn = now;                      /* pattern before clear */

            if (0U != (wfmode & TWF_CLR))
            {
                g_flags[flgid].pattern = 0U;
            }
            else if (0U != (wfmode & TWF_BITCLR))
            {
                g_flags[flgid].pattern &= ~waiptn;
            }
            else
            {
                /* No clear. */
            }

            result = E_OK;
        }
        else
        {
            result = E_TMOUT;
        }
    }

    return (result);
}

ER
tk_ref_flg (ID flgid, T_RFLG * p_rflg)
{
    ER result = E_ID;

    if ((flgid > 0) && (flgid <= MAX_FLAGS) && g_flags[flgid].b_used)
    {
        p_rflg->flgptn = g_flags[flgid].pattern;
        p_rflg->wtsk   = 0;
        result         = E_OK;
    }

    return (result);
}

/* -------------------------------------------------------- message buffers */
ID
tk_cre_mbf (T_CMBF const * p_cmbf)
{
    ID id = 0;
    ID k  = 0;

    for (k = 1; (k <= MAX_MBFS) && (0 == id); k++)
    {
        if (!g_mbfs[k].b_used)
        {
            (void) memset(&g_mbfs[k], 0, sizeof(g_mbfs[k]));
            g_mbfs[k].b_used = true;
            g_mbfs[k].bufsz  = p_cmbf->bufsz;
            g_mbfs[k].maxmsz = p_cmbf->maxmsz;
            id               = k;
        }
    }

    return ((0 != id) ? id : E_ID);
}

/** Bytes one message takes in the buffer: 4-byte header + size rounded up. */
static SZ
mbf_cost (INT len)
{
    return ((SZ) MBF_HEADER +
            ((((SZ) len + MBF_ROUND - 1U) / MBF_ROUND) * MBF_ROUND));
}

ER
tk_snd_mbf (ID mbfid, void const * p_msg, INT msgsz, TMO tmout)
{
    ER result = E_ID;

    if (TMO_POL != tmout)
    {
        stub_nonpoll_calls++;
    }

    if ((mbfid > 0) && (mbfid <= MAX_MBFS) && g_mbfs[mbfid].b_used)
    {
        mbf_t * const p_mbf = &g_mbfs[mbfid];

        if ((msgsz <= 0) || (msgsz > p_mbf->maxmsz) || (msgsz > MAX_MSG_LEN))
        {
            result = E_PAR;
        }
        else if (((p_mbf->bufsz - p_mbf->used_bytes) < mbf_cost(msgsz)) ||
                 (p_mbf->count >= MAX_MSGS))
        {
            result = E_TMOUT;                     /* full, no waiting */
        }
        else
        {
            int const tail = (p_mbf->head + p_mbf->count) % MAX_MSGS;

            p_mbf->msgs[tail].len = msgsz;
            (void) memcpy(p_mbf->msgs[tail].data, p_msg, (size_t) msgsz);
            p_mbf->count++;
            p_mbf->used_bytes += mbf_cost(msgsz);
            result = E_OK;
        }
    }

    return (result);
}

INT
tk_rcv_mbf (ID mbfid, void * p_msg, TMO tmout)
{
    INT result = E_ID;

    if (TMO_POL != tmout)
    {
        stub_nonpoll_calls++;
    }

    if ((mbfid > 0) && (mbfid <= MAX_MBFS) && g_mbfs[mbfid].b_used)
    {
        mbf_t * const p_mbf = &g_mbfs[mbfid];

        if (0 == p_mbf->count)
        {
            result = E_TMOUT;                     /* empty, no waiting */
        }
        else
        {
            msg_t const * const p_first = &p_mbf->msgs[p_mbf->head];

            (void) memcpy(p_msg, p_first->data, (size_t) p_first->len);
            result = p_first->len;
            p_mbf->used_bytes -= mbf_cost(p_first->len);
            p_mbf->head = (p_mbf->head + 1) % MAX_MSGS;
            p_mbf->count--;
        }
    }

    return (result);
}

/* ------------------------------------------------------------- the rest */
ID
tk_cre_mtx (T_CMTX const * p_cmtx)
{
    (void) p_cmtx;
    g_mtx_count++;

    return (g_mtx_count);
}

ER
tk_loc_mtx (ID mtxid, TMO tmout)
{
    (void) mtxid;
    (void) tmout;

    return (E_OK);
}

ER
tk_unl_mtx (ID mtxid)
{
    (void) mtxid;

    return (E_OK);
}

ER
tk_dly_tsk (RELTIM dlytim)
{
    g_now_us += (uint64_t) dlytim * 1000U;

    return (E_OK);
}

ER
tk_slp_tsk (TMO tmout)
{
    (void) tmout;

    return (E_OK);
}

/*** end of file ***/

/**
 * @file    safety.c
 * @brief   Safety Monitor Task (priority 1) - FR7 / NFR3.
 *
 *  Every 50 ms: check all 7 task heartbeats and the collision /
 *  over-tilt / line-lost / e-stop events. On any fault it brakes the
 *  motors DIRECTLY (GP8-GP11 LOW), latches, and posts STOP.
 */

#include "port.h"

#include "app.h"
#include "hal.h"

#define BOOT_GRACE_MS   (2000U)
#define N_HB            (7U)

static volatile bool g_latched;

bool safety_is_latched(void) { return g_latched; }

void safety_clear(void)
{
    (void)tk_clr_flg(g_evt_flg,
                     ~(UINT)(EVT_COLLISION | EVT_LINE_LOST | EVT_ESTOP | EVT_OVERTILT));
    g_latched = false;
    fault_clear_all();   /* live bits are re-set by Telemetry within 100 ms */
}

static void trip(fault_code_t code, int16_t detail)
{
    hal_motor_brake();                 /* first, before anything else */
    if (!g_latched)
    {
        g_latched = true;
        stop();
        telemetry_post_event(EV_FAULT, (uint8_t)code, detail, 0);
    }
}

void safety_task(INT stacd, void *exinf)
{
    uint64_t next_us = time_us_64();
    uint32_t last_seen[N_HB];
    uint32_t boot_ms = now_ms();
    uint32_t i;

    (void)stacd;
    (void)exinf;

    for (i = 0U; i < N_HB; i++)
    {
        last_seen[i] = boot_ms;
    }

    for (;;)
    {
        UINT     hb = 0U;
        UINT     ev = 0U;
        uint32_t t;

        period_wait(&next_us, PERIOD_SAFETY_MS);
        t = now_ms();

        /* ---- heartbeats ---- */
        (void)tk_wai_flg(g_hb_flg, HB_ALL, TWF_ORW | TWF_CLR, &hb, TMO_POL);
        for (i = 0U; i < N_HB; i++)
        {
            if (0U == (HB_ALL & (1U << i)))
            {
                continue;                          /* not supervised */
            }
            if (0U != (hb & (1U << i)))
            {
                last_seen[i] = t;
            }
            else if (((t - boot_ms) > BOOT_GRACE_MS) &&
                     ((t - last_seen[i]) > SAFETY_HB_TIMEOUT_MS))
            {
                if (0U != (HB_WARN & (1U << i)))
                {
                    fault_set(FC_TELEM_LATE);         /* warning only */
                }
                else
                {
                    trip(FC_HEARTBEAT, (int16_t)i);   /* v1 = missing HB_* bit */
                }
            }
            else
            {
                /* within deadline */
            }
        }

        /* ---- safety events (leave other bits untouched) ---- */
        (void)tk_wai_flg(g_evt_flg,
                         EVT_COLLISION | EVT_LINE_LOST | EVT_ESTOP | EVT_OVERTILT,
                         TWF_ORW | TWF_BITCLR, &ev, TMO_POL);
        if (0U != (ev & EVT_COLLISION)) { trip(FC_COLLISION, 0); }
        if (0U != (ev & EVT_OVERTILT))  { trip(FC_OVERTILT, 0);  }
        if (0U != (ev & EVT_LINE_LOST)) { trip(FC_LINE_LOST, 0); }
        if (0U != (ev & EVT_ESTOP))     { trip(FC_ESTOP, 0);     }

        /* keep the brake asserted while latched */
        if (g_latched)
        {
            hal_motor_brake();
        }
    }
}

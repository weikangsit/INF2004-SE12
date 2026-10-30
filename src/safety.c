/** @file    safety.c
 *  @brief   Safety Monitor task (team), see safety.h.
 *
 *  Response time: a fault event wakes the task immediately (it waits on
 *  its event flag with a PERIOD_SAFETY_MS timeout, Tribunal 1 Version C);
 *  the brake follows in the same activation. Heartbeats are checked at
 *  least every PERIOD_SAFETY_MS.
 *
 *  Policy (Tribunal 1):
 *   - A missed deadline of Line, Motion, IMU, Ultrasonic or Nav brakes.
 *   - A missed Telemetry deadline is a warning only (FC_TELEM_LATE).
 *   - WiFi loss never brakes (comms.c reports FC_WIFI_LOSS), but a stalled
 *     radio task does: remote e-stop would no longer be possible. Its
 *     liveness is the template's radio poll counter, which keeps counting
 *     while WiFi is down.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <string.h>

#include "port.h"

#include "car_config.h"
#include "comms.h"
#include "fault.h"
#include "motion_control.h"
#include "motor_driver.h"
#include "proto.h"
#include "safety.h"
#include "systime.h"
#include "telemetry.h"

#define ALL_EVENTS           (SAFETY_EVT_COLLISION | SAFETY_EVT_OVERTILT |  \
                              SAFETY_EVT_LINE_LOST | SAFETY_EVT_ESTOP)
#define ALL_HEARTBEATS       (0x7FU)
#define TASK_COUNT           (7U)

/** Supervision of one task. */
typedef struct
{
    uint32_t bit;            /* SAFETY_HB_*                               */
    uint32_t deadline_ms;
    bool     b_warn_only;
} hb_rule_t;

static hb_rule_t const g_rules[TASK_COUNT] =
{
    { SAFETY_HB_LINE,   HB_DEADLINE_LINE_MS,   false },
    { SAFETY_HB_MOTION, HB_DEADLINE_MOTION_MS, false },
    { SAFETY_HB_IMU,    HB_DEADLINE_IMU_MS,    false },
    { SAFETY_HB_US,     HB_DEADLINE_US_MS,     false },
    { SAFETY_HB_NAV,    HB_DEADLINE_NAV_MS,    false },
    { SAFETY_HB_COMMS,  HB_DEADLINE_COMMS_MS,  false },
    { SAFETY_HB_TELEM,  HB_DEADLINE_TELEM_MS,  true  }
};

static ID            g_evt_flg   = 0;
static ID            g_hb_flg    = 0;
static volatile bool g_b_latched = false;

static void events_handle (uint32_t events);
static void heartbeats_check (uint32_t now, uint32_t boot_ms,
                              uint32_t * p_last_seen);
static void trip (fault_code_t code, uint32_t detail);
static ID   flag_create (void);

/*!
 * @brief  Create the event and heartbeat flags. Call from usermain()
 *         before the tasks start.
 */
void
safety_init (void)
{
    g_evt_flg = flag_create();
    g_hb_flg  = flag_create();
}

/*!
 * @brief  Safety Monitor task body (priority PRI_SAFETY).
 * @param  stacd    Start code (unused).
 * @param  p_exinf  Extended information (unused).
 */
void
safety_task (INT stacd, void * p_exinf)
{
    uint32_t const boot_ms = systime_now_ms();
    uint32_t       last_seen[TASK_COUNT];
    uint32_t       last_check = boot_ms;
    uint32_t       idx        = 0U;

    (void) stacd;
    (void) p_exinf;

    for (idx = 0U; idx < TASK_COUNT; idx++)
    {
        last_seen[idx] = boot_ms;
    }

    for (;;)
    {
        UINT     events = 0U;
        uint32_t now    = 0U;

        if (E_OK == tk_wai_flg(g_evt_flg, ALL_EVENTS, TWF_ORW | TWF_BITCLR,
                               &events, (TMO) PERIOD_SAFETY_MS))
        {
            events_handle((uint32_t) events);
        }

        now = systime_now_ms();
        if ((now - last_check) >= PERIOD_SAFETY_MS)
        {
            last_check = now;
            heartbeats_check(now, boot_ms, last_seen);
        }

        /* Keep the brake asserted while latched. */
        if (g_b_latched)
        {
            motor_driver_brake();
        }
    }
}

/*!
 * @brief  Report liveness (each supervised task, every period).
 * @param  task_bit  The caller's SAFETY_HB_* bit.
 */
void
safety_heartbeat (uint32_t task_bit)
{
    /* SAFETY_HB_* bits fit a UINT pattern. */
    (void) tk_set_flg(g_hb_flg, (UINT) task_bit);
}

/*!
 * @brief  Report fault events; the Safety Monitor wakes at once. Safe
 *         from any task.
 * @param  events  SAFETY_EVT_* bits.
 */
void
safety_report (uint32_t events)
{
    /* SAFETY_EVT_* bits fit a UINT pattern. */
    (void) tk_set_flg(g_evt_flg, (UINT) events);
}

/*!
 * @brief  Report the safety latch.
 * @return True from a trip until safety_clear().
 */
bool
safety_is_latched (void)
{
    return (g_b_latched);
}

/*!
 * @brief  Operator reset: clear pending events, the latch, the motor
 *         inhibit and the fault mask (live bits are re-set by Telemetry
 *         within 100 ms).
 */
void
safety_clear (void)
{
    (void) tk_clr_flg(g_evt_flg, ~(UINT) ALL_EVENTS);
    g_b_latched = false;
    motor_driver_inhibit(false);
    fault_clear_all();
}

/*!
 * @brief  Trip on every fault event received.
 * @param  events  SAFETY_EVT_* bits.
 */
static void
events_handle (uint32_t events)
{
    if (0U != (events & SAFETY_EVT_ESTOP))
    {
        trip(FC_ESTOP, 0U);
    }

    if (0U != (events & SAFETY_EVT_COLLISION))
    {
        trip(FC_COLLISION, 0U);
    }

    if (0U != (events & SAFETY_EVT_OVERTILT))
    {
        trip(FC_OVERTILT, 0U);
    }

    if (0U != (events & SAFETY_EVT_LINE_LOST))
    {
        trip(FC_LINE_LOST, 0U);
    }
}

/*!
 * @brief  Check every task's heartbeat against its deadline.
 * @param  now          Current time, ms.
 * @param  boot_ms      Start time of the task (grace period).
 * @param  p_last_seen  Last heartbeat time per rule (updated).
 */
static void
heartbeats_check (uint32_t now, uint32_t boot_ms, uint32_t * p_last_seen)
{
    UINT     beats = 0U;
    uint32_t idx   = 0U;

    (void) tk_wai_flg(g_hb_flg, ALL_HEARTBEATS, TWF_ORW | TWF_CLR, &beats,
                      TMO_POL);

    if (comms_radio_is_alive())
    {
        beats |= SAFETY_HB_COMMS;
    }

    for (idx = 0U; idx < TASK_COUNT; idx++)
    {
        hb_rule_t const * const p_rule = &g_rules[idx];

        if (0U != ((uint32_t) beats & p_rule->bit))
        {
            p_last_seen[idx] = now;
        }
        else if (((now - boot_ms) > SAFETY_BOOT_GRACE_MS) &&
                 ((now - p_last_seen[idx]) > p_rule->deadline_ms))
        {
            if (p_rule->b_warn_only)
            {
                fault_set(FC_TELEM_LATE);
            }
            else
            {
                trip(FC_HEARTBEAT, idx);          /* detail = task bit no. */
            }
        }
        else
        {
            /* Within its deadline. */
        }
    }
}

/*!
 * @brief  Inhibit and brake the motors first, then latch, stop Motion
 *         Control and report the fault (once per latch).
 * @param  code    Fault.
 * @param  detail  EV_FAULT detail (missing heartbeat task number).
 */
static void
trip (fault_code_t code, uint32_t detail)
{
    motor_driver_inhibit(true);                   /* brakes immediately   */

    if (!g_b_latched)
    {
        g_b_latched = true;
        motion_control_stop();
        /* code < 16 and detail < 7, so both fit. */
        telemetry_post_event(EV_FAULT, (uint8_t) code, (int16_t) detail, 0);
    }
}

/*!
 * @brief  Create an empty multi-wait event flag.
 * @return Flag ID.
 */
static ID
flag_create (void)
{
    T_CFLG create;

    (void) memset(&create, 0, sizeof(create));
    create.flgatr  = TA_TFIFO | TA_WMUL;
    create.iflgptn = 0U;

    return (tk_cre_flg(&create));
}

/*** end of file ***/

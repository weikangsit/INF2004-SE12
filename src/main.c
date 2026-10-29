/**
 * @file    main.c
 * @brief   usermain(): create every kernel object and task (Section 3/4).
 */
#include <string.h>

#include <tm/tmonitor.h>

#include "port.h"

#include "app.h"
#include "barcode_map.h"
#include "hal.h"
#include "comms.h"

/* ------------------------------------------------------------------ */
/* Kernel objects                                                      */
/* ------------------------------------------------------------------ */
ID g_line_pos_q;
ID g_bar_cmd_q;
ID g_motion_cmd_q;
ID g_motion_evt_q;
ID g_obstacle_q;
ID g_scan_req_q;
ID g_remote_cmd_q;
ID g_telemetry_q;
ID g_hb_flg;
ID g_evt_flg;
ID g_line_evt_flg;
ID g_state_mtx;

static motion_state_t g_state;

/* ------------------------------------------------------------------ */
/* Shared helpers                                                      */
/* ------------------------------------------------------------------ */
void state_lock(void)   { (void)tk_loc_mtx(g_state_mtx, TMO_FEVR); }
void state_unlock(void) { (void)tk_unl_mtx(g_state_mtx); }
motion_state_t *state_raw(void) { return &g_state; }

void state_read(motion_state_t *p_out)
{
    state_lock();
    *p_out = g_state;
    state_unlock();
}

uint32_t now_ms(void)
{
    return (uint32_t)(time_us_64() / 1000U);
}

/* Fixed-rate wait: next deadline = previous + period (no drift). */
void period_wait(uint64_t *p_next_us, uint32_t period_ms)
{
    uint64_t now;
    int64_t  wait_us;

    *p_next_us += (uint64_t)period_ms * 1000U;
    now = time_us_64();
    wait_us = (int64_t)(*p_next_us - now);
    if (wait_us <= 0)
    {
        *p_next_us = now;                 /* overrun: resync */
        (void)tk_dly_tsk(0);              /* still yield */
    }
    else
    {
        (void)tk_dly_tsk((TMO)((wait_us + 999) / 1000));
    }
}

void heartbeat(uint32_t bit)
{
    (void)tk_set_flg(g_hb_flg, (UINT)bit);
}

/* ------------------------------------------------------------------ */
/* Creation helpers                                                    */
/* ------------------------------------------------------------------ */
static ID make_mbf(INT msg_size, INT depth)
{
    T_CMBF c;

    (void)memset(&c, 0, sizeof(c));
    c.mbfatr = TA_TFIFO;
    c.bufsz  = (SZ)(depth * (msg_size + 8));   /* + header per message */
    c.maxmsz = msg_size;
    return tk_cre_mbf(&c);
}

static ID make_flg(void)
{
    T_CFLG c;

    (void)memset(&c, 0, sizeof(c));
    c.flgatr  = TA_TFIFO | TA_WMUL;
    c.iflgptn = 0U;
    return tk_cre_flg(&c);
}

static void make_task(FP entry, PRI pri, SZ stack)
{
    T_CTSK c;
    ID     id;

    (void)memset(&c, 0, sizeof(c));
    c.tskatr  = TA_HLNG | TA_RNG3;
    c.task    = entry;
    c.itskpri = pri;
    c.stksz   = stack;
    id = tk_cre_tsk(&c);
    if (id > 0)
    {
        (void)tk_sta_tsk(id, 0);
    }
}

/* ------------------------------------------------------------------ */
/* Entry point called by micro T-Kernel after kernel start-up          */
/* ------------------------------------------------------------------ */
EXPORT INT usermain(void)
{
    T_CMTX cm;

    /* The template already brought up the USB console (tm_printf) and the
     * CYW43 radio task, which also runs our MQTT client (comms.c).      */
    tm_printf((UB *)"\n=== Group 12 robot car (micro T-Kernel) ===\n");

    /* Brake first so the wheels cannot move during boot. */
    hal_motor_init();
    hal_servo_init();
    hal_buttons_init();
    hal_led_init();

    g_line_pos_q   = make_mbf((INT)sizeof(line_pos_t),     4);
    g_bar_cmd_q    = make_mbf((INT)sizeof(bar_cmd_t),      4);
    g_motion_cmd_q = make_mbf((INT)sizeof(motion_cmd_t),   4);
    g_motion_evt_q = make_mbf((INT)sizeof(motion_evt_t),   8);
    g_obstacle_q   = make_mbf((INT)sizeof(obstacle_msg_t), 4);
    g_scan_req_q   = make_mbf((INT)sizeof(scan_req_t),     2);
    g_remote_cmd_q = make_mbf((INT)sizeof(uint8_t),        4);  /* 1-byte cmd */
    g_telemetry_q  = make_mbf((INT)sizeof(telem_msg_t),    16);
    g_hb_flg       = make_flg();
    g_evt_flg      = make_flg();
    g_line_evt_flg = make_flg();

    (void)memset(&cm, 0, sizeof(cm));
    cm.mtxatr = TA_INHERIT;
    g_state_mtx = tk_cre_mtx(&cm);

    fault_init();
    barcode_map_init();
    comms_init();
    hal_isr_init();     /* after the event flag exists - the ISR uses it */

    make_task((FP)safety_task,       PRI_SAFETY, 1024);
    make_task((FP)line_barcode_task, PRI_LINE,   2048);
    make_task((FP)motion_task,       PRI_MOTION, 2048);
    make_task((FP)nav_fsm_task,      PRI_NAV,    2048);
    make_task((FP)imu_task,          PRI_IMU,    2048);
    make_task((FP)ultrasonic_task,   PRI_US,     1536);
    make_task((FP)telemetry_task,    PRI_TELEM,  2048);
    /* No comms task: MQTT runs inside the template's CYW43 task (see
     * comms.c), which owns the radio and lwIP. */

    tm_printf((UB *)"car: all tasks started\n");

    /* usermain may sleep forever; the tasks run on */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}

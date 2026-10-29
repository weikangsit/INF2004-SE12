/**
 * @file    nav_fsm.c
 * @brief   Navigation FSM Task (team) - system-level statechart, Section 5.
 *
 *  INIT -> READY -> LINE_FOLLOW <-> {BARCODE_ACT, HUMP_REPORT,
 *  OBSTACLE_AVOID -> LINE_REACQUIRE} -> MISSION_DONE ; any -> FAULT
 *
 *  micro T-Kernel cannot block on several message buffers at once, so
 *  the task polls its inputs every 10 ms (it is still event-driven in
 *  that it only acts when a message or flag arrives).
 */

#include "port.h"

#include "app.h"
#include "barcode_map.h"
#include "hal.h"
#include "imu.h"
#include "line_barcode.h"
#include "ultrasonic.h"
#include "nav_fsm.h"

#define INIT_TIMEOUT_MS      (15000U)
#define BTN_STABLE_TICKS     (3U)
#define BYPASS_TURN_DEG      (45.0f)
#define BARCODE_STRAIGHT_M   (0.10f)
#define REVERSE_RETRY_M      (0.10f)
#ifndef REQUIRE_WIFI_AT_INIT
#define REQUIRE_WIFI_AT_INIT (1)
#endif

typedef struct
{
    bool     stable;
    uint8_t  count;
} debounce_t;

/* returns true once on press */
static bool debounce(debounce_t *p_d, bool raw)
{
    bool pressed = false;

    if (raw != p_d->stable)
    {
        p_d->count++;
        if (p_d->count >= BTN_STABLE_TICKS)
        {
            p_d->stable = raw;
            p_d->count = 0U;
            pressed = raw;
        }
    }
    else
    {
        p_d->count = 0U;
    }
    return pressed;
}

static nav_state_t g_state = NAV_INIT;
static uint32_t    g_state_ms;

/* p_reason documents the transition; the wire carries only the state */
static void enter(nav_state_t s, const char *p_reason)
{
    (void)p_reason;
    g_state = s;
    g_state_ms = now_ms();
    state_lock();
    state_raw()->sys_state = (uint8_t)s;
    state_unlock();

    switch (s)
    {
        case NAV_READY:
        case NAV_MISSION_DONE:  hal_led_set(LED_BLUE);  break;
        case NAV_FAULT:         hal_led_set(LED_RED);   break;
        case NAV_INIT:          hal_led_set(LED_OFF);   break;
        default:
            hal_led_set(comms_is_online() ? LED_GREEN : LED_AMBER);
            break;
    }
    telem_event(EV_STATE, (uint8_t)s, 0, 0);
}

static void cruise(void)
{
    moveForward(SPEED_CRUISE_MPS, 0.0f, true);
}

static void fault(fault_code_t code)
{
    stop();
    enter(NAV_FAULT, "fault");
    telem_event(EV_FAULT, (uint8_t)code, 0, 0);
}

/* Turn by `deg` toward `left`. */
static void turn(bool left, float deg, bool end_on_line)
{
    if (left)
    {
        turnLeft(deg, end_on_line);
    }
    else
    {
        turnRight(deg, end_on_line);
    }
}

void nav_fsm_task(INT stacd, void *exinf)
{
    uint64_t   next_us = time_us_64();
    debounce_t b_start = { false, 0U };
    debounce_t b_cal = { false, 0U };
    uint8_t    init_step = 0U;
    uint8_t    bypass_step = 0U;
    bool       bypass_left = true;
    bool       motion_busy = false;
    bool       reverse_pending = false;
    bool       led_online = false;

    (void)stacd;
    (void)exinf;

    enter(NAV_INIT, "boot");

    for (;;)
    {
        bar_cmd_t      bar;
        motion_evt_t   mev;
        obstacle_msg_t obs;
        remote_cmd_t   rc;
        UINT           lev = 0U;
        bool           have_bar;
        bool           have_mev;
        bool           have_obs;
        bool           have_rc;
        bool           start_pressed;
        bool           cal_pressed;

        period_wait(&next_us, PERIOD_NAV_POLL_MS);
        heartbeat(HB_NAV);

        start_pressed = debounce(&b_start, hal_btn_start_raw());
        cal_pressed   = debounce(&b_cal, hal_btn_cal_raw());
        have_bar = (tk_rcv_mbf(g_bar_cmd_q, &bar, TMO_POL) > 0);
        have_mev = (tk_rcv_mbf(g_motion_evt_q, &mev, TMO_POL) > 0);
        have_obs = (tk_rcv_mbf(g_obstacle_q, &obs, TMO_POL) > 0);
        have_rc  = (tk_rcv_mbf(g_remote_cmd_q, &rc, TMO_POL) > 0);
        (void)tk_wai_flg(g_line_evt_flg, 0xFFFFU, TWF_ORW | TWF_CLR, &lev, TMO_POL);

        if (have_mev && (MEVT_DONE == mev.id))
        {
            motion_busy = false;
        }

        /* WiFi status LED while running */
        if ((g_state >= NAV_LINE_FOLLOW) && (g_state <= NAV_LINE_REACQUIRE) &&
            (comms_is_online() != led_online))
        {
            led_online = comms_is_online();
            hal_led_set(led_online ? LED_GREEN : LED_AMBER);
        }

        /* ---------- global transitions (any state) ---------- */
        if (have_rc && (RCMD_ESTOP == rc))
        {
            (void)tk_set_flg(g_evt_flg, EVT_ESTOP);
            fault(FC_ESTOP);
            continue;
        }
        if ((NAV_FAULT != g_state) && (NAV_INIT != g_state) &&
            (safety_is_latched() || (have_mev && (MEVT_STALL == mev.id)) ||
             (0U != (lev & LEVT_LOST))))
        {
            fault(safety_is_latched() ? FC_COLLISION :
                  ((have_mev && (MEVT_STALL == mev.id)) ? FC_STALL : FC_LINE_LOST));
            continue;
        }
        if ((g_state >= NAV_LINE_FOLLOW) && (g_state <= NAV_LINE_REACQUIRE))
        {
            if ((have_rc && (RCMD_FINISH == rc)) || start_pressed)
            {
                motion_state_t st;

                stop();
                enter(NAV_MISSION_DONE, "finish");
                state_read(&st);
                telem_event(EV_SUMMARY, 0U, (int16_t)(st.hump_peak_mm * 10.0f),
                            (int16_t)(st.dist_m * 100.0f));
                continue;
            }
            if (have_rc && (RCMD_STOP == rc))
            {
                stop();
                enter(NAV_READY, "remote stop");
                continue;
            }
        }

        /* ---------- per-state behaviour ---------- */
        /* Bench motor test (web app button): only while not driving */
        if (have_rc && (RCMD_MOTOR_TEST == rc) &&
            ((NAV_READY == g_state) || (NAV_FAULT == g_state) || (NAV_MISSION_DONE == g_state)))
        {
            motion_cmd_t c = { MCMD_MOTOR_TEST, 0.0f, 0.0f, 0.0f, false, false };

            safety_clear();
            stop();                               /* leaves motion FAULT first */
            (void)tk_snd_mbf(g_motion_cmd_q, &c, (INT)sizeof(c), TMO_POL);
            continue;
        }

        switch (g_state)
        {
            case NAV_INIT:
                /* power-on self-test */
                if ((now_ms() - g_state_ms) > INIT_TIMEOUT_MS)
                {
                    fault(FC_SELFTEST);
                    break;
                }
                if ((0U == init_step) && imu_is_ok() && line_is_ready() &&
                    (us_front_cm() < 900.0f) &&
                    ((0 == REQUIRE_WIFI_AT_INIT) ||
                     (comms_is_online() && barcode_map_is_loaded())))
                {
                    /* encoder self-test: short open-loop push, learns
                     * each encoder's count direction (motion.c) */
                    motion_cmd_t c = { MCMD_ENC_CAL, 0.0f, 0.0f, 0.0f, false, false };
                    (void)tk_snd_mbf(g_motion_cmd_q, &c, (INT)sizeof(c), TMO_POL);
                    init_step = 1U;
                }
                else if ((1U == init_step) && have_mev && (MEVT_DONE == mev.id))
                {
                    enter(NAV_READY, "selftest ok");
                }
                else if ((1U == init_step) && have_mev && (MEVT_STALL == mev.id))
                {
                    fault(FC_STALL);
                }
                else if (0U != (lev & LEVT_SENSOR_FAIL))
                {
                    fault(FC_SENSOR);
                }
                else
                {
                    /* waiting */
                }
                break;

            case NAV_READY:
                if (start_pressed || (have_rc && (RCMD_START == rc)))
                {
                    cruise();
                    enter(NAV_LINE_FOLLOW, "start");
                }
                else if (cal_pressed)
                {
                    line_request_calibration();
                }
                else
                {
                    /* idle */
                }
                break;

            case NAV_LINE_FOLLOW:
            case NAV_HUMP_REPORT:
                if (have_obs && (OBS_NEAR == obs.kind))
                {
                    scan_req_t req = SCAN_REQ_PROFILE;

                    stop();
                    (void)tk_snd_mbf(g_scan_req_q, &req, (INT)sizeof(req), TMO_POL);
                    enter(NAV_OBSTACLE_AVOID, "obstacle");
                }
                else if ((NAV_LINE_FOLLOW == g_state) && have_bar)
                {
                    barcode_action_t act = barcode_map_get(bar);

                    /* v1 = wire action code 0..3, -1 = letter not configured */
                    telem_event(EV_BARCODE, (uint8_t)bar, (int16_t)((int16_t)act - 1), 0);

                    switch (act)
                    {
                        case BA_LEFT:     turnLeft(90.0f, true);  break;
                        case BA_RIGHT:    turnRight(90.0f, true); break;
                        case BA_UTURN:
                        {
                            motion_cmd_t c = { MCMD_UTURN, SPEED_TURN_MPS, 0.0f,
                                               180.0f, false, true };
                            (void)tk_snd_mbf(g_motion_cmd_q, &c, (INT)sizeof(c), TMO_POL);
                            break;
                        }
                        case BA_STRAIGHT:
                            moveForward(SPEED_CRUISE_MPS, BARCODE_STRAIGHT_M, false);
                            break;
                        case BA_NONE:
                        default:
                            break;    /* not configured: keep following */
                    }
                    if (BA_NONE != act)
                    {
                        motion_busy = true;
                        enter(NAV_BARCODE_ACT, "barcode");
                    }
                }
                else if ((NAV_LINE_FOLLOW == g_state) && have_mev &&
                         (MEVT_HUMP_START == mev.id))
                {
                    moveForward(SPEED_HUMP_MPS, 0.0f, true);
                    enter(NAV_HUMP_REPORT, "hump");
                }
                else if ((NAV_HUMP_REPORT == g_state) && have_mev &&
                         (MEVT_HUMP_END == mev.id))
                {
                    cruise();
                    enter(NAV_LINE_FOLLOW, "hump done");
                }
                else
                {
                    /* cruising */
                }
                break;

            case NAV_BARCODE_ACT:
                if (!motion_busy)
                {
                    cruise();
                    enter(NAV_LINE_FOLLOW, "barcode done");
                }
                break;

            case NAV_OBSTACLE_AVOID:
                if (have_obs && (OBS_PROFILE != obs.kind) && (OBS_NEAR != obs.kind))
                {
                    if (OBS_ROUTE_CONTINUE == obs.kind)
                    {
                        cruise();
                        enter(NAV_LINE_FOLLOW, "path clear");
                    }
                    else if (OBS_ROUTE_REVERSE == obs.kind)
                    {
                        /* REVERSE_RETRY: back off, rescan once done */
                        moveBackward(REVERSE_RETRY_M);
                        reverse_pending = true;
                        motion_busy = true;
                    }
                    else if (OBS_NO_ROUTE == obs.kind)
                    {
                        fault(FC_NO_ROUTE);
                    }
                    else
                    {
                        /* BYPASS_EXEC: out 45, offset, back 45, along, in 45 */
                        bypass_left = (OBS_ROUTE_LEFT == obs.kind);
                        bypass_step = 1U;
                        turn(bypass_left, BYPASS_TURN_DEG, false);
                        motion_busy = true;
                    }
                }
                else if (reverse_pending && !motion_busy)
                {
                    scan_req_t req = SCAN_REQ_PROFILE;

                    reverse_pending = false;
                    (void)tk_snd_mbf(g_scan_req_q, &req, (INT)sizeof(req), TMO_POL);
                }
                else if ((0U != bypass_step) && !motion_busy)
                {
                    bypass_step++;
                    motion_busy = true;
                    switch (bypass_step)
                    {
                        case 2U: moveForward(SPEED_SEARCH_MPS, BYPASS_OFFSET_M, false); break;
                        case 3U: turn(!bypass_left, BYPASS_TURN_DEG, false);            break;
                        case 4U: moveForward(SPEED_SEARCH_MPS, BYPASS_LENGTH_M, false); break;
                        case 5U: turn(!bypass_left, BYPASS_TURN_DEG, false);            break;
                        default:
                            /* LINE_REACQUIRE: drive back toward the line */
                            moveForward(SPEED_SEARCH_MPS, BYPASS_OFFSET_M * 2.0f, false);
                            bypass_step = 0U;
                            enter(NAV_LINE_REACQUIRE, "bypass done");
                            break;
                    }
                }
                else
                {
                    /* waiting for scan / motion */
                }
                break;

            case NAV_LINE_REACQUIRE:
            {
                motion_state_t st;
                state_read(&st);

                if ((0U != (lev & LEVT_REGAINED)) || st.line_seen)
                {
                    /* line found: turn back onto it and resume */
                    turn(bypass_left, BYPASS_TURN_DEG, true);
                    motion_busy = true;
                    enter(NAV_BARCODE_ACT, "line regained");   /* reuse: wait done -> cruise */
                }
                else if ((now_ms() - g_state_ms) > REACQUIRE_TIMEOUT_MS)
                {
                    fault(FC_REACQUIRE);
                }
                else
                {
                    /* searching */
                }
                break;
            }

            case NAV_MISSION_DONE:
            case NAV_FAULT:
                if (cal_pressed || (have_rc && (RCMD_RESET == rc)))
                {
                    safety_clear();
                    stop();
                    init_step = 0U;
                    bypass_step = 0U;
                    motion_busy = false;
                    reverse_pending = false;
                    enter(NAV_READY, "reset");
                }
                break;

            default:
                fault(FC_NONE);
                break;
        }
    }
}

/** @file    nav_fsm.c
 *  @brief   Navigation FSM task (team), see nav_fsm.h.
 *
 *  INIT -> READY -> LINE_FOLLOW <-> {BARCODE_ACT, HUMP_REPORT,
 *  OBSTACLE_AVOID -> LINE_REACQUIRE} -> MISSION_DONE ; any -> FAULT
 *
 *  micro T-Kernel cannot wait on several message buffers at once, so the
 *  task polls its inputs with a bounded wait of PERIOD_NAV_POLL_MS and
 *  reports its heartbeat every period (Tribunal 1 Version C: bounded waits
 *  while waiting for motion completion).
 *
 *  Motion completion: every command returns a sequence number; only the
 *  completion with the awaited number moves the FSM on, so a late report
 *  of an older command is ignored.
 *
 *  Obstacle (Tribunal 1 Version C): stop; reverse REVERSE_BEFORE_SCAN_CM
 *  with line following only if the car drove at least that far plus the
 *  margin straight along the line since its last turn; then scan. A scan
 *  without enough valid echoes stops with FC_NO_ROUTE detail 1 ("scan
 *  cannot proceed safely").
 *
 *  Faults already reported by Safety or Motion Control are not reported
 *  again here; the FSM just enters FAULT.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <string.h>

#include "port.h"

#include "barcode.h"
#include "barcode_map.h"
#include "car_config.h"
#include "comms.h"
#include "fault.h"
#include "hal.h"
#include "imu_terrain.h"
#include "motion_control.h"
#include "nav_fsm.h"
#include "proto.h"
#include "safety.h"
#include "sensor_line.h"
#include "systime.h"
#include "telemetry.h"
#include "ultrasonic_scan.h"
#include "vehicle_state.h"

#define BTN_STABLE_TICKS     (3U)
#define REMOTE_Q_DEPTH       (4U)
#define MBF_SLOT_BYTES       (8U)        /* 4-byte header + 1 byte       */
#define MM_PER_CM            (10)
#define NO_SEQ               (0U)
#define DETAIL_SCAN_UNSAFE   (1)         /* FC_NO_ROUTE detail           */
#define LETTER_FIRST         ('A')

/** Obstacle bypass steps. */
typedef enum
{
    BYPASS_NONE = 0,
    BYPASS_TURN_OUT,
    BYPASS_OFFSET,
    BYPASS_TURN_BACK,
    BYPASS_ALONG,
    BYPASS_TURN_IN,
    BYPASS_RETURN
} bypass_step_t;

/** One debounced button. */
typedef struct
{
    bool     b_stable;       /* debounced level, true = pressed          */
    uint8_t  count;
    uint32_t press_ms;       /* time of the debounced press              */
    bool     b_long_sent;
} button_t;

/** Inputs of one period. */
typedef struct
{
    uint32_t            now;
    bool                b_start;          /* GP20 pressed                */
    bool                b_cal_short;      /* GP21 released before long   */
    bool                b_cal_long;       /* GP21 held BTN_LONG_PRESS_MS */
    bool                b_have_letter;
    uint8_t             letter;           /* 0..3 = A..D                 */
    bool                b_have_done;
    motion_done_t       done;
    bool                b_have_terrain;
    imu_terrain_event_t terrain;
    bool                b_have_us;
    us_result_t         us;
    bool                b_have_rc;
    proto_cmd_t         remote;
    uint32_t            line_events;
} nav_in_t;

/** Task-private FSM context. */
typedef struct
{
    nav_fsm_state_t state;
    uint32_t        state_ms;
    bool            b_init_started;
    uint8_t         pending_seq;          /* awaited motion completion   */
    bypass_step_t   bypass;
    bool            b_bypass_left;
    bool            b_reversed;           /* scan follows a reverse      */
    bool            b_reverse_wait;
    bool            b_mag_cal;
    bool            b_led_online;
    int32_t         straight_start_mm;    /* start of the straight run   */
    button_t        btn_start;
    button_t        btn_cal;
} nav_ctx_t;

static ID        g_remote_q = 0;
static nav_ctx_t g_nav;                   /* task only                   */

static void    inputs_read (nav_in_t * p_in);
static bool    button_press (button_t * p_btn, bool b_raw, uint32_t now);
static bool    button_release_short (button_t * p_btn, bool b_raw,
                                     uint32_t now, bool * p_long);
static bool    global_transitions (nav_in_t const * p_in);
static void    state_step (nav_in_t const * p_in);
static void    init_step (nav_in_t const * p_in);
static void    ready_step (nav_in_t const * p_in);
static void    follow_step (nav_in_t const * p_in);
static void    barcode_act (uint8_t letter);
static void    obstacle_begin (void);
static void    obstacle_step (nav_in_t const * p_in);
static void    bypass_next (void);
static void    reacquire_step (nav_in_t const * p_in);
static void    reset_step (nav_in_t const * p_in);
static bool    done_awaited (nav_in_t const * p_in, motion_result_t * p_res);
static void    follow_begin (void);
static void    enter (nav_fsm_state_t state);
static void    nav_fault (fault_code_t code, bool b_report, int16_t detail);
static int32_t dist_now_mm (void);

/*!
 * @brief  Create the remote command queue. Call from usermain() before
 *         the tasks start.
 */
void
nav_fsm_init (void)
{
    T_CMBF create;

    (void) memset(&g_nav, 0, sizeof(g_nav));
    g_nav.state = NAV_INIT;

    (void) memset(&create, 0, sizeof(create));
    create.mbfatr = TA_TFIFO;
    create.bufsz  = (SZ) (REMOTE_Q_DEPTH * MBF_SLOT_BYTES);    /* 32 B    */
    create.maxmsz = (INT) sizeof(uint8_t);
    g_remote_q    = tk_cre_mbf(&create);
}

/*!
 * @brief  Queue a validated remote command (comms.c). Never blocks; if the
 *         queue is full the command is dropped (an e-stop has already
 *         reached Safety directly).
 * @param  cmd  proto_cmd_t value, 0..5.
 */
void
nav_fsm_post_remote (uint8_t cmd)
{
    (void) tk_snd_mbf(g_remote_q, &cmd, (INT) sizeof(cmd), TMO_POL);
}

/*!
 * @brief  Navigation FSM task body (priority PRI_NAV, 10 ms poll).
 * @param  stacd    Start code (unused).
 * @param  p_exinf  Extended information (unused).
 */
void
nav_fsm_task (INT stacd, void * p_exinf)
{
    uint64_t next_us = time_us_64();
    nav_in_t in;

    (void) stacd;
    (void) p_exinf;

    enter(NAV_INIT);

    for (;;)
    {
        systime_period_wait(&next_us, PERIOD_NAV_POLL_MS);
        safety_heartbeat(SAFETY_HB_NAV);

        inputs_read(&in);

        /* WiFi status LED while running. */
        if ((g_nav.state >= NAV_LINE_FOLLOW) &&
            (g_nav.state <= NAV_LINE_REACQUIRE) &&
            (comms_is_online() != g_nav.b_led_online))
        {
            g_nav.b_led_online = comms_is_online();
            hal_led_set(g_nav.b_led_online ? HAL_LED_GREEN : HAL_LED_AMBER);
        }

        if (!global_transitions(&in))
        {
            state_step(&in);
        }
    }
}

/*!
 * @brief  Collect every input of this period.
 * @param  p_in  Output inputs.
 */
static void
inputs_read (nav_in_t * p_in)
{
    uint8_t remote = 0U;

    (void) memset(p_in, 0, sizeof(*p_in));
    p_in->now = systime_now_ms();

    p_in->b_start      = button_press(&g_nav.btn_start, hal_btn_start_raw(),
                                      p_in->now);
    p_in->b_cal_short  = button_release_short(&g_nav.btn_cal,
                                              hal_btn_cal_raw(), p_in->now,
                                              &p_in->b_cal_long);
    p_in->b_have_letter  = barcode_take(&p_in->letter);
    p_in->b_have_done    = motion_control_done_take(&p_in->done);
    p_in->b_have_terrain = imu_terrain_event_take(&p_in->terrain);
    p_in->b_have_us      = ultrasonic_scan_result_take(&p_in->us);
    p_in->b_have_rc      = (tk_rcv_mbf(g_remote_q, &remote, TMO_POL) > 0);
    p_in->line_events    = sensor_line_events_take();

    if (p_in->b_have_rc)
    {
        /* comms.c queues only validated commands 0..5. */
        p_in->remote = (proto_cmd_t) remote;
    }
}

/*!
 * @brief  Debounce a button.
 * @param  p_btn  Button state.
 * @param  b_raw  Raw level, true = pressed.
 * @param  now    Current time.
 * @return True once, on the debounced press.
 */
static bool
button_press (button_t * p_btn, bool b_raw, uint32_t now)
{
    bool b_pressed = false;

    if (b_raw != p_btn->b_stable)
    {
        p_btn->count++;

        if (p_btn->count >= BTN_STABLE_TICKS)
        {
            p_btn->b_stable    = b_raw;
            p_btn->count       = 0U;
            p_btn->press_ms    = now;
            p_btn->b_long_sent = false;
            b_pressed          = b_raw;
        }
    }
    else
    {
        p_btn->count = 0U;
    }

    return (b_pressed);
}

/*!
 * @brief  Short / long press of a button: a long press fires once while
 *         held; a short press fires on release.
 * @param  p_btn   Button state.
 * @param  b_raw   Raw level, true = pressed.
 * @param  now     Current time.
 * @param  p_long  Output: true once when the hold reaches the long time.
 * @return True on release after a short press.
 */
static bool
button_release_short (button_t * p_btn, bool b_raw, uint32_t now,
                      bool * p_long)
{
    bool const b_was_pressed = p_btn->b_stable;
    bool const b_long_sent   = p_btn->b_long_sent;
    bool       b_short       = false;

    (void) button_press(p_btn, b_raw, now);

    if (b_was_pressed && (!p_btn->b_stable))
    {
        b_short = !b_long_sent;                   /* released             */
    }
    else if (p_btn->b_stable && (!p_btn->b_long_sent) &&
             ((now - p_btn->press_ms) >= BTN_LONG_PRESS_MS))
    {
        p_btn->b_long_sent = true;
        *p_long            = true;
    }
    else
    {
        /* No change. */
    }

    return (b_short);
}

/*!
 * @brief  Transitions that apply in any state.
 * @param  p_in  Inputs of this period.
 * @return True if a transition was taken (skip the state step).
 */
static bool
global_transitions (nav_in_t const * p_in)
{
    bool const b_driving = (g_nav.state >= NAV_LINE_FOLLOW) &&
                           (g_nav.state <= NAV_LINE_REACQUIRE);
    bool       b_taken   = true;

    if ((NAV_FAULT != g_nav.state) &&
        ((p_in->b_have_rc && (PROTO_CMD_ESTOP == p_in->remote)) ||
         safety_is_latched()))
    {
        nav_fault(FC_ESTOP, false, 0);            /* Safety reported it   */
    }
    else if (b_driving && (p_in->b_start ||
                           (p_in->b_have_rc &&
                            (PROTO_CMD_FINISH == p_in->remote))))
    {
        vehicle_state_t state;

        motion_control_stop();
        enter(NAV_MISSION_DONE);
        vehicle_state_read(&state);
        /* Hump in 0.1 mm and distance in cm both fit an int16_t. */
        telemetry_post_event(EV_SUMMARY, 0U,
                             (int16_t) (state.hump_peak_half_mm * 5U),
                             (int16_t) (state.dist_mm / MM_PER_CM));
    }
    else if (b_driving && p_in->b_have_rc && (PROTO_CMD_STOP == p_in->remote))
    {
        motion_control_stop();
        enter(NAV_READY);
    }
    else if (p_in->b_have_rc &&
             ((PROTO_CMD_MOTOR_TEST == p_in->remote) ||
              (PROTO_CMD_DEADBAND_TEST == p_in->remote)) &&
             ((NAV_READY == g_nav.state) || (NAV_FAULT == g_nav.state) ||
              (NAV_MISSION_DONE == g_nav.state)))
    {
        safety_clear();                           /* bench, wheels up     */

        if (PROTO_CMD_MOTOR_TEST == p_in->remote)
        {
            (void) motion_control_motor_test();
        }
        else
        {
            (void) motion_control_deadband_test();
        }
    }
    else
    {
        b_taken = false;
    }

    return (b_taken);
}

/*!
 * @brief  Behaviour of the current state.
 * @param  p_in  Inputs of this period.
 */
static void
state_step (nav_in_t const * p_in)
{
    motion_result_t result = MOTION_RESULT_DONE;
    bool const      b_done = done_awaited(p_in, &result);

    if (b_done && (MOTION_RESULT_DONE != result))
    {
        /* Stall or failed self-test: Motion Control reported it. */
        nav_fault(FC_STALL, false, 0);
    }
    else
    {
        switch (g_nav.state)
        {
            case NAV_INIT:           init_step(p_in);      break;
            case NAV_READY:          ready_step(p_in);     break;
            case NAV_LINE_FOLLOW:
            case NAV_HUMP_REPORT:    follow_step(p_in);    break;
            case NAV_BARCODE_ACT:
                if (b_done)
                {
                    follow_begin();               /* a turn ends the run  */
                    enter(NAV_LINE_FOLLOW);
                }
            break;
            case NAV_OBSTACLE_AVOID: obstacle_step(p_in);  break;
            case NAV_LINE_REACQUIRE: reacquire_step(p_in); break;
            case NAV_MISSION_DONE:
            case NAV_FAULT:          reset_step(p_in);     break;
            default:
                nav_fault(FC_SOFTWARE, true, 0);
            break;
        }

        /* A completion of the awaited command that the state did not use
         * (e.g. the bypass or reverse steps) is handled there; READY uses
         * it to finish the magnetometer calibration. */
        if (b_done && (NAV_READY == g_nav.state) && g_nav.b_mag_cal)
        {
            g_nav.b_mag_cal = false;
            imu_terrain_mag_cal_finish();
        }
    }
}

/*!
 * @brief  INIT: power-on self-test, then the encoder self-test.
 * @param  p_in  Inputs of this period.
 */
static void
init_step (nav_in_t const * p_in)
{
    if ((p_in->now - g_nav.state_ms) > INIT_TIMEOUT_MS)
    {
        nav_fault(FC_SELFTEST, true, 0);
    }
    else if (0U != (p_in->line_events & SENSOR_LINE_EVT_SENSOR_FAIL))
    {
        nav_fault(FC_SENSOR, true, 0);
    }
    else if ((!g_nav.b_init_started) && imu_terrain_is_ok() &&
             sensor_line_is_ready() && ultrasonic_scan_is_ok())
    {
        g_nav.b_init_started = true;
        g_nav.pending_seq    = motion_control_self_test();
    }
    else if (g_nav.b_init_started && (NO_SEQ == g_nav.pending_seq))
    {
        enter(NAV_READY);                         /* self-test done       */
    }
    else
    {
        /* Waiting for the sensors or the self-test. */
    }
}

/*!
 * @brief  READY: start, line calibration (short GP21) or magnetometer
 *         calibration (GP21 held).
 * @param  p_in  Inputs of this period.
 */
static void
ready_step (nav_in_t const * p_in)
{
    if (g_nav.b_mag_cal)
    {
        /* Spinning: wait for the turn to finish (state_step). */
    }
    else if (p_in->b_start ||
             (p_in->b_have_rc && (PROTO_CMD_START == p_in->remote)))
    {
        follow_begin();
        enter(NAV_LINE_FOLLOW);
    }
    else if (p_in->b_cal_long)
    {
        g_nav.b_mag_cal = true;
        imu_terrain_mag_cal_start();
        g_nav.pending_seq = motion_control_turn(true, MAG_CAL_SPEED_CMS,
                                                MAG_CAL_SPIN_DEG, false);
    }
    else if (p_in->b_cal_short)
    {
        sensor_line_request_calibration();
    }
    else
    {
        /* Idle. */
    }
}

/*!
 * @brief  LINE_FOLLOW and HUMP_REPORT: obstacles, barcodes, humps, and
 *         the line-lost timeout.
 * @param  p_in  Inputs of this period.
 */
static void
follow_step (nav_in_t const * p_in)
{
    uint32_t const in_state = p_in->now - g_nav.state_ms;

    if (p_in->b_have_us && (US_RESULT_NEAR == p_in->us))
    {
        obstacle_begin();
    }
    else if ((sensor_line_black_age_ms() > LINE_LOST_MS) &&
             (in_state > LINE_LOST_MS))
    {
        safety_report(SAFETY_EVT_LINE_LOST);      /* Safety trips         */
    }
    else if ((NAV_LINE_FOLLOW == g_nav.state) && p_in->b_have_letter)
    {
        barcode_act(p_in->letter);
    }
    else if ((NAV_LINE_FOLLOW == g_nav.state) && p_in->b_have_terrain &&
             (IMU_EVT_HUMP_START == p_in->terrain))
    {
        (void) motion_control_forward(SPEED_HUMP_CMS, 0U, true);
        enter(NAV_HUMP_REPORT);
    }
    else if ((NAV_HUMP_REPORT == g_nav.state) && p_in->b_have_terrain &&
             (IMU_EVT_HUMP_END == p_in->terrain))
    {
        (void) motion_control_forward(SPEED_CRUISE_CMS, 0U, true);
        enter(NAV_LINE_FOLLOW);
    }
    else
    {
        /* Cruising. */
    }
}

/*!
 * @brief  Act on a decoded barcode letter (table in barcode_map.c).
 * @param  letter  0..3 for A..D.
 */
static void
barcode_act (uint8_t letter)
{
    /* letter is 0..3, so 'A' + letter is a valid character. */
    char const                 code   = (char) (LETTER_FIRST + letter);
    barcode_map_action_t const action = barcode_map_get(code);
    uint8_t                    seq    = NO_SEQ;

    /* value1 = wire action code 0..3, -1 = letter not configured. */
    telemetry_post_event(EV_BARCODE, (uint8_t) code,
                         (int16_t) ((int32_t) action - 1), 0);

    switch (action)
    {
        case BA_LEFT:
            seq = motion_control_turn(true, SPEED_TURN_CMS, BARCODE_TURN_DEG,
                                      true);
        break;

        case BA_RIGHT:
            seq = motion_control_turn(false, SPEED_TURN_CMS,
                                      BARCODE_TURN_DEG, true);
        break;

        case BA_UTURN:
            seq = motion_control_uturn();
        break;

        case BA_STRAIGHT:
            seq = motion_control_forward(SPEED_CRUISE_CMS,
                                         BARCODE_STRAIGHT_CM, false);
        break;

        case BA_NONE:
        default:
            /* Not configured: keep following. */
        break;
    }

    if (NO_SEQ != seq)
    {
        g_nav.pending_seq = seq;
        enter(NAV_BARCODE_ACT);
    }
}

/*!
 * @brief  Obstacle ahead: stop, then reverse along the own track if it is
 *         long enough, or scan from here.
 */
static void
obstacle_begin (void)
{
    int32_t const run_mm  = dist_now_mm() - g_nav.straight_start_mm;
    int32_t const need_mm = (int32_t) (REVERSE_BEFORE_SCAN_CM +
                                       REVERSE_MARGIN_CM) * MM_PER_CM;

    motion_control_stop();
    g_nav.bypass     = BYPASS_NONE;
    g_nav.b_reversed = false;

    if ((0U != REVERSE_BEFORE_SCAN_CM) && (run_mm >= need_mm))
    {
        g_nav.b_reversed     = true;
        g_nav.b_reverse_wait = true;
        g_nav.pending_seq    = motion_control_reverse(SPEED_SEARCH_CMS,
                                                      REVERSE_BEFORE_SCAN_CM,
                                                      true);
    }
    else
    {
        ultrasonic_scan_request();
    }

    enter(NAV_OBSTACLE_AVOID);
}

/*!
 * @brief  OBSTACLE_AVOID: wait for the reverse, act on the scan result,
 *         and run the bypass steps.
 * @param  p_in  Inputs of this period.
 */
static void
obstacle_step (nav_in_t const * p_in)
{
    bool const b_idle = (NO_SEQ == g_nav.pending_seq);

    if (g_nav.b_reverse_wait && b_idle)
    {
        g_nav.b_reverse_wait = false;             /* stopped: now scan    */
        ultrasonic_scan_request();
    }
    else if (p_in->b_have_us && (US_RESULT_NEAR != p_in->us) &&
             (BYPASS_NONE == g_nav.bypass) && (!g_nav.b_reverse_wait))
    {
        switch (p_in->us)
        {
            case US_RESULT_CONTINUE:
                (void) motion_control_forward(SPEED_CRUISE_CMS, 0U, true);
                enter(NAV_LINE_FOLLOW);
            break;

            case US_RESULT_REVERSE:
                g_nav.b_reverse_wait = true;      /* back off, rescan     */
                g_nav.pending_seq    = motion_control_reverse(
                    SPEED_SEARCH_CMS, REVERSE_RETRY_CM, true);
            break;

            case US_RESULT_LEFT:
            case US_RESULT_RIGHT:
                g_nav.b_bypass_left = (US_RESULT_LEFT == p_in->us);
                bypass_next();
            break;

            case US_RESULT_UNSAFE:
                nav_fault(FC_NO_ROUTE, true, DETAIL_SCAN_UNSAFE);
            break;

            case US_RESULT_NO_ROUTE:
            default:
                nav_fault(FC_NO_ROUTE, true, 0);
            break;
        }
    }
    else if ((BYPASS_NONE != g_nav.bypass) && b_idle)
    {
        bypass_next();
    }
    else
    {
        /* Waiting for the reverse, the scan or a bypass step. */
    }
}

/*!
 * @brief  Next bypass step: out 45 deg, offset, back 45, along the
 *         obstacle (longer if the car reversed first), in 45, then drive
 *         back toward the line (LINE_REACQUIRE).
 */
static void
bypass_next (void)
{
    bool const     b_out = g_nav.b_bypass_left;
    uint32_t const along = BYPASS_LENGTH_CM +
                           (g_nav.b_reversed ? REVERSE_BEFORE_SCAN_CM : 0U);

    /* bypass_step_t values follow each other, so +1 is the next step. */
    g_nav.bypass = (bypass_step_t) ((uint32_t) g_nav.bypass + 1U);

    switch (g_nav.bypass)
    {
        case BYPASS_TURN_OUT:
            g_nav.pending_seq = motion_control_turn(b_out, SPEED_TURN_CMS,
                                                    BYPASS_TURN_DEG, false);
        break;

        case BYPASS_OFFSET:
            g_nav.pending_seq = motion_control_forward(SPEED_SEARCH_CMS,
                                                       BYPASS_OFFSET_CM,
                                                       false);
        break;

        case BYPASS_TURN_BACK:
        case BYPASS_TURN_IN:
            g_nav.pending_seq = motion_control_turn(!b_out, SPEED_TURN_CMS,
                                                    BYPASS_TURN_DEG, false);
        break;

        case BYPASS_ALONG:
            g_nav.pending_seq = motion_control_forward(SPEED_SEARCH_CMS,
                                                       along, false);
        break;

        case BYPASS_RETURN:
        default:
            g_nav.pending_seq = motion_control_forward(SPEED_SEARCH_CMS,
                                                       BYPASS_OFFSET_CM * 2U,
                                                       false);
            g_nav.bypass      = BYPASS_NONE;
            enter(NAV_LINE_REACQUIRE);
        break;
    }
}

/*!
 * @brief  LINE_REACQUIRE: drive back toward the line until a sensor sees
 *         it, then turn onto it and resume.
 * @param  p_in  Inputs of this period.
 */
static void
reacquire_step (nav_in_t const * p_in)
{
    uint8_t    bits    = 0U;
    bool const b_fresh = sensor_line_read(&bits);

    if ((0U != (p_in->line_events & SENSOR_LINE_EVT_REGAINED)) ||
        (b_fresh && (0U != bits)))
    {
        g_nav.pending_seq = motion_control_turn(g_nav.b_bypass_left,
                                                SPEED_TURN_CMS,
                                                BYPASS_TURN_DEG, true);
        enter(NAV_BARCODE_ACT);                   /* wait, then follow    */
    }
    else if ((p_in->now - g_nav.state_ms) > REACQUIRE_TIMEOUT_MS)
    {
        nav_fault(FC_REACQUIRE, true, 0);
    }
    else
    {
        /* Searching. */
    }
}

/*!
 * @brief  MISSION_DONE and FAULT: operator reset (GP21 short press or the
 *         remote RESET command).
 * @param  p_in  Inputs of this period.
 */
static void
reset_step (nav_in_t const * p_in)
{
    if (p_in->b_cal_short ||
        (p_in->b_have_rc && (PROTO_CMD_RESET == p_in->remote)))
    {
        safety_clear();
        motion_control_stop();
        ultrasonic_scan_abort();
        g_nav.pending_seq    = NO_SEQ;
        g_nav.bypass         = BYPASS_NONE;
        g_nav.b_reverse_wait = false;
        g_nav.b_mag_cal      = false;
        enter(NAV_READY);
    }
}

/*!
 * @brief  Take the completion of the awaited motion command, if it is in
 *         this period's inputs. Older completions are ignored.
 * @param  p_in   Inputs of this period.
 * @param  p_res  Output result.
 * @return True if the awaited command finished.
 */
static bool
done_awaited (nav_in_t const * p_in, motion_result_t * p_res)
{
    bool b_done = false;

    if (p_in->b_have_done && (NO_SEQ != g_nav.pending_seq) &&
        (p_in->done.seq == g_nav.pending_seq))
    {
        g_nav.pending_seq = NO_SEQ;
        *p_res            = p_in->done.result;
        b_done            = true;
    }

    return (b_done);
}

/*!
 * @brief  Start (or restart after a turn) a straight line-following run.
 */
static void
follow_begin (void)
{
    g_nav.straight_start_mm = dist_now_mm();
    (void) motion_control_forward(SPEED_CRUISE_CMS, 0U, true);
}

/*!
 * @brief  Enter a state: record it, set the LED, report it.
 * @param  state  New state.
 */
static void
enter (nav_fsm_state_t state)
{
    g_nav.state    = state;
    g_nav.state_ms = systime_now_ms();
    /* nav_fsm_state_t values are 0..8, so they fit 4 bits. */
    vehicle_state_set_nav((uint8_t) state);

    switch (state)
    {
        case NAV_READY:
        case NAV_MISSION_DONE:
            hal_led_set(HAL_LED_BLUE);
        break;

        case NAV_FAULT:
            hal_led_set(HAL_LED_RED);
        break;

        case NAV_INIT:
            hal_led_set(HAL_LED_OFF);
        break;

        default:
            g_nav.b_led_online = comms_is_online();
            hal_led_set(g_nav.b_led_online ? HAL_LED_GREEN : HAL_LED_AMBER);
        break;
    }

    telemetry_post_event(EV_STATE, (uint8_t) state, 0, 0);
}

/*!
 * @brief  Stop and enter FAULT.
 * @param  code      Fault code.
 * @param  b_report  True if this FSM detected it (otherwise Safety or
 *                   Motion Control has already reported it).
 * @param  detail    EV_FAULT detail.
 */
static void
nav_fault (fault_code_t code, bool b_report, int16_t detail)
{
    motion_control_stop();
    g_nav.pending_seq = NO_SEQ;
    g_nav.bypass      = BYPASS_NONE;
    enter(NAV_FAULT);

    if (b_report)
    {
        /* Fault codes are 0..15. */
        telemetry_post_event(EV_FAULT, (uint8_t) code, detail, 0);
    }
}

/*!
 * @brief  Distance travelled since boot (Motion Control, via the record).
 * @return Millimetres.
 */
static int32_t
dist_now_mm (void)
{
    vehicle_state_t state;

    vehicle_state_read(&state);

    return (state.dist_mm);
}

/*** end of file ***/

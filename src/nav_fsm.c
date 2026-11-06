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
 *  Obstacle: while following, a standby range under US_OBSTACLE_CM
 *  (checked every period, so an obstacle that came into view during a
 *  barcode turn is still caught afterwards) stops the car and scans
 *  where it stopped. A scan without enough valid echoes, or with both
 *  sides blocked, stops with FC_NO_ROUTE detail 1 ("scan cannot proceed
 *  safely"). The bypass goes round on the BYPASS_ON_LEFT side, with
 *  BYPASS_TURN_DEG (90 deg) spot turns, step and check: turn out,
 *  step sideways, face forward and check (servo pings at PEEK_RIGHT_DEG,
 *  centre, PEEK_LEFT_DEG); still blocked -> another sideways step; first
 *  clear -> a margin step and check again. Then drive along in steps and
 *  check sideways (BYPASS_SIDE_DEG) after each, without turning; side
 *  clear -> BYPASS_PAST_CM more, turn toward the line and check ahead;
 *  still blocked -> turn back and step along. After BYPASS_MAX_STEPS
 *  either way it
 *  stops with FC_NO_ROUTE detail 3 ("obstacle too large"). The return
 *  leg is the measured sideways steps plus a margin. On the forward legs the
 *  centred ultrasonic sensor keeps watching ahead: something closer than
 *  US_OBSTACLE_CM stops the car with FC_NO_ROUTE detail 2 ("bypass path
 *  blocked") instead of driving into it. Reacquire: when a sensor sees
 *  the line the car drives LINE_SENSOR_AHEAD_CM more so its axle is on
 *  the line, turns back to its original heading and follows again; it
 *  faults (FC_REACQUIRE) if the return drive ends without the line.
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
#define NO_SEQ               (0U)
#define NO_SLOT              (0xFFU)      /* pending_slot: no barcode held */
#define DETAIL_SCAN_UNSAFE   (1)         /* FC_NO_ROUTE detail           */
#define DETAIL_BYPASS_BLOCKED (2)        /* FC_NO_ROUTE detail           */
#define DETAIL_BYPASS_TOO_BIG (3)        /* FC_NO_ROUTE detail           */
#define HALF_DEG_PER_DEG     (2)         /* heading is in 0.5 deg        */
#define FULL_TURN_HALF_DEG   (720)
#define HALF_TURN_HALF_DEG   (360)

/** Obstacle bypass steps (step-and-check, see bypass_go()). */
typedef enum
{
    BYPASS_NONE = 0,
    BYPASS_TURN_OUT,        /* 90 deg away from the obstacle             */
    BYPASS_STEP_OUT,        /* sideways BYPASS_STEP_CM                   */
    BYPASS_FACE_FWD,        /* 90 deg back to the original heading       */
    BYPASS_PEEK_WIDTH,      /* obstacle still ahead? -> TURN_OUT again   */
    BYPASS_ALONG,           /* along the obstacle, one step              */
    BYPASS_PEEK_SIDE,       /* still beside the car? -> ALONG again      */
    BYPASS_PAST,            /* side clear: drive BYPASS_PAST_CM more     */
    BYPASS_TURN_IN,         /* 90 deg toward the line                    */
    BYPASS_PEEK_LENGTH,     /* obstacle still there? -> TURN_PARALLEL    */
    BYPASS_TURN_PARALLEL    /* 90 deg back to the original heading       */
} bypass_step_t;

/** Dashboard calibration: compass spin, then a checked 90 deg turn. */
typedef enum
{
    CAL_IDLE = 0,
    CAL_SPIN,               /* compass calibration spin                  */
    CAL_SETTLE_BEFORE,      /* stopped: compass settles, heading noted   */
    CAL_TURN,               /* one BYPASS_TURN_DEG turn on the encoders  */
    CAL_SETTLE_AFTER        /* stopped: compass settles, angle reported  */
} cal_stage_t;

/** One debounced button. */
typedef struct
{
    bool     b_stable;       /* debounced level, true = pressed          */
    uint8_t  count;
} button_t;

/** Inputs of one period. */
typedef struct
{
    uint32_t            now;
    bool                b_start;          /* GP20 pressed                */
    bool                b_cal_short;      /* GP21 released               */
    bool                b_have_bar;
    uint8_t             bar_slot;         /* 0..3 = action wire code     */
    bool                b_have_done;
    motion_done_t       done;
    bool                b_have_terrain;
    imu_terrain_event_t terrain;
    bool                b_have_us;
    us_result_t         us_result;
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
    uint8_t         pending_slot;         /* barcode action held for the
                                             next junction (NO_SLOT = none,
                                             latest barcode wins)         */
    bypass_step_t   bypass;
    bool            b_bypass_left;
    barcode_map_action_t bar_turn;        /* turn to make once the axle
                                             reaches the junction        */
    bool            b_on_line_step;       /* reacquire: line seen, axle
                                             being driven onto it        */
    int32_t         leg_start_mm;         /* start of a sideways step    */
    int32_t         offset_mm;            /* sideways steps, measured    */
    uint32_t        width_steps;          /* sideways steps driven       */
    bool            b_width_margin;       /* last step out was the margin */
    uint32_t        length_steps;         /* extra along steps driven    */
    cal_stage_t     cal_stage;
    uint32_t        cal_ms;               /* start of a settle wait      */
    uint16_t        cal_hdg0;             /* heading before the turn     */
    bool            b_cal_hdg0;           /* cal_hdg0 is usable          */
    bool            b_led_online;
    button_t        btn_start;
    button_t        btn_cal;
} nav_ctx_t;

static ID        g_remote_q = 0;
static nav_ctx_t g_nav;                   /* task only                   */

static void    inputs_read (nav_in_t * p_in);
static bool    button_press (button_t * p_btn, bool b_raw);
static bool    button_release (button_t * p_btn, bool b_raw);
static bool    global_transitions (nav_in_t const * p_in);
static void    state_step (nav_in_t const * p_in);
static void    init_step (nav_in_t const * p_in);
static void    ready_step (nav_in_t const * p_in);
static void    follow_step (nav_in_t const * p_in);
static void    barcode_act (uint8_t slot);
static void    obstacle_begin (void);
static void    obstacle_step (nav_in_t const * p_in);
static void    bypass_go (bypass_step_t step);
static void    bypass_step_done (void);
static void    bypass_peek_result (bool b_blocked);
static void    bypass_return (void);
static void    reacquire_step (nav_in_t const * p_in);
static bool    path_is_blocked (void);
static void    cal_motion_done (uint32_t now);
static void    cal_settle_step (uint32_t now);
static bool    heading_read (uint16_t * p_half_deg);
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
    g_nav.state        = NAV_INIT;
    g_nav.pending_slot = NO_SLOT;

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
 * @param  cmd  proto_cmd_t value, 0..7.
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
    nav_in_t inputs;

    (void) stacd;
    (void) p_exinf;

    enter(NAV_INIT);

    for (;;)
    {
        systime_period_wait(&next_us, PERIOD_NAV_POLL_MS);
        safety_heartbeat(SAFETY_HB_NAV);

        inputs_read(&inputs);

        /* WiFi status LED while running. */
        if ((g_nav.state >= NAV_LINE_FOLLOW) &&
            (g_nav.state <= NAV_LINE_REACQUIRE) &&
            (comms_is_online() != g_nav.b_led_online))
        {
            g_nav.b_led_online = comms_is_online();
            hal_led_set(g_nav.b_led_online ? HAL_LED_GREEN : HAL_LED_AMBER);
        }

        if (!global_transitions(&inputs))
        {
            state_step(&inputs);
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

    p_in->b_start      = button_press(&g_nav.btn_start, hal_btn_start_raw());
    p_in->b_cal_short  = button_release(&g_nav.btn_cal, hal_btn_cal_raw());
    p_in->b_have_bar     = barcode_take(&p_in->bar_slot);
    p_in->b_have_done    = motion_control_done_take(&p_in->done);
    p_in->b_have_terrain = imu_terrain_event_take(&p_in->terrain);
    p_in->b_have_us      = ultrasonic_scan_result_take(&p_in->us_result);
    p_in->b_have_rc      = (tk_rcv_mbf(g_remote_q, &remote, TMO_POL) > 0);
    p_in->line_events    = sensor_line_events_take();

    if (p_in->b_have_rc)
    {
        /* comms.c queues only validated commands 0..7. */
        p_in->remote = (proto_cmd_t) remote;
    }
}

/*!
 * @brief  Debounce a button.
 * @param  p_btn  Button state.
 * @param  b_raw  Raw level, true = pressed.
 * @return True once when the debounced button is pressed.
 */
static bool
button_press (button_t * p_btn, bool b_raw)
{
    bool b_pressed = false;

    if (b_raw != p_btn->b_stable)
    {
        p_btn->count++;

        if (p_btn->count >= BTN_STABLE_TICKS)
        {
            p_btn->b_stable = b_raw;
            p_btn->count    = 0U;
            b_pressed       = b_raw;
        }
    }
    else
    {
        p_btn->count = 0U;
    }

    return (b_pressed);
}

/*!
 * @brief  Release of a button (GP21: line calibration, reset).
 * @param  p_btn  Button state.
 * @param  b_raw  Raw level, true = pressed.
 * @return True once when the debounced button is released.
 */
static bool
button_release (button_t * p_btn, bool b_raw)
{
    bool const b_was_pressed = p_btn->b_stable;

    (void) button_press(p_btn, b_raw);

    return (b_was_pressed && (!p_btn->b_stable));
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
                if (b_done && (BA_NONE != g_nav.bar_turn))
                {
                    /* Axle now on the junction: turn onto the branch. */
                    barcode_map_action_t const turn = g_nav.bar_turn;

                    g_nav.bar_turn    = BA_NONE;
                    g_nav.pending_seq = (BA_UTURN == turn) ?
                        motion_control_uturn() :
                        motion_control_turn(BA_LEFT == turn, SPEED_TURN_CMS,
                                            BARCODE_TURN_DEG, true);
                }
                else if (b_done)
                {
                    follow_begin();               /* a turn ends the run  */
                    enter(NAV_LINE_FOLLOW);
                }
                else
                {
                    /* Driving onto the junction or turning. */
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

        /* READY: a completion moves the calibration on. */
        if (b_done && (NAV_READY == g_nav.state))
        {
            cal_motion_done(p_in->now);
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
 * @brief  READY: start, line calibration (GP21) or the dashboard
 *         calibration: compass spin, then one checked 90 deg turn.
 * @param  p_in  Inputs of this period.
 */
static void
ready_step (nav_in_t const * p_in)
{
    if (CAL_IDLE != g_nav.cal_stage)
    {
        cal_settle_step(p_in->now);               /* spins end in state_step */
    }
    else if (p_in->b_start ||
             (p_in->b_have_rc && (PROTO_CMD_START == p_in->remote)))
    {
        follow_begin();
        enter(NAV_LINE_FOLLOW);
    }
    else if (p_in->b_have_rc && (PROTO_CMD_MAG_CAL == p_in->remote))
    {
        g_nav.cal_stage = CAL_SPIN;
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
 *
 *  A decoded barcode does not act at once: its action is held in
 *  pending_slot and executed at the next junction (both line sensors
 *  raw black). The barcode sensor reads the code somewhere along the
 *  line; the car keeps following until it reaches the junction tied to
 *  that letter, then turns. The latest barcode wins if two are read
 *  before a junction. The line-lost timeout waits while a barcode is
 *  read: the hold drives straight without steering, and both line
 *  sensors can stay white for all of it (bounded by BAR_STALL_TIMEOUT_US).
 *
 * @param  p_in  Inputs of this period.
 */
static void
follow_step (nav_in_t const * p_in)
{
    uint32_t const in_state = p_in->now - g_nav.state_ms;

    /* A barcode read this period updates the pending action first, so a
     * junction in the same period acts on the freshest letter. */
    if (p_in->b_have_bar)
    {
        g_nav.pending_slot = p_in->bar_slot;      /* latest wins          */
    }

    if (path_is_blocked())
    {
        obstacle_begin();
    }
    else if ((sensor_line_black_age_ms() > LINE_LOST_MS) &&
             (in_state > LINE_LOST_MS) && !barcode_is_reading())
    {
        safety_report(SAFETY_EVT_LINE_LOST);      /* Safety trips         */
    }
    else if ((NO_SLOT != g_nav.pending_slot) &&
             (0U != (p_in->line_events & SENSOR_LINE_EVT_JUNCTION)))
    {
        /* Both line sensors saw raw black: execute the held barcode
         * action. Both following states act on it: a hump (or a false
         * hump detection) must not make the car drive past its turn. */
        uint8_t const slot = g_nav.pending_slot;

        g_nav.pending_slot = NO_SLOT;
        barcode_act(slot);
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
 * @brief  Act on a barcode (the Line task already looked its letter up
 *         and posted the BARCODE event).
 * @param  slot  0..3 = action wire code (PROTO_ACT_*).
 */
static void
barcode_act (uint8_t slot)
{
    /* slot is 0..3 and BA_STRAIGHT..BA_UTURN follow wire codes 0..3. */
    barcode_map_action_t const action =
        (barcode_map_action_t) ((uint32_t) BA_STRAIGHT +
                                ((uint32_t) slot & VEHICLE_BAR_SLOT));
    uint8_t                    seq    = NO_SEQ;

    switch (action)
    {
        case BA_LEFT:
        case BA_RIGHT:
        case BA_UTURN:
            if (0U != JUNCTION_AHEAD_CM)
            {
                /* The line sensors are ahead of the axle: drive the axle
                 * onto the junction first, so the spot turn swings the
                 * sensors onto the branch (NAV_BARCODE_ACT turns next). */
                g_nav.bar_turn = action;
                seq = motion_control_forward(SPEED_SEARCH_CMS,
                                             JUNCTION_AHEAD_CM, false);
            }
            else if (BA_UTURN == action)
            {
                seq = motion_control_uturn();
            }
            else
            {
                seq = motion_control_turn(BA_LEFT == action, SPEED_TURN_CMS,
                                          BARCODE_TURN_DEG, true);
            }
        break;

        case BA_STRAIGHT:
            seq = motion_control_forward(SPEED_CRUISE_CMS,
                                         BARCODE_STRAIGHT_CM, false);
        break;

        case BA_NONE:
        default:
            /* Cannot happen: the slot always names an action. */
        break;
    }

    if (NO_SEQ != seq)
    {
        g_nav.pending_seq = seq;
        enter(NAV_BARCODE_ACT);
    }
}

/*!
 * @brief  Obstacle ahead: stop and scan from here.
 */
static void
obstacle_begin (void)
{
    motion_control_stop();
    g_nav.bypass = BYPASS_NONE;
    ultrasonic_scan_request();
    enter(NAV_OBSTACLE_AVOID);
}

/*!
 * @brief  OBSTACLE_AVOID: act on the scan result and run the bypass
 *         steps.
 * @param  p_in  Inputs of this period.
 */
static void
obstacle_step (nav_in_t const * p_in)
{
    bool const b_idle = (NO_SEQ == g_nav.pending_seq);

    if (p_in->b_have_us && (BYPASS_NONE == g_nav.bypass))
    {
        switch (p_in->us_result)
        {
            case US_RESULT_CONTINUE:
                (void) motion_control_forward(SPEED_CRUISE_CMS, 0U, true);
                enter(NAV_LINE_FOLLOW);
            break;

            case US_RESULT_LEFT:
            case US_RESULT_RIGHT:
                /* Fixed side (BYPASS_ON_LEFT, set by track direction);
                 * the scan only decides whether a bypass is needed. */
                g_nav.b_bypass_left  = (0 != BYPASS_ON_LEFT);
                g_nav.offset_mm      = 0;
                g_nav.width_steps    = 0U;
                g_nav.b_width_margin = false;
                g_nav.length_steps  = 0U;
                bypass_go(BYPASS_TURN_OUT);
            break;

            case US_RESULT_UNSAFE:
                nav_fault(FC_NO_ROUTE, true, DETAIL_SCAN_UNSAFE);
            break;

            default:
                nav_fault(FC_NO_ROUTE, true, 0);
            break;
        }
    }
    else if ((BYPASS_PEEK_WIDTH == g_nav.bypass) ||
             (BYPASS_PEEK_SIDE == g_nav.bypass) ||
             (BYPASS_PEEK_LENGTH == g_nav.bypass))
    {
        /* Car stopped; only the check's answer moves it on. */
        if (p_in->b_have_us && ((US_RESULT_PEEK_CLEAR == p_in->us_result) ||
                                (US_RESULT_PEEK_BLOCKED == p_in->us_result)))
        {
            bypass_peek_result(US_RESULT_PEEK_BLOCKED == p_in->us_result);
        }
    }
    else if (((BYPASS_STEP_OUT == g_nav.bypass) ||
              (BYPASS_ALONG == g_nav.bypass) ||
              (BYPASS_PAST == g_nav.bypass)) && (!b_idle) &&
             path_is_blocked())
    {
        /* Driving a step into something, e.g. a second obstacle. */
        nav_fault(FC_NO_ROUTE, true, DETAIL_BYPASS_BLOCKED);
    }
    else if ((BYPASS_NONE != g_nav.bypass) && b_idle)
    {
        bypass_step_done();
    }
    else
    {
        /* Waiting for the scan or a bypass step. */
    }
}

/*!
 * @brief  Start a bypass step. Going left: away = left turn, back to
 *         the original heading and toward the line = right turns.
 * @param  step  Step to start.
 */
static void
bypass_go (bypass_step_t step)
{
    bool const b_out = g_nav.b_bypass_left;

    g_nav.bypass = step;

    switch (step)
    {
        case BYPASS_TURN_OUT:
        case BYPASS_TURN_PARALLEL:
            g_nav.pending_seq = motion_control_turn(b_out, SPEED_TURN_CMS,
                                                    BYPASS_TURN_DEG, false);
        break;

        case BYPASS_FACE_FWD:
        case BYPASS_TURN_IN:
            g_nav.pending_seq = motion_control_turn(!b_out, SPEED_TURN_CMS,
                                                    BYPASS_TURN_DEG, false);
        break;

        case BYPASS_STEP_OUT:
            g_nav.leg_start_mm = dist_now_mm();
            g_nav.pending_seq  = motion_control_forward(
                SPEED_SEARCH_CMS,
                g_nav.b_width_margin ? BYPASS_WIDTH_MARGIN_CM :
                                       BYPASS_STEP_CM,
                false);
        break;

        case BYPASS_ALONG:
            g_nav.pending_seq = motion_control_forward(
                SPEED_SEARCH_CMS,
                (0U == g_nav.length_steps) ? BYPASS_ALONG_FIRST_CM :
                                             BYPASS_ALONG_STEP_CM,
                false);
        break;

        case BYPASS_PAST:
            g_nav.pending_seq = motion_control_forward(SPEED_SEARCH_CMS,
                                                       BYPASS_PAST_CM,
                                                       false);
        break;

        case BYPASS_PEEK_SIDE:
            ultrasonic_scan_side_peek();          /* car is stopped       */
        break;

        case BYPASS_PEEK_WIDTH:
        case BYPASS_PEEK_LENGTH:
            ultrasonic_scan_peek();               /* car is stopped       */
        break;

        case BYPASS_NONE:
        default:
            /* Nothing to start. */
        break;
    }
}

/*!
 * @brief  The awaited motion of a bypass step finished: start the next.
 */
static void
bypass_step_done (void)
{
    switch (g_nav.bypass)
    {
        case BYPASS_TURN_OUT:      bypass_go(BYPASS_STEP_OUT);    break;
        case BYPASS_STEP_OUT:
            g_nav.offset_mm += dist_now_mm() - g_nav.leg_start_mm;
            bypass_go(BYPASS_FACE_FWD);
        break;
        case BYPASS_FACE_FWD:      bypass_go(BYPASS_PEEK_WIDTH);  break;
        case BYPASS_ALONG:         bypass_go(BYPASS_PEEK_SIDE);   break;
        case BYPASS_PAST:          bypass_go(BYPASS_TURN_IN);     break;
        case BYPASS_TURN_IN:       bypass_go(BYPASS_PEEK_LENGTH); break;
        case BYPASS_TURN_PARALLEL: bypass_go(BYPASS_ALONG);       break;
        default:
            /* The checks finish on their answer, not on a motion. */
        break;
    }
}

/*!
 * @brief  Act on a bypass check. Width: still ahead -> one more step
 *         sideways; first clear -> one BYPASS_WIDTH_MARGIN_CM step and
 *         check again; clear after the margin -> drive along. Side:
 *         still beside -> one more step along, clear -> BYPASS_PAST_CM
 *         and turn toward the line. Facing the line: blocked -> back to
 *         parallel and one more step along, else drive back to the
 *         line. Gives up after BYPASS_MAX_STEPS.
 * @param  b_blocked  True if the check saw the obstacle.
 */
static void
bypass_peek_result (bool b_blocked)
{
    if (BYPASS_PEEK_WIDTH == g_nav.bypass)
    {
        g_nav.width_steps++;

        if (!b_blocked && !g_nav.b_width_margin)
        {
            /* Clear, but the car's side may be only ~2.5 cm from the
             * edge: step out BYPASS_WIDTH_MARGIN_CM more and check
             * again, so the spot turns beside it clear the obstacle. */
            g_nav.b_width_margin = true;
            bypass_go(BYPASS_TURN_OUT);
        }
        else if (!b_blocked)
        {
            bypass_go(BYPASS_ALONG);
        }
        else if (g_nav.width_steps >= BYPASS_MAX_STEPS)
        {
            nav_fault(FC_NO_ROUTE, true, DETAIL_BYPASS_TOO_BIG);
        }
        else
        {
            g_nav.b_width_margin = false;         /* a full step again    */
            bypass_go(BYPASS_TURN_OUT);
        }
    }
    else if (BYPASS_PEEK_SIDE == g_nav.bypass)
    {
        if (!b_blocked)
        {
            bypass_go(BYPASS_PAST);               /* sensor is past it    */
        }
        else if (g_nav.length_steps >= BYPASS_MAX_STEPS)
        {
            nav_fault(FC_NO_ROUTE, true, DETAIL_BYPASS_TOO_BIG);
        }
        else
        {
            g_nav.length_steps++;
            bypass_go(BYPASS_ALONG);
        }
    }
    else
    {
        /* Facing the line after BYPASS_PAST: normally clear. */
        if (!b_blocked)
        {
            bypass_return();
        }
        else if (g_nav.length_steps >= BYPASS_MAX_STEPS)
        {
            nav_fault(FC_NO_ROUTE, true, DETAIL_BYPASS_TOO_BIG);
        }
        else
        {
            g_nav.length_steps++;
            bypass_go(BYPASS_TURN_PARALLEL);
        }
    }
}

/*!
 * @brief  Past the obstacle and facing the line: drive back for the
 *         measured sideways distance plus BYPASS_RETURN_EXTRA_CM
 *         (LINE_REACQUIRE).
 */
static void
bypass_return (void)
{
    /* BYPASS_RETURN_EXTRA_CM is a small constant (15). */
    int32_t const back_cm = (g_nav.offset_mm / MM_PER_CM) +
                            (int32_t) BYPASS_RETURN_EXTRA_CM;

    /* offset_mm is at most BYPASS_MAX_STEPS x BYPASS_STEP_CM plus
     * overrun; motion_control_forward() clamps to its 511 cm field. */
    g_nav.pending_seq    = motion_control_forward(
        SPEED_SEARCH_CMS, (uint32_t) ((back_cm > 0) ? back_cm : 0), false);
    g_nav.bypass         = BYPASS_NONE;
    g_nav.b_on_line_step = false;
    enter(NAV_LINE_REACQUIRE);
}

/*!
 * @brief  LINE_REACQUIRE: drive back toward the line until a sensor sees
 *         it, drive the axle onto it (LINE_SENSOR_AHEAD_CM), then turn
 *         onto it toward the original heading and resume. Stops if the
 *         path ahead is blocked, and faults if the return drive ends (or
 *         the backstop REACQUIRE_TIMEOUT_MS expires) without the line.
 * @param  p_in  Inputs of this period.
 */
static void
reacquire_step (nav_in_t const * p_in)
{
    uint8_t    bits    = 0U;
    bool const b_fresh = sensor_line_read(&bits);
    bool const b_timed = ((p_in->now - g_nav.state_ms) >
                          REACQUIRE_TIMEOUT_MS);

    if (g_nav.b_on_line_step)
    {
        if (NO_SEQ == g_nav.pending_seq)
        {
            /* Axle on the line: the spot turn puts the sensors on it. */
            g_nav.b_on_line_step = false;
            g_nav.pending_seq    = motion_control_turn(g_nav.b_bypass_left,
                                                       SPEED_TURN_CMS,
                                                       BYPASS_TURN_DEG,
                                                       true);
            enter(NAV_BARCODE_ACT);               /* wait, then follow    */
        }
        else if (b_timed)
        {
            nav_fault(FC_REACQUIRE, true, 0);
        }
        else
        {
            /* Driving the axle onto the line. */
        }
    }
    else if ((0U != (p_in->line_events & SENSOR_LINE_EVT_REGAINED)) ||
             (b_fresh && (0U != bits)))
    {
        /* Latest command wins, so this replaces the return drive. */
        if (0U != LINE_SENSOR_AHEAD_CM)
        {
            g_nav.b_on_line_step = true;
            g_nav.pending_seq    = motion_control_forward(
                SPEED_SEARCH_CMS, LINE_SENSOR_AHEAD_CM, false);
        }
        else
        {
            g_nav.pending_seq = motion_control_turn(g_nav.b_bypass_left,
                                                    SPEED_TURN_CMS,
                                                    BYPASS_TURN_DEG, true);
            enter(NAV_BARCODE_ACT);               /* wait, then follow    */
        }
    }
    else if (path_is_blocked())
    {
        nav_fault(FC_NO_ROUTE, true, DETAIL_BYPASS_BLOCKED);
    }
    else if ((NO_SEQ == g_nav.pending_seq) || b_timed)
    {
        /* The return drive finished (state_step took its completion)
         * and no sensor saw the line, or the backstop expired. */
        nav_fault(FC_REACQUIRE, true, 0);
    }
    else
    {
        /* Searching. */
    }
}

/*!
 * @brief  Calibration: the awaited spin or turn finished. After the
 *         compass spin the offsets are applied; both then wait
 *         CAL_SETTLE_MS with the motors off, so the compass reads true.
 * @param  now  Time now, ms.
 */
static void
cal_motion_done (uint32_t now)
{
    if (CAL_SPIN == g_nav.cal_stage)
    {
        imu_terrain_mag_cal_finish();
        g_nav.cal_stage = CAL_SETTLE_BEFORE;
        g_nav.cal_ms    = now;
    }
    else if (CAL_TURN == g_nav.cal_stage)
    {
        g_nav.cal_stage = CAL_SETTLE_AFTER;
        g_nav.cal_ms    = now;
    }
    else
    {
        /* Not calibrating. */
    }
}

/*!
 * @brief  Calibration: after the settle wait, note the heading and make
 *         one BYPASS_TURN_DEG turn on the encoders; after the next wait,
 *         report the angle the compass saw as EV_TURN_XCHECK (encoder vs
 *         IMU), so TURN_TRACK_WIDTH_M can be scaled by encoder / IMU.
 * @param  now  Time now, ms.
 */
static void
cal_settle_step (uint32_t now)
{
    uint16_t hdg = 0U;

    if ((CAL_SETTLE_BEFORE == g_nav.cal_stage) &&
        ((now - g_nav.cal_ms) >= CAL_SETTLE_MS))
    {
        g_nav.b_cal_hdg0  = heading_read(&g_nav.cal_hdg0);
        g_nav.cal_stage   = CAL_TURN;
        g_nav.pending_seq = motion_control_turn(true, SPEED_TURN_CMS,
                                                BYPASS_TURN_DEG, false);
    }
    else if ((CAL_SETTLE_AFTER == g_nav.cal_stage) &&
             ((now - g_nav.cal_ms) >= CAL_SETTLE_MS))
    {
        g_nav.cal_stage = CAL_IDLE;

        if (g_nav.b_cal_hdg0 && heading_read(&hdg))
        {
            /* Headings are 0..719 half degrees: the difference wrapped
             * to +/-360 is the angle turned; under 180 deg, so the
             * event's 9-bit field holds it. */
            int32_t turned = ((int32_t) hdg - (int32_t) g_nav.cal_hdg0) %
                             FULL_TURN_HALF_DEG;

            turned = (turned > HALF_TURN_HALF_DEG) ?
                     (turned - FULL_TURN_HALF_DEG) :
                     ((turned < -HALF_TURN_HALF_DEG) ?
                      (turned + FULL_TURN_HALF_DEG) : turned);
            turned = (turned < 0) ? -turned : turned;
            /* Both values are 0..180 deg, so they fit an int16_t. */
            telemetry_post_event(EV_TURN_XCHECK, 0U,
                                 (int16_t) BYPASS_TURN_DEG,
                                 (int16_t) ((turned + 1) /
                                            HALF_DEG_PER_DEG));
        }
    }
    else
    {
        /* Spinning, turning or settling. */
    }
}

/*!
 * @brief  Compass heading for the calibration check.
 * @param  p_half_deg  Output heading, 0.5 deg (0..719).
 * @return True if the IMU is valid and the reading no older than
 *         IMU_MAX_AGE_MS.
 */
static bool
heading_read (uint16_t * p_half_deg)
{
    vehicle_state_t state;

    vehicle_state_read(&state);
    *p_half_deg = state.heading_half_deg;

    return (state.b_imu_valid &&
            (systime_stamp_age(state.imu_stamp) <= IMU_MAX_AGE_MS));
}

/*!
 * @brief  Check the standby range straight ahead (servo centred after the
 *         scan) while the car drives a bypass or return leg. Every forward
 *         leg follows a 90 deg spot turn of ~0.8 s, so by then the 100 ms
 *         standby ping already reads the new heading, not the obstacle
 *         the scan stopped for.
 * @return True if an echo closer than US_OBSTACLE_CM is ahead.
 */
static bool
path_is_blocked (void)
{
    uint16_t const range = ultrasonic_scan_front_cm();

    return ((US_RANGE_NO_ECHO != range) && (range < US_OBSTACLE_CM));
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
        g_nav.pending_slot   = NO_SLOT;
        g_nav.bar_turn       = BA_NONE;
        g_nav.bypass         = BYPASS_NONE;
        g_nav.b_on_line_step = false;
        g_nav.cal_stage      = CAL_IDLE;
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
    g_nav.pending_slot = NO_SLOT;         /* drop any stale barcode       */
    g_nav.bar_turn     = BA_NONE;
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

    telemetry_post_event(EV_STATE, (uint8_t) state, 0, 0);  /* 0..8 */
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
    ultrasonic_scan_abort();                      /* end side tracking    */
    g_nav.pending_seq  = NO_SEQ;
    g_nav.pending_slot = NO_SLOT;
    g_nav.bypass       = BYPASS_NONE;
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

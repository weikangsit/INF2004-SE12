/** @file    motion_control.c
 *  @brief   Motion Control task (Buddy 2), see motion_control.h.
 *
 *  STOPPED -> MOVING{STRAIGHT, REVERSE, TURN_L, TURN_R, U_TURN}
 *          -> STOPPED | SAFE_STOP | FAULT
 *
 *  Per-wheel speed PID on the encoders (float inside this task only,
 *  Tribunal 1 Buddy 2 decision). Straight runs steer on the two digital
 *  line sensors; off the line they equalise wheel distance and hold the
 *  IMU heading. Turns end on encoder arc length, or early on a new line
 *  branch. A ramp limits set-point changes to ACCEL_LIMIT_MPS2.
 *
 *  If an encoder fails the power-on self-test the task drops to open loop
 *  (feed-forward duty, speed and distance estimated from the command), as
 *  the Week 6 feedback asked; stall detection is then off and only the
 *  command time limit protects the motors.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored;
 *  (UB *) and (int) casts match tm_printf()'s parameter and %d types.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <tm/tmonitor.h>

#include "port.h"

#include "car_config.h"
#include "fault.h"
#include "motion_control.h"
#include "motor_driver.h"
#include "safety.h"
#include "sensor_line.h"
#include "systime.h"
#include "telemetry.h"
#include "vehicle_state.h"

#define PI_F                 (3.14159265f)
#define M_PER_COUNT          ((PI_F * WHEEL_DIAMETER_M) / ENC_COUNTS_PER_REV)
#define DT_S                 ((float32_t) PERIOD_MOTION_MS / 1000.0f)
#define HALF                 (0.5f)
#define DEG_PER_RAD          (180.0f / PI_F)
#define HALF_DEG             (2.0f)        /* heading is in 0.5 deg       */
#define MM_PER_M             (1000.0f)
#define PCT_PER_UNIT         (100.0f)
#define PM_PER_UNIT          (1000.0f)
#define CM_PER_M             (100.0f)
#define LIMIT_FACTOR         (2000.0f)     /* 2x nominal, in ms           */
#define LIMIT_EXTRA_MS       (2000U)
#define MIN_LIMIT_SPEED_MPS  (0.05f)
#define TEST_PHASES          (4U)
#define SWEEP_WHEELS         (2U)
#define SWEEP_FAILED_PCT     (100)     /* dead band never found          */
#define PCT_PER_PM           (10)
#define KFF_X100             (100.0f)
#define MS_PER_S             (1000.0f)
#define TWO                  (2.0f)
#define COUNT_PRINT_MS       (500U)    /* encoder print when stopped  */

/* Packed command word (32 bits, Change Requirement 1). */
#define CMD_ID_SHIFT         (0U)
#define CMD_ID_MASK          (0x7U)
#define CMD_SEQ_SHIFT        (3U)
#define CMD_SEQ_MASK         (0x7U)
#define CMD_FOLLOW_BIT       (1U << 6)
#define CMD_END_LINE_BIT     (1U << 7)
#define CMD_SPEED_SHIFT      (8U)
#define CMD_SPEED_MASK       (0x3FU)       /* 0..63 cm/s                  */
#define CMD_DIST_SHIFT       (14U)
#define CMD_DIST_MASK        (0x1FFU)      /* 0..511 cm, 0 = no limit     */
#define CMD_ANGLE_SHIFT      (23U)
#define CMD_ANGLE_MASK       (0x1FFU)      /* 0..511 deg                  */

/* Packed completion byte. */
#define DONE_SEQ_MASK        (0x7U)
#define DONE_RESULT_SHIFT    (3U)
#define DONE_RESULT_MASK     (0x3U)

#define CMD_Q_DEPTH          (4U)
#define DONE_Q_DEPTH         (4U)
#define MBF_SLOT_BYTES       (8U)          /* 4-byte header + 4 bytes     */

/* Stall detail bits (EV_FAULT value1). */
#define WHY_LEFT_SILENT      (1U << 0)
#define WHY_RIGHT_SILENT     (1U << 1)
#define WHY_TIME_LIMIT       (1U << 2)

/* Self-test result codes (EV_ENC_CAL code). */
#define ENC_CAL_OK           (0U)
#define ENC_CAL_LEFT_SILENT  (1U)
#define ENC_CAL_RIGHT_SILENT (2U)
#define ENC_CAL_ROTATED      (4U)

/** Command ids (3 bits). */
typedef enum
{
    CMD_STOP = 0,
    CMD_FWD,
    CMD_BWD,
    CMD_TURN_L,
    CMD_TURN_R,
    CMD_UTURN,
    CMD_ENC_CAL,
    CMD_MOTOR_TEST
} cmd_id_t;

/** Task states. The numbers 0..5 are also the stall "motion state"
 *  reported in EV_FAULT (web/static/proto.js MOTION_STATES). */
typedef enum
{
    MS_STOPPED = 0,
    MS_STRAIGHT,
    MS_REVERSE,
    MS_TURN_L,
    MS_TURN_R,
    MS_UTURN,
    MS_ENC_CAL,
    MS_MOTOR_TEST,
    MS_SAFE_STOP,
    MS_FAULT,
    MS_DEADBAND
} motion_mode_t;

/** Stages of the dead-band sweep for one wheel. */
typedef enum
{
    SWEEP_SETTLE = 0,
    SWEEP_RAMP,
    SWEEP_SPINUP,
    SWEEP_MEASURE
} sweep_stage_t;

typedef struct
{
    float32_t integ;
    float32_t prev_err;
} wheel_pid_t;

/** Open-loop model of one wheel: duty = dead_band + slope x speed. */
typedef struct
{
    float32_t dead_band;     /* duty                                      */
    float32_t slope;         /* duty per m/s                              */
} wheel_ff_t;

/** Everything the task keeps between periods (task-private). */
typedef struct
{
    motion_mode_t mode;
    cmd_id_t      id;
    uint8_t       seq;
    bool          b_follow;
    bool          b_steered;     /* line steering seen in this command    */
    uint32_t      steer_ms;      /* time of the last steering correction  */
    int32_t       line_last;     /* side that last saw black: -1, 0, +1   */
    bool          b_end_on_line;
    float32_t     speed_mps;
    float32_t     dist_m;
    float32_t     angle_deg;
    float32_t     dist_l;            /* wheel travel since boot, m            */
    float32_t     dist_r;
    float32_t     dist_l0;           /* at command start                      */
    float32_t     dist_r0;
    float32_t     speed_l;       /* measured (or estimated), m/s          */
    float32_t     speed_r;
    float32_t     v_base;        /* ramped base speed magnitude, m/s      */
    float32_t     tgt_l;         /* last wheel targets, m/s               */
    float32_t     tgt_r;
    float32_t     cmd_l;         /* slew-limited targets fed to the PID   */
    float32_t     cmd_r;
    float32_t     duty_l;        /* -1..1                                 */
    float32_t     duty_r;
    float32_t     hdg0;          /* heading at command start, deg         */
    bool          b_hdg0;        /* hdg0 came from fresh IMU data         */
    uint32_t      start_ms;
    uint32_t      limit_ms;      /* 0 = no time limit                     */
    int32_t       prev_cl;
    int32_t       prev_cr;
    int32_t       hist_l[SPEED_AVG_PERIODS];  /* last count deltas       */
    int32_t       hist_r[SPEED_AVG_PERIODS];
    uint32_t      hist_idx;
    int32_t       printed_cl;    /* counts at the last stopped print      */
    int32_t       printed_cr;
    uint32_t      printed_ms;
    int32_t       cal_l0;        /* raw counts at self-test / phase start */
    int32_t       cal_r0;
    uint32_t      test_phase;
    sweep_stage_t sweep_stage;
    int32_t       sweep_pm;      /* duty of the current sweep step        */
    int32_t       sweep_db_pm;   /* dead band found                       */
    uint32_t      stage_ms;
    int32_t       stage_count0;
    wheel_pid_t   pid_l;
    wheel_pid_t   pid_r;
} motion_ctx_t;

/** Inputs sampled at the start of each period. */
typedef struct
{
    uint32_t  now_ms;
    int32_t   count_l;       /* raw encoder counts                        */
    int32_t   count_r;
    uint8_t   line_bits;     /* VEHICLE_IR_*                              */
    bool      b_line_fresh;
    float32_t heading_deg;
    bool      b_imu_fresh;
} motion_in_t;

static ID           g_cmd_q        = 0;
static ID           g_done_q       = 0;
static uint8_t      g_next_seq     = 1U;     /* Nav only, see header     */
static volatile bool g_b_open_loop = false;
static int32_t      g_sign_l       = ENC_SIGN_LEFT;
static int32_t      g_sign_r       = ENC_SIGN_RIGHT;

static wheel_ff_t const g_ff_l = { MOTOR_DEADBAND_L, PID_KFF_L };
static wheel_ff_t const g_ff_r = { MOTOR_DEADBAND_R, PID_KFF_R };
static motion_ctx_t g_ctx;

static uint8_t   cmd_send (cmd_id_t id, uint32_t speed_cms, uint32_t dist_cm,
                           uint32_t angle_deg, bool b_follow, bool b_end);
static void      inputs_read (motion_in_t * p_in);
static void      speed_update (motion_in_t const * p_in);
static void      commands_take (motion_in_t const * p_in);
static void      command_start (uint32_t word, motion_in_t const * p_in);
static bool      targets_set (motion_in_t const * p_in);
static bool      straight_targets (motion_in_t const * p_in, float32_t sign);
static bool      turn_targets (motion_in_t const * p_in);
static void      ramp_base (float32_t target);
static void      outputs_drive (motion_in_t const * p_in, bool b_done);
static void      motor_test_step (motion_in_t const * p_in);
static void      deadband_step (motion_in_t const * p_in);
static void      sweep_wheel_done (motion_in_t const * p_in, int32_t db_pct,
                                   int32_t kff_x100);
static void      sweep_drive (int32_t duty_pm);
static float32_t end_speed_cap (float32_t remaining_m);
static float32_t line_steer (motion_in_t const * p_in);
static void      wheels_slew (void);
static void      self_test_step (motion_in_t const * p_in);
static void      closed_loop_drive (motion_in_t const * p_in);
static uint32_t  edge_clamp (uint32_t edge_ms, uint32_t now);
static void      brake_to (motion_mode_t mode);
static void      done_post (motion_result_t result);
static float32_t feed_forward (float32_t target,
                               wheel_ff_t const * p_ff);
static void      counts_print (motion_in_t const * p_in);
static float32_t pid_step (wheel_pid_t * p_pid, float32_t target,
                           float32_t measured, wheel_ff_t const * p_ff);
static void      pid_reset (wheel_pid_t * p_pid);
static float32_t clamp_f (float32_t value, float32_t low, float32_t high);
static float32_t wrap180 (float32_t deg);
static int32_t   round_i (float32_t value);
static void      state_publish (void);

/*!
 * @brief  Create the command and completion queues. Call from usermain()
 *         before the tasks start.
 */
void
motion_control_init (void)
{
    T_CMBF create;

    (void) memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.mode = MS_STOPPED;

    (void) memset(&create, 0, sizeof(create));
    create.mbfatr = TA_TFIFO;
    create.bufsz  = (SZ) (CMD_Q_DEPTH * MBF_SLOT_BYTES);       /* 32 B    */
    create.maxmsz = (INT) sizeof(uint32_t);
    g_cmd_q       = tk_cre_mbf(&create);

    create.bufsz  = (SZ) (DONE_Q_DEPTH * MBF_SLOT_BYTES);      /* 32 B    */
    create.maxmsz = (INT) sizeof(uint8_t);
    g_done_q      = tk_cre_mbf(&create);
}

/*!
 * @brief  Drive forward.
 * @param  speed_cms      Speed, cm/s (0..63).
 * @param  dist_cm        Distance, cm (1..511); 0 = until the next command.
 * @param  b_follow_line  True to steer on the line sensors.
 * @return Sequence number of the command.
 */
uint8_t
motion_control_forward (uint32_t speed_cms, uint32_t dist_cm,
                        bool b_follow_line)
{
    return (cmd_send(CMD_FWD, speed_cms, dist_cm, 0U, b_follow_line, false));
}

/*!
 * @brief  Drive backward. With b_follow_line the front sensors keep the
 *         car on the line it came along (Tribunal 1 reverse-and-scan).
 * @param  speed_cms      Speed, cm/s.
 * @param  dist_cm        Distance, cm (1..511).
 * @param  b_follow_line  True to steer on the line sensors.
 * @return Sequence number of the command.
 */
uint8_t
motion_control_reverse (uint32_t speed_cms, uint32_t dist_cm,
                        bool b_follow_line)
{
    return (cmd_send(CMD_BWD, speed_cms, dist_cm, 0U, b_follow_line, false));
}

/*!
 * @brief  Spin in place.
 * @param  b_left         True to turn left (counter-clockwise).
 * @param  speed_cms      Wheel speed, cm/s.
 * @param  angle_deg      Angle, degrees (0..511).
 * @param  b_end_on_line  True to stop early (after 60 %) on a new branch.
 * @return Sequence number of the command.
 */
uint8_t
motion_control_turn (bool b_left, uint32_t speed_cms, uint32_t angle_deg,
                     bool b_end_on_line)
{
    return (cmd_send(b_left ? CMD_TURN_L : CMD_TURN_R, speed_cms, 0U,
                     angle_deg, false, b_end_on_line));
}

/*!
 * @brief  U-turn: 180 degrees to the left, ending on the line.
 * @return Sequence number of the command.
 */
uint8_t
motion_control_uturn (void)
{
    return (cmd_send(CMD_UTURN, SPEED_TURN_CMS, 0U, UTURN_DEG, false, true));
}

/*!
 * @brief  Bench dead-band sweep: measures each wheel's dead band and
 *         feed-forward slope (wheels in the air).
 * @return Sequence number of the command.
 */
uint8_t
motion_control_deadband_test (void)
{
    /* CMD_MOTOR_TEST with the follow bit selects the sweep variant. */
    return (cmd_send(CMD_MOTOR_TEST, 0U, 0U, 0U, true, false));
}

/*!
 * @brief  Power-on encoder self-test (short open-loop push).
 * @return Sequence number of the command.
 */
uint8_t
motion_control_self_test (void)
{
    return (cmd_send(CMD_ENC_CAL, 0U, 0U, 0U, false, false));
}

/*!
 * @brief  Bench motor test: each wheel alone, both directions.
 * @return Sequence number of the command.
 */
uint8_t
motion_control_motor_test (void)
{
    return (cmd_send(CMD_MOTOR_TEST, 0U, 0U, 0U, false, false));
}

/*!
 * @brief  Stop and brake. Safe from any task; reports no completion.
 */
void
motion_control_stop (void)
{
    uint32_t const word = (uint32_t) CMD_STOP;   /* sequence 0          */

    (void) tk_snd_mbf(g_cmd_q, &word, (INT) sizeof(word), TMO_POL);
}

/*!
 * @brief  Take one completion report, if any.
 * @param  p_done  Output report.
 * @return True if a report was taken.
 */
bool
motion_control_done_take (motion_done_t * p_done)
{
    uint8_t    msg   = 0U;
    bool const b_got = (tk_rcv_mbf(g_done_q, &msg, TMO_POL) > 0);

    if (b_got)
    {
        uint32_t const result = ((uint32_t) msg >> DONE_RESULT_SHIFT) &
                                DONE_RESULT_MASK;

        /* Masked to 3 and 2 bits; result <= 2 by construction. */
        p_done->seq    = (uint8_t) ((uint32_t) msg & DONE_SEQ_MASK);
        p_done->result = (motion_result_t) result;
    }

    return (b_got);
}

/*!
 * @brief  Report the degraded mode.
 * @return True if an encoder failed and the task drives open loop.
 */
bool
motion_control_is_open_loop (void)
{
    return (g_b_open_loop);
}

/*!
 * @brief  Encoder direction signs (+1 / -1 per wheel: raw count change
 *         per forward count), as found by the power-on self-test.
 *
 *  NOTE: Written once by this task during the self-test; each sign is an
 *  aligned 32-bit word, so a reader in another task sees the old or the
 *  new value, never a mix.
 *
 * @param  p_sign_l  Output, left wheel sign.
 * @param  p_sign_r  Output, right wheel sign.
 */
void
motion_control_encoder_signs (int32_t * p_sign_l, int32_t * p_sign_r)
{
    *p_sign_l = g_sign_l;
    *p_sign_r = g_sign_r;
}

/*!
 * @brief  Motion Control task body (priority PRI_MOTION, 10 ms).
 * @param  stacd    Start code (unused).
 * @param  p_exinf  Extended information (unused).
 */
void
motion_control_task (INT stacd, void * p_exinf)
{
    uint64_t    next_us = time_us_64();
    motion_in_t in;

    (void) stacd;
    (void) p_exinf;

    (void) memset(&in, 0, sizeof(in));
    g_ctx.prev_cl = motor_driver_count_left();
    g_ctx.prev_cr = motor_driver_count_right();

    for (;;)
    {
        bool b_done = false;

        systime_period_wait(&next_us, PERIOD_MOTION_MS);
        safety_heartbeat(SAFETY_HB_MOTION);

        inputs_read(&in);
        speed_update(&in);

        /* The safety latch overrides everything: Safety has braked and
         * inhibited the motors; queued commands are discarded. */
        if (safety_is_latched())
        {
            g_ctx.mode = MS_SAFE_STOP;
        }
        else if (MS_SAFE_STOP == g_ctx.mode)
        {
            g_ctx.mode = MS_STOPPED;              /* fault cleared         */
        }
        else
        {
            /* Normal operation. */
        }

        commands_take(&in);
        b_done = targets_set(&in);
        outputs_drive(&in, b_done);
        counts_print(&in);
        state_publish();
    }
}

/*!
 * @brief  Pack and queue a command.
 * @return Its sequence number (1..7).
 */
static uint8_t
cmd_send (cmd_id_t id, uint32_t speed_cms, uint32_t dist_cm,
          uint32_t angle_deg, bool b_follow, bool b_end)
{
    uint8_t const seq  = g_next_seq;
    uint32_t      word = (((uint32_t) id & CMD_ID_MASK) << CMD_ID_SHIFT) |
                         (((uint32_t) seq & CMD_SEQ_MASK) << CMD_SEQ_SHIFT);

    /* Sequence 1..7; 0 is reserved for STOP. */
    g_next_seq = (uint8_t) ((seq >= CMD_SEQ_MASK) ? 1U : (seq + 1U));

    word |= b_follow ? CMD_FOLLOW_BIT : 0U;
    word |= b_end ? CMD_END_LINE_BIT : 0U;
    word |= ((speed_cms > CMD_SPEED_MASK) ? CMD_SPEED_MASK : speed_cms)
            << CMD_SPEED_SHIFT;
    word |= ((dist_cm > CMD_DIST_MASK) ? CMD_DIST_MASK : dist_cm)
            << CMD_DIST_SHIFT;
    word |= ((angle_deg > CMD_ANGLE_MASK) ? CMD_ANGLE_MASK : angle_deg)
            << CMD_ANGLE_SHIFT;

    (void) tk_snd_mbf(g_cmd_q, &word, (INT) sizeof(word), TMO_POL);

    return (seq);
}

/*!
 * @brief  Sample encoders, line sensors and IMU heading.
 * @param  p_in  Output inputs of this period.
 */
static void
inputs_read (motion_in_t * p_in)
{
    vehicle_state_t state;

    vehicle_state_read(&state);

    p_in->now_ms       = systime_now_ms();
    p_in->count_l           = motor_driver_count_left();
    p_in->count_r           = motor_driver_count_right();
    p_in->b_line_fresh = sensor_line_read(&p_in->line_bits);
    p_in->heading_deg  = (float32_t) state.heading_half_deg / HALF_DEG;
    p_in->b_imu_fresh  = state.b_imu_valid &&
                         (systime_stamp_age(state.imu_stamp) <=
                          IMU_MAX_AGE_MS);
}

/*!
 * @brief  Wheel speed and travel from the encoders, or from the last
 *         targets when driving open loop. Travel uses every count; the
 *         speed fed to the PID is the average of the last
 *         SPEED_AVG_PERIODS periods, which removes most of the +/-1 count
 *         quantisation noise.
 * @param  p_in  Inputs of this period.
 */
static void
speed_update (motion_in_t const * p_in)
{
    /* Count deltas per period are small, so the floats are exact. */
    int32_t const delta_l = g_sign_l * (p_in->count_l - g_ctx.prev_cl);
    int32_t const delta_r = g_sign_r * (p_in->count_r - g_ctx.prev_cr);
    int32_t       sum_l   = 0;
    int32_t       sum_r   = 0;
    uint32_t      idx     = 0U;

    g_ctx.prev_cl = p_in->count_l;
    g_ctx.prev_cr = p_in->count_r;

    if (g_b_open_loop)
    {
        g_ctx.speed_l = g_ctx.cmd_l;
        g_ctx.speed_r = g_ctx.cmd_r;
        g_ctx.dist_l += g_ctx.speed_l * DT_S;
        g_ctx.dist_r += g_ctx.speed_r * DT_S;
    }
    else
    {
        g_ctx.hist_l[g_ctx.hist_idx] = delta_l;
        g_ctx.hist_r[g_ctx.hist_idx] = delta_r;
        g_ctx.hist_idx = (g_ctx.hist_idx + 1U) % SPEED_AVG_PERIODS;

        for (idx = 0U; idx < SPEED_AVG_PERIODS; idx++)
        {
            sum_l += g_ctx.hist_l[idx];
            sum_r += g_ctx.hist_r[idx];
        }

        g_ctx.speed_l = ((float32_t) sum_l * M_PER_COUNT) /
                        (DT_S * (float32_t) SPEED_AVG_PERIODS);
        g_ctx.speed_r = ((float32_t) sum_r * M_PER_COUNT) /
                        (DT_S * (float32_t) SPEED_AVG_PERIODS);
        g_ctx.dist_l += (float32_t) delta_l * M_PER_COUNT;
        g_ctx.dist_r += (float32_t) delta_r * M_PER_COUNT;
    }
}

/*!
 * @brief  Take queued commands; the latest one wins. Ignored while the
 *         safety latch is set, and in FAULT except STOP.
 * @param  p_in  Inputs of this period.
 */
static void
commands_take (motion_in_t const * p_in)
{
    uint32_t word = 0U;

    while (tk_rcv_mbf(g_cmd_q, &word, TMO_POL) > 0)
    {
        cmd_id_t const id = (cmd_id_t) ((word >> CMD_ID_SHIFT) & CMD_ID_MASK);

        if ((MS_SAFE_STOP != g_ctx.mode) &&
            ((MS_FAULT != g_ctx.mode) || (CMD_STOP == id)))
        {
            command_start(word, p_in);
        }
    }
}

/*!
 * @brief  Unpack a command and enter its state.
 * @param  word  Packed command.
 * @param  p_in  Inputs of this period.
 */
static void
command_start (uint32_t word, motion_in_t const * p_in)
{
    motion_mode_t const old = g_ctx.mode;
    float32_t           arc = 0.0f;

    /* Each field is masked to its width, so the casts keep the value. */
    g_ctx.id            = (cmd_id_t) ((word >> CMD_ID_SHIFT) & CMD_ID_MASK);
    g_ctx.seq           = (uint8_t) ((word >> CMD_SEQ_SHIFT) & CMD_SEQ_MASK);
    g_ctx.b_follow      = (0U != (word & CMD_FOLLOW_BIT));
    g_ctx.b_end_on_line = (0U != (word & CMD_END_LINE_BIT));
    g_ctx.speed_mps     = (float32_t) ((word >> CMD_SPEED_SHIFT) &
                                       CMD_SPEED_MASK) / CM_PER_M;
    g_ctx.dist_m        = (float32_t) ((word >> CMD_DIST_SHIFT) &
                                       CMD_DIST_MASK) / CM_PER_M;
    g_ctx.angle_deg     = (float32_t) ((word >> CMD_ANGLE_SHIFT) &
                                       CMD_ANGLE_MASK);
    g_ctx.dist_l0           = g_ctx.dist_l;
    g_ctx.dist_r0           = g_ctx.dist_r;
    g_ctx.hdg0          = p_in->heading_deg;
    g_ctx.b_hdg0        = p_in->b_imu_fresh;
    g_ctx.start_ms      = p_in->now_ms;
    g_ctx.cal_l0        = p_in->count_l;
    g_ctx.cal_r0        = p_in->count_r;
    g_ctx.test_phase    = 0U;
    g_ctx.b_steered     = false;
    g_ctx.line_last     = 0;
    g_ctx.sweep_stage   = SWEEP_SETTLE;
    g_ctx.stage_ms      = p_in->now_ms;
    pid_reset(&g_ctx.pid_l);
    pid_reset(&g_ctx.pid_r);

    switch (g_ctx.id)
    {
        case CMD_FWD:        g_ctx.mode = MS_STRAIGHT;   break;
        case CMD_BWD:        g_ctx.mode = MS_REVERSE;    break;
        case CMD_TURN_L:     g_ctx.mode = MS_TURN_L;     break;
        case CMD_TURN_R:     g_ctx.mode = MS_TURN_R;     break;
        case CMD_UTURN:      g_ctx.mode = MS_UTURN;      break;
        case CMD_ENC_CAL:    g_ctx.mode = MS_ENC_CAL;    break;
        case CMD_MOTOR_TEST:
            g_ctx.mode = g_ctx.b_follow ? MS_DEADBAND : MS_MOTOR_TEST;
        break;
        case CMD_STOP:
        default:             g_ctx.mode = MS_STOPPED;    break;
    }

    /* A new kind of movement starts its ramp from rest; a speed change
     * of the same movement ramps from the current speed. */
    if (g_ctx.mode != old)
    {
        g_ctx.v_base = 0.0f;
    }

    /* Generous time limit: 2x nominal + 2 s; none for open-ended runs. */
    g_ctx.limit_ms = 0U;
    if (((MS_STRAIGHT == g_ctx.mode) || (MS_REVERSE == g_ctx.mode)) &&
        (g_ctx.dist_m > 0.0f))
    {
        g_ctx.limit_ms = (uint32_t) ((LIMIT_FACTOR * g_ctx.dist_m) /
                         fmaxf(g_ctx.speed_mps, MIN_LIMIT_SPEED_MPS)) +
                         LIMIT_EXTRA_MS;
    }
    else if ((MS_TURN_L == g_ctx.mode) || (MS_TURN_R == g_ctx.mode) ||
             (MS_UTURN == g_ctx.mode))
    {
        arc = (g_ctx.angle_deg / DEG_PER_RAD) * (TRACK_WIDTH_M * HALF);
        g_ctx.limit_ms = (uint32_t) ((LIMIT_FACTOR * arc) /
                         fmaxf(g_ctx.speed_mps, MIN_LIMIT_SPEED_MPS)) +
                         LIMIT_EXTRA_MS;
    }
    else
    {
        /* Stop, self-test and motor test have their own timing. */
    }
}

/*!
 * @brief  Wheel targets for the current state.
 * @param  p_in  Inputs of this period.
 * @return True when the command has reached its target.
 */
static bool
targets_set (motion_in_t const * p_in)
{
    bool b_done = false;

    switch (g_ctx.mode)
    {
        case MS_STRAIGHT:
            b_done = straight_targets(p_in, 1.0f);
        break;

        case MS_REVERSE:
            b_done = straight_targets(p_in, -1.0f);
        break;

        case MS_TURN_L:
        case MS_TURN_R:
        case MS_UTURN:
            b_done = turn_targets(p_in);
        break;

        default:
            g_ctx.v_base = 0.0f;
            g_ctx.tgt_l  = 0.0f;
            g_ctx.tgt_r  = 0.0f;
        break;
    }

    return (b_done);
}

/*!
 * @brief  Straight run, forward (sign +1) or backward (sign -1).
 *
 *  On the line: steer toward the sensor that sees black and slow to
 *  SPEED_STEER_CMS while correcting. The sensors sit at the front, so the
 *  same differential pulls the front toward the line in both directions.
 *  Off the line: equalise wheel distance and hold the IMU heading.
 *
 * @param  p_in  Inputs of this period.
 * @param  sign  +1 forward, -1 backward.
 * @return True when the commanded distance is covered.
 */
static bool
straight_targets (motion_in_t const * p_in, float32_t sign)
{
    float32_t const travel = ((g_ctx.dist_l - g_ctx.dist_l0) +
                              (g_ctx.dist_r - g_ctx.dist_r0)) * HALF;
    float32_t       speed  = 0.0f;
    float32_t       steer  = 0.0f;
    float32_t       target = g_ctx.speed_mps;

    if (g_ctx.dist_m > 0.0f)
    {
        target = fminf(target, end_speed_cap(g_ctx.dist_m - (sign * travel)));
    }

    if (g_ctx.b_follow && p_in->b_line_fresh)
    {
        steer = line_steer(p_in);
    }
    else
    {
        float32_t const d_err = (g_ctx.dist_l - g_ctx.dist_l0) -
                                (g_ctx.dist_r - g_ctx.dist_r0); /* + right */
        float32_t const h_err = (g_ctx.b_hdg0 && p_in->b_imu_fresh) ?
                                wrap180(p_in->heading_deg - g_ctx.hdg0) :
                                0.0f;

        /* Wheel speed differential turns the car the same way in both
         * directions, so the correction does not change sign: the
         * wheel that has travelled further is slowed either way. */
        steer = -((K_DIST_EQ * d_err) + (K_HEADING * h_err));
    }

    /* Through a curve: slow to steering speed at once (the wheel slew
     * smooths the drop) and only ramp back up once it is over. */
    if (g_ctx.b_follow && g_ctx.b_steered &&
        ((p_in->now_ms - g_ctx.steer_ms) < LINE_SLOW_HOLD_MS))
    {
        target       = fminf(target, (float32_t) SPEED_STEER_CMS / CM_PER_M);
        g_ctx.v_base = fminf(g_ctx.v_base, target);
    }

    ramp_base(target);
    speed = g_ctx.v_base;

    g_ctx.tgt_l = (sign * speed) + steer;
    g_ctx.tgt_r = (sign * speed) - steer;

    return ((g_ctx.dist_m > 0.0f) && ((sign * travel) >= g_ctx.dist_m));
}

/*!
 * @brief  Steering differential from the line sensors, which straddle
 *         the line: one sensor on black turns toward it; both white keeps
 *         turning gently (LINE_LAST_GAIN) toward the side that last saw
 *         black (both white cannot
 *         tell centred from a line that slipped out past the inner sensor
 *         on a curve); both black (junction) drives straight.
 * @param  p_in  Inputs of this period (fresh line reading).
 * @return Steering differential, m/s (+ turns right).
 */
static float32_t
line_steer (motion_in_t const * p_in)
{
    int32_t   pos  = 0;                           /* -1 left, +1 right    */
    float32_t gain = 1.0f;

    if (VEHICLE_IR_LEFT == p_in->line_bits)
    {
        pos = -1;
    }
    else if (VEHICLE_IR_RIGHT == p_in->line_bits)
    {
        pos = 1;
    }
    else if (0U == p_in->line_bits)
    {
        pos  = g_ctx.line_last;                   /* keep turning, gently */
        gain = LINE_LAST_GAIN;
    }
    else
    {
        g_ctx.line_last = 0;                      /* junction: straight   */
    }

    if ((VEHICLE_IR_LEFT == p_in->line_bits) ||
        (VEHICLE_IR_RIGHT == p_in->line_bits))
    {
        g_ctx.line_last = pos;
        g_ctx.b_steered = true;                   /* hold steering speed  */
        g_ctx.steer_ms  = p_in->now_ms;
    }

    /* pos and LINE_STEER_DIR are +/-1, so the product is exact. */
    return ((float32_t) (LINE_STEER_DIR * pos) * K_LINE_STEER_MPS * gain);
}

/*!
 * @brief  Spin in place until the arc is covered, or (with end-on-line)
 *         a new branch appears after TURN_EARLY_FRACTION of it. On
 *         completion the turn is cross-checked against the IMU heading.
 * @param  p_in  Inputs of this period.
 * @return True when the turn is complete.
 */
static bool
turn_targets (motion_in_t const * p_in)
{
    float32_t const arc    = (g_ctx.angle_deg / DEG_PER_RAD) *
                             (TRACK_WIDTH_M * HALF);
    /* Rotation only: forward motion while the wheels slew from a
     * straight run into the spin cancels out of the difference. */
    float32_t const travel = fabsf((g_ctx.dist_r - g_ctx.dist_r0) -
                                   (g_ctx.dist_l - g_ctx.dist_l0)) * HALF;
    float32_t const frac   = (arc > 0.0f) ? (travel / arc) : 1.0f;
    bool const      b_left = (MS_TURN_R != g_ctx.mode);  /* U-turn: left */
    bool            b_done = false;

    ramp_base(fminf(g_ctx.speed_mps, end_speed_cap(arc - travel)));
    g_ctx.tgt_l = b_left ? -g_ctx.v_base : g_ctx.v_base;
    g_ctx.tgt_r = b_left ? g_ctx.v_base : -g_ctx.v_base;

    if (frac >= 1.0f)
    {
        b_done = true;
    }
    else if (g_ctx.b_end_on_line && (frac >= TURN_EARLY_FRACTION) &&
             p_in->b_line_fresh && (0U != p_in->line_bits))
    {
        b_done = true;                            /* new branch found     */
    }
    else
    {
        /* Keep turning. */
    }

    if (b_done && g_ctx.b_hdg0 && p_in->b_imu_fresh)
    {
        float32_t const imu_turn = fabsf(wrap180(p_in->heading_deg -
                                                 g_ctx.hdg0));
        float32_t const enc_turn = frac * g_ctx.angle_deg;

        if (fabsf(imu_turn - enc_turn) > TURN_XCHECK_DEG)
        {
            telemetry_post_event(EV_TURN_XCHECK, 0U,
                                 (int16_t) round_i(enc_turn),
                                 (int16_t) round_i(imu_turn));
        }
    }

    return (b_done);
}

/*!
 * @brief  Move the base speed toward a target by at most the ramp step.
 * @param  target  Target speed magnitude, m/s.
 */
static void
ramp_base (float32_t target)
{
    float32_t const step = ACCEL_LIMIT_MPS2 * DT_S;

    g_ctx.v_base += clamp_f(target - g_ctx.v_base, -step, step);
}

/*!
 * @brief  Highest speed from which the car can still slow to
 *         END_MIN_SPEED_MPS at ACCEL_LIMIT_MPS2 within the remaining
 *         distance (v^2 = 2 a d), so runs and turns end softly.
 * @param  remaining_m  Distance or arc left, m.
 * @return Speed cap, m/s (never below END_MIN_SPEED_MPS).
 */
static float32_t
end_speed_cap (float32_t remaining_m)
{
    float32_t cap = END_MIN_SPEED_MPS;

    if (remaining_m > 0.0f)
    {
        cap = fmaxf(sqrtf(TWO * ACCEL_LIMIT_MPS2 * remaining_m),
                    END_MIN_SPEED_MPS);
    }

    return (cap);
}

/*!
 * @brief  Move each wheel's commanded speed toward its target by at most
 *         WHEEL_ACCEL_MPS2 per second.
 */
static void
wheels_slew (void)
{
    float32_t const step = WHEEL_ACCEL_MPS2 * DT_S;

    g_ctx.cmd_l += clamp_f(g_ctx.tgt_l - g_ctx.cmd_l, -step, step);
    g_ctx.cmd_r += clamp_f(g_ctx.tgt_r - g_ctx.cmd_r, -step, step);
}

/*!
 * @brief  Apply the outputs of the current state.
 * @param  p_in    Inputs of this period.
 * @param  b_done  True if the command reached its target this period.
 */
static void
outputs_drive (motion_in_t const * p_in, bool b_done)
{
    if (MS_MOTOR_TEST == g_ctx.mode)
    {
        motor_test_step(p_in);
    }
    else if (MS_DEADBAND == g_ctx.mode)
    {
        deadband_step(p_in);
    }
    else if (MS_ENC_CAL == g_ctx.mode)
    {
        self_test_step(p_in);
    }
    else if ((MS_STOPPED == g_ctx.mode) || (MS_SAFE_STOP == g_ctx.mode) ||
             (MS_FAULT == g_ctx.mode))
    {
        /* In SAFE_STOP Safety owns the outputs (braked and inhibited). */
        g_ctx.duty_l = 0.0f;
        g_ctx.duty_r = 0.0f;
        motor_driver_brake();
    }
    else if (b_done)
    {
        brake_to(MS_STOPPED);
        done_post(MOTION_RESULT_DONE);
    }
    else
    {
        closed_loop_drive(p_in);
    }
}

/*!
 * @brief  Bench motor test, open loop, wheels off the ground: phase 0
 *         left +duty, 1 left -duty, 2 right +duty, 3 right -duty. A
 *         negative duty drives the other input of the channel, so the
 *         four results show the driver's real truth table.
 * @param  p_in  Inputs of this period.
 */
static void
motor_test_step (motion_in_t const * p_in)
{
    uint32_t const elapsed = p_in->now_ms - g_ctx.start_ms;

    if (elapsed >= MOTOR_TEST_MS)
    {
        int32_t const moved_l = p_in->count_l - g_ctx.cal_l0;
        int32_t const moved_r = p_in->count_r - g_ctx.cal_r0;

        /* Phase 0..3 -> code 1..4. Counts over one second are well under
         * 32767 (the packer sends them divided by 4). */
        telemetry_post_event(EV_MOTOR_TEST,
                             (uint8_t) (g_ctx.test_phase + 1U),
                             (int16_t) moved_l, (int16_t) moved_r);
        motor_driver_brake();
        g_ctx.cal_l0   = p_in->count_l;
        g_ctx.cal_r0   = p_in->count_r;
        g_ctx.start_ms = p_in->now_ms;
        g_ctx.test_phase++;

        if (g_ctx.test_phase >= TEST_PHASES)
        {
            brake_to(MS_STOPPED);
            done_post(MOTION_RESULT_DONE);
        }
    }
    else if (elapsed < MOTOR_TEST_SETTLE_MS)
    {
        motor_driver_brake();                     /* let the wheel stop   */
    }
    else
    {
        int32_t const duty = (0U == (g_ctx.test_phase & 1U)) ?
                             MOTOR_TEST_DUTY_PM : -MOTOR_TEST_DUTY_PM;
        bool const    b_left_phase = (g_ctx.test_phase < 2U);

        motor_driver_set(b_left_phase ? duty : 0, b_left_phase ? 0 : duty);
    }
}

/*!
 * @brief  Dead-band sweep, one wheel at a time (left, then right), open
 *         loop, wheels in the air: settle, raise the duty step by step
 *         until the wheel moves (dead band), then hold SWEEP_HOLD_PM,
 *         spin up and measure the speed (slope).
 * @param  p_in  Inputs of this period.
 */
static void
deadband_step (motion_in_t const * p_in)
{
    uint32_t const elapsed = p_in->now_ms - g_ctx.stage_ms;
    int32_t const  count   = (0U == g_ctx.test_phase) ? p_in->count_l :
                                                        p_in->count_r;
    int32_t const  moved   = abs(count - g_ctx.stage_count0);

    switch (g_ctx.sweep_stage)
    {
        case SWEEP_SETTLE:
            motor_driver_brake();
            if (elapsed >= MOTOR_TEST_SETTLE_MS)
            {
                g_ctx.sweep_stage  = SWEEP_RAMP;
                g_ctx.sweep_pm     = SWEEP_START_PM;
                g_ctx.stage_ms     = p_in->now_ms;
                g_ctx.stage_count0 = count;
            }
        break;

        case SWEEP_RAMP:
            sweep_drive(g_ctx.sweep_pm);
            if ((elapsed >= SWEEP_STEP_MS) && (moved >= SWEEP_MOVE_COUNTS))
            {
                g_ctx.sweep_db_pm = g_ctx.sweep_pm;
                g_ctx.sweep_stage = SWEEP_SPINUP;
                g_ctx.stage_ms    = p_in->now_ms;
            }
            else if (elapsed >= SWEEP_STEP_MS)
            {
                g_ctx.sweep_pm    += SWEEP_STEP_PM;
                g_ctx.stage_ms     = p_in->now_ms;
                g_ctx.stage_count0 = count;

                if (g_ctx.sweep_pm > SWEEP_MAX_PM)
                {
                    sweep_wheel_done(p_in, SWEEP_FAILED_PCT, 0);
                }
            }
            else
            {
                /* Within the step. */
            }
        break;

        case SWEEP_SPINUP:
            sweep_drive(SWEEP_HOLD_PM);
            if (elapsed >= SWEEP_SPINUP_MS)
            {
                g_ctx.sweep_stage  = SWEEP_MEASURE;
                g_ctx.stage_ms     = p_in->now_ms;
                g_ctx.stage_count0 = count;
            }
        break;

        case SWEEP_MEASURE:
        default:
            sweep_drive(SWEEP_HOLD_PM);
            if (elapsed >= SWEEP_MEASURE_MS)
            {
                /* Counts in SWEEP_MEASURE_MS are small; floats are exact. */
                float32_t const speed = ((float32_t) moved * M_PER_COUNT *
                                         MS_PER_S) /
                                        (float32_t) SWEEP_MEASURE_MS;
                float32_t const span  = (float32_t) (SWEEP_HOLD_PM -
                                                     g_ctx.sweep_db_pm) /
                                        PM_PER_UNIT;
                float32_t const kff   = (speed > 0.0f) ? (span / speed) :
                                                         0.0f;

                sweep_wheel_done(p_in, g_ctx.sweep_db_pm / PCT_PER_PM,
                                 round_i(kff * KFF_X100));
            }
        break;
    }
}

/*!
 * @brief  Report one wheel's sweep and move on to the next wheel (or
 *         finish).
 * @param  p_in      Inputs of this period.
 * @param  db_pct    Dead band, percent (SWEEP_FAILED_PCT = never moved).
 * @param  kff_x100  Feed-forward slope x100 (duty per m/s).
 */
static void
sweep_wheel_done (motion_in_t const * p_in, int32_t db_pct,
                  int32_t kff_x100)
{
    motor_driver_brake();
    tm_printf((UB *) "sweep %s: dead band %d %%, KFF %d (x0.01 duty/(m/s))\n",
              (0U == g_ctx.test_phase) ? "LEFT" : "RIGHT", (int) db_pct,
              (int) kff_x100);
    /* Wheel 0 / 1; dead band <= 100 and slope x100 fit an int16_t. */
    telemetry_post_event(EV_DEADBAND, (uint8_t) g_ctx.test_phase,
                         (int16_t) db_pct, (int16_t) kff_x100);

    g_ctx.test_phase++;
    g_ctx.sweep_stage = SWEEP_SETTLE;
    g_ctx.stage_ms    = p_in->now_ms;

    if (g_ctx.test_phase >= SWEEP_WHEELS)
    {
        brake_to(MS_STOPPED);
        done_post(MOTION_RESULT_DONE);
    }
}

/*!
 * @brief  Drive only the wheel under test, forward.
 * @param  duty_pm  Duty, per mille.
 */
static void
sweep_drive (int32_t duty_pm)
{
    if (0U == g_ctx.test_phase)
    {
        motor_driver_set(duty_pm, 0);
    }
    else
    {
        motor_driver_set(0, duty_pm);
    }
}

/*!
 * @brief  Power-on encoder self-test. Equal open-loop forward drive on
 *         both wheels: the sign of each raw count change gives that
 *         encoder's forward direction. A silent encoder switches the task
 *         to open loop (degraded, reported as FC_ENCODER); a car that
 *         rotates has a motor wired backwards (fault).
 * @param  p_in  Inputs of this period.
 */
static void
self_test_step (motion_in_t const * p_in)
{
    int32_t const raw_l  = p_in->count_l - g_ctx.cal_l0;
    int32_t const raw_r  = p_in->count_r - g_ctx.cal_r0;
    bool const    b_ok_l = (raw_l >= ENC_CAL_COUNTS) ||
                           (raw_l <= -ENC_CAL_COUNTS);
    bool const    b_ok_r = (raw_r >= ENC_CAL_COUNTS) ||
                           (raw_r <= -ENC_CAL_COUNTS);
    float32_t     spin   = 0.0f;

    if (g_ctx.b_hdg0 && p_in->b_imu_fresh)
    {
        spin = fabsf(wrap180(p_in->heading_deg - g_ctx.hdg0));
    }

    if (spin > ENC_CAL_SPIN_DEG)
    {
        brake_to(MS_FAULT);
        telemetry_post_event(EV_ENC_CAL, ENC_CAL_ROTATED,
                             (int16_t) round_i(spin), 0);
        done_post(MOTION_RESULT_FAULT);
    }
    else if (b_ok_l && b_ok_r)
    {
        g_sign_l      = (raw_l > 0) ? 1 : -1;
        g_sign_r      = (raw_r > 0) ? 1 : -1;
        g_b_open_loop = false;
        brake_to(MS_STOPPED);
        telemetry_post_event(EV_ENC_CAL, ENC_CAL_OK, (int16_t) g_sign_l,
                             (int16_t) g_sign_r);
        done_post(MOTION_RESULT_DONE);
    }
    else if ((p_in->now_ms - g_ctx.start_ms) > ENC_CAL_TIMEOUT_MS)
    {
        uint32_t const silent = (b_ok_l ? 0U : ENC_CAL_LEFT_SILENT) |
                                (b_ok_r ? 0U : ENC_CAL_RIGHT_SILENT);

        g_b_open_loop = true;                     /* degraded mode        */
        brake_to(MS_STOPPED);
        /* silent <= 3; counts below ENC_CAL_COUNTS fit an int16_t. */
        telemetry_post_event(EV_ENC_CAL, (uint8_t) silent, (int16_t) raw_l,
                             (int16_t) raw_r);
        done_post(MOTION_RESULT_DONE);
    }
    else
    {
        g_ctx.duty_l = (float32_t) ENC_CAL_DUTY_PM / PM_PER_UNIT;
        g_ctx.duty_r = g_ctx.duty_l;
        motor_driver_set(ENC_CAL_DUTY_PM, ENC_CAL_DUTY_PM);
    }
}

/*!
 * @brief  PID (or open-loop feed-forward) drive with stall and time-limit
 *         checks.
 * @param  p_in  Inputs of this period.
 */
static void
closed_loop_drive (motion_in_t const * p_in)
{
    uint32_t const now      = p_in->now_ms;
    uint32_t       edge_l = motor_driver_edge_ms_left();
    uint32_t       edge_r = motor_driver_edge_ms_right();
    uint32_t       why    = 0U;

    wheels_slew();

    if (g_b_open_loop)
    {
        g_ctx.duty_l = clamp_f(feed_forward(g_ctx.cmd_l, &g_ff_l),
                               -1.0f, 1.0f);
        g_ctx.duty_r = clamp_f(feed_forward(g_ctx.cmd_r, &g_ff_r),
                               -1.0f, 1.0f);
    }
    else
    {
        g_ctx.duty_l = clamp_f(pid_step(&g_ctx.pid_l, g_ctx.cmd_l,
                                        g_ctx.speed_l, &g_ff_l),
                               -1.0f, 1.0f);
        g_ctx.duty_r = clamp_f(pid_step(&g_ctx.pid_r, g_ctx.cmd_r,
                                        g_ctx.speed_r, &g_ff_r),
                               -1.0f, 1.0f);

        edge_l = edge_clamp(edge_l, now);
        edge_r = edge_clamp(edge_r, now);

        if ((fabsf(g_ctx.duty_l) > STALL_MIN_DUTY) &&
            ((now - edge_l) > STALL_TIMEOUT_MS))
        {
            why |= WHY_LEFT_SILENT;
        }

        if ((fabsf(g_ctx.duty_r) > STALL_MIN_DUTY) &&
            ((now - edge_r) > STALL_TIMEOUT_MS))
        {
            why |= WHY_RIGHT_SILENT;
        }
    }

    if ((0U != g_ctx.limit_ms) && ((now - g_ctx.start_ms) > g_ctx.limit_ms))
    {
        why |= WHY_TIME_LIMIT;
    }

    if (0U != why)
    {
        motion_mode_t const mode = g_ctx.mode;

        brake_to(MS_FAULT);
        /* why <= 7 and mode <= 5: both fit an int16_t. */
        telemetry_post_event(EV_FAULT, (uint8_t) FC_STALL, (int16_t) why,
                             (int16_t) mode);
        done_post((WHY_TIME_LIMIT == why) ? MOTION_RESULT_TIMEOUT :
                                            MOTION_RESULT_FAULT);
    }
    else
    {
        motor_driver_set(round_i(g_ctx.duty_l * PM_PER_UNIT),
                         round_i(g_ctx.duty_r * PM_PER_UNIT));
    }
}

/*!
 * @brief  Bound an encoder edge time for the stall check.
 *
 *  An edge before the command started counts from the start, so a wheel
 *  that never moves stalls STALL_TIMEOUT_MS after the start. The ISR can
 *  stamp an edge just after t was sampled; that edge counts as now, so
 *  (t - edge) cannot wrap around.
 *
 * @param  edge_ms  Time of the last edge.
 * @param  t        Time of this period.
 * @return Edge time limited to start..t.
 */
static uint32_t
edge_clamp (uint32_t edge_ms, uint32_t now)
{
    uint32_t result = edge_ms;

    /* Signed views of modular differences: negative = "before". */
    if ((int32_t) (edge_ms - g_ctx.start_ms) < 0)
    {
        result = g_ctx.start_ms;
    }
    else if ((int32_t) (now - edge_ms) < 0)
    {
        result = now;
    }
    else
    {
        /* Within the command. */
    }

    return (result);
}

/*!
 * @brief  Brake and enter a resting state.
 * @param  mode  MS_STOPPED or MS_FAULT.
 */
static void
brake_to (motion_mode_t mode)
{
    motor_driver_brake();
    g_ctx.duty_l = 0.0f;
    g_ctx.duty_r = 0.0f;
    g_ctx.tgt_l  = 0.0f;
    g_ctx.tgt_r  = 0.0f;
    g_ctx.cmd_l  = 0.0f;
    g_ctx.cmd_r  = 0.0f;
    g_ctx.v_base = 0.0f;
    g_ctx.mode   = mode;
}

/*!
 * @brief  Report completion of the current command (1 byte).
 * @param  result  Outcome.
 */
static void
done_post (motion_result_t result)
{
    /* seq <= 7 and result <= 2, so the packed value fits a byte. */
    uint8_t const msg = (uint8_t) (((uint32_t) g_ctx.seq & DONE_SEQ_MASK) |
                                   (((uint32_t) result & DONE_RESULT_MASK)
                                    << DONE_RESULT_SHIFT));

    (void) tk_snd_mbf(g_done_q, &msg, (INT) sizeof(msg), TMO_POL);
}

/*!
 * @brief  One PID step on top of the feed-forward, with a clamped
 *         integrator that stops integrating while the output is
 *         saturated in the direction of the error (anti-windup).
 * @param  p_pid     Controller state.
 * @param  target    Target speed, m/s.
 * @param  measured  Measured (averaged) speed, m/s.
 * @param  p_ff      That wheel's open-loop model.
 * @return Duty, -1..1 before clamping.
 */
static float32_t
pid_step (wheel_pid_t * p_pid, float32_t target, float32_t measured,
          wheel_ff_t const * p_ff)
{
    float32_t const err     = target - measured;
    float32_t const der     = (err - p_pid->prev_err) / DT_S;
    float32_t const base    = feed_forward(target, p_ff) + (PID_KP * err) +
                              (PID_KD * der);
    float32_t const integ   = clamp_f(p_pid->integ + (err * DT_S),
                                      -PID_I_LIMIT, PID_I_LIMIT);
    float32_t const out     = base + (PID_KI * integ);
    bool const      b_wound = ((out > 1.0f) && (err > 0.0f)) ||
                              ((out < -1.0f) && (err < 0.0f));

    if (!b_wound)
    {
        p_pid->integ = integ;
    }

    p_pid->prev_err = err;

    return (base + (PID_KI * p_pid->integ));
}

/*!
 * @brief  Open-loop duty for a target speed: the motor dead band plus a
 *         slope from the measured duty-to-speed ratio. A target below
 *         FF_MIN_SPEED_MPS means stop (no dead-band kick).
 * @param  target  Target wheel speed, m/s (sign = direction).
 * @param  p_ff    That wheel's open-loop model.
 * @return Duty, -1..1 before clamping.
 */
static float32_t
feed_forward (float32_t target, wheel_ff_t const * p_ff)
{
    float32_t const magnitude = fabsf(target);
    float32_t       duty      = 0.0f;

    if (magnitude >= FF_MIN_SPEED_MPS)
    {
        duty = p_ff->dead_band + (p_ff->slope * magnitude);
        duty = (target < 0.0f) ? -duty : duty;
    }

    return (duty);
}

/*!
 * @brief  While stopped, print the raw encoder counts on the USB console
 *         when they change (at most every COUNT_PRINT_MS). Turn a wheel
 *         one full turn by hand and the change is ENC_COUNTS_PER_REV.
 * @param  p_in  Inputs of this period.
 */
static void
counts_print (motion_in_t const * p_in)
{
    if ((MS_STOPPED == g_ctx.mode) &&
        ((p_in->now_ms - g_ctx.printed_ms) >= COUNT_PRINT_MS) &&
        ((p_in->count_l != g_ctx.printed_cl) ||
         (p_in->count_r != g_ctx.printed_cr)))
    {
        g_ctx.printed_ms = p_in->now_ms;
        g_ctx.printed_cl = p_in->count_l;
        g_ctx.printed_cr = p_in->count_r;
        tm_printf((UB *) "enc: L %d R %d (raw counts)\n",
                  (int) p_in->count_l, (int) p_in->count_r);
    }
}

/*!
 * @brief  Reset a PID controller.
 * @param  p_pid  Controller state.
 */
static void
pid_reset (wheel_pid_t * p_pid)
{
    p_pid->integ    = 0.0f;
    p_pid->prev_err = 0.0f;
}

/*!
 * @brief  Clamp a value to a range.
 * @return The clamped value.
 */
static float32_t
clamp_f (float32_t value, float32_t low, float32_t high)
{
    float32_t result = value;

    if (value < low)
    {
        result = low;
    }
    else if (value > high)
    {
        result = high;
    }
    else
    {
        /* In range. */
    }

    return (result);
}

/*!
 * @brief  Wrap an angle to -180..180 degrees.
 * @return The wrapped angle.
 */
static float32_t
wrap180 (float32_t deg)
{
    float32_t result = deg;

    while (result > 180.0f)
    {
        result -= 360.0f;
    }

    while (result <= -180.0f)
    {
        result += 360.0f;
    }

    return (result);
}

/*!
 * @brief  Round to the nearest integer; 0 for NaN or infinity (Barr-C
 *         5.4.b.v), limited to +/-1e6.
 * @return Rounded value.
 */
static int32_t
round_i (float32_t value)
{
    int32_t result = 0;

    if (isfinite(value))
    {
        /* Clamped to +/-1e6 first, so lroundf() fits an int32_t. */
        result = (int32_t) lroundf(clamp_f(value, -1.0e6f, 1.0e6f));
    }

    return (result);
}

/*!
 * @brief  Publish speed, duty and distance to the shared state record.
 */
static void
state_publish (void)
{
    vehicle_state_set_motion(round_i(g_ctx.speed_l * MM_PER_M),
                             round_i(g_ctx.speed_r * MM_PER_M),
                             round_i(g_ctx.duty_l * PCT_PER_UNIT),
                             round_i(g_ctx.duty_r * PCT_PER_UNIT),
                             round_i((g_ctx.dist_l + g_ctx.dist_r) * HALF *
                                     MM_PER_M));
}

/*** end of file ***/

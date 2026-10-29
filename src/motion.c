/**
 * @file    motion.c
 * @brief   Motion Control Task (Buddy 2) - Section 6.2.
 *
 *  STOPPED -> MOVING{STRAIGHT, REVERSE, TURN_L, TURN_R, U_TURN}
 *          -> STOPPED | SAFE_STOP | FAULT
 *
 *  Per-wheel speed PID on the Hall encoders, encoder distance and turn
 *  targets, line steering on the line, distance-equalise + IMU heading
 *  hold off the line, stall detection (no edges for 300 ms).
 */
#include <math.h>

#include "port.h"

#include "app.h"
#include "hal.h"

#define PI_F            (3.14159265f)
#define M_PER_COUNT     ((PI_F * WHEEL_DIAMETER_M) / ENC_COUNTS_PER_REV)
#define DT_S            ((float)PERIOD_MOTION_MS / 1000.0f)
#define I_LIMIT         (0.5f)
#define LINE_MSG_MAX_AGE_MS (50U)

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
    MS_FAULT
} motion_state_id_t;

/* Encoder count direction per wheel, learned by the self-test. */
static int32_t g_sign_l = ENC_SIGN_LEFT;
static int32_t g_sign_r = ENC_SIGN_RIGHT;

typedef struct
{
    float integ;
    float prev_err;
} wheel_pid_t;

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static float wrap180(float deg)
{
    while (deg > 180.0f)   { deg -= 360.0f; }
    while (deg <= -180.0f) { deg += 360.0f; }
    return deg;
}

static float pid_step(wheel_pid_t *p, float target, float measured)
{
    float err = target - measured;
    float der = (err - p->prev_err) / DT_S;

    p->integ = clampf(p->integ + (err * DT_S), -I_LIMIT, I_LIMIT);
    p->prev_err = err;
    return (PID_KFF * target) + (PID_KP * err) + (PID_KI * p->integ) + (PID_KD * der);
}

static void pid_reset(wheel_pid_t *p)
{
    p->integ = 0.0f;
    p->prev_err = 0.0f;
}

static void post_evt(motion_evt_id_t id, float value)
{
    motion_evt_t e = { id, value };
    (void)tk_snd_mbf(g_motion_evt_q, &e, (INT)sizeof(e), TMO_POL);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
static void send_cmd(const motion_cmd_t *p_cmd)
{
    (void)tk_snd_mbf(g_motion_cmd_q, (void *)p_cmd, (INT)sizeof(*p_cmd), TMO_POL);
}

void moveForward(float speed_mps, float dist_m, bool follow_line)
{
    motion_cmd_t c = { MCMD_FWD, speed_mps, dist_m, 0.0f, follow_line, false };
    send_cmd(&c);
}

void moveBackward(float dist_m)
{
    motion_cmd_t c = { MCMD_BWD, SPEED_SEARCH_MPS, dist_m, 0.0f, false, false };
    send_cmd(&c);
}

void turnLeft(float angle_deg, bool end_on_line)
{
    motion_cmd_t c = { MCMD_TURN_L, SPEED_TURN_MPS, 0.0f, angle_deg, false, end_on_line };
    send_cmd(&c);
}

void turnRight(float angle_deg, bool end_on_line)
{
    motion_cmd_t c = { MCMD_TURN_R, SPEED_TURN_MPS, 0.0f, angle_deg, false, end_on_line };
    send_cmd(&c);
}

void stop(void)
{
    motion_cmd_t c = { MCMD_STOP, 0.0f, 0.0f, 0.0f, false, false };
    send_cmd(&c);
}

/* ------------------------------------------------------------------ */
/* Task                                                                */
/* ------------------------------------------------------------------ */
void motion_task(INT stacd, void *exinf)
{
    uint64_t          next_us = time_us_64();
    motion_state_id_t ms = MS_STOPPED;
    motion_cmd_t      cmd = { MCMD_STOP, 0.0f, 0.0f, 0.0f, false, false };
    line_pos_t        line = { 0, false, false, 0U };
    uint32_t          line_rx_ms = 0U;
    wheel_pid_t       pid_l = { 0.0f, 0.0f };
    wheel_pid_t       pid_r = { 0.0f, 0.0f };
    int32_t           prev_cl = 0;
    int32_t           prev_cr = 0;
    float             dl = 0.0f;           /* wheel distance, m        */
    float             dr = 0.0f;
    float             dl0 = 0.0f;          /* at command start         */
    float             dr0 = 0.0f;
    float             hdg0 = 0.0f;
    float             duty_l = 0.0f;
    float             duty_r = 0.0f;
    uint32_t          cmd_start_ms = 0U;
    uint32_t          cmd_limit_ms = 0U;
    int32_t           cal_l0 = 0;          /* raw counts at ENC_CAL start */
    int32_t           cal_r0 = 0;
    uint8_t           test_phase = 0U;

    (void)stacd;
    (void)exinf;

    prev_cl = hal_enc_left();
    prev_cr = hal_enc_right();

    for (;;)
    {
        motion_state_t st;
        motion_cmd_t   rx;
        line_pos_t     lrx;
        int32_t        cl;
        int32_t        cr;
        float          vl;
        float          vr;
        float          tgt_l = 0.0f;
        float          tgt_r = 0.0f;
        bool           done = false;
        uint32_t       t;

        period_wait(&next_us, PERIOD_MOTION_MS);
        t = now_ms();
        heartbeat(HB_MOTION);
        state_read(&st);

        /* ---- encoders -> speed / distance ---- */
        cl = hal_enc_left();
        cr = hal_enc_right();
        vl = (float)(g_sign_l * (cl - prev_cl)) * M_PER_COUNT;
        vr = (float)(g_sign_r * (cr - prev_cr)) * M_PER_COUNT;
        prev_cl = cl;
        prev_cr = cr;
        dl += vl;
        dr += vr;
        vl /= DT_S;
        vr /= DT_S;

        /* ---- latest line position ---- */
        while (tk_rcv_mbf(g_line_pos_q, &lrx, TMO_POL) > 0)
        {
            line = lrx;
            line_rx_ms = t;
        }

        /* ---- safety latch overrides everything ---- */
        if (safety_is_latched())
        {
            if (MS_SAFE_STOP != ms)
            {
                ms = MS_SAFE_STOP;     /* Safety Monitor already braked */
            }
        }
        else if (MS_SAFE_STOP == ms)
        {
            ms = MS_STOPPED;           /* fault cleared                 */
        }
        else
        {
            /* normal operation */
        }

        /* ---- new command (latest wins) ---- */
        while (tk_rcv_mbf(g_motion_cmd_q, &rx, TMO_POL) > 0)
        {
            if ((MS_SAFE_STOP == ms) || ((MS_FAULT == ms) && (MCMD_STOP != rx.id)))
            {
                continue;
            }
            cmd = rx;
            dl0 = dl;
            dr0 = dr;
            hdg0 = st.heading_deg;
            cmd_start_ms = t;
            pid_reset(&pid_l);
            pid_reset(&pid_r);

            switch (cmd.id)
            {
                case MCMD_FWD:    ms = MS_STRAIGHT; break;
                case MCMD_BWD:    ms = MS_REVERSE;  break;
                case MCMD_TURN_L: ms = MS_TURN_L;   break;
                case MCMD_TURN_R: ms = MS_TURN_R;   break;
                case MCMD_UTURN:
                    ms = MS_UTURN;
                    cmd.angle = 180.0f;
                    cmd.speed = SPEED_TURN_MPS;
                    break;
                case MCMD_ENC_CAL:
                    ms = MS_ENC_CAL;
                    cal_l0 = cl;               /* raw counts, before sign */
                    cal_r0 = cr;
                    break;
                case MCMD_MOTOR_TEST:
                    ms = MS_MOTOR_TEST;
                    cal_l0 = cl;
                    cal_r0 = cr;
                    test_phase = 0U;           /* 0: left wheel, 1: right */
                    break;
                case MCMD_STOP:
                default:          ms = MS_STOPPED;  break;
            }

            /* generous time limit: 2x nominal + 2 s */
            if ((MS_STRAIGHT == ms) || (MS_REVERSE == ms))
            {
                cmd_limit_ms = (cmd.dist > 0.0f) ?
                    (uint32_t)((2000.0f * cmd.dist) / fmaxf(cmd.speed, 0.05f)) + 2000U : 0U;
            }
            else if ((MS_STOPPED != ms) && (MS_ENC_CAL != ms) && (MS_MOTOR_TEST != ms))
            {
                float arc = (cmd.angle * PI_F / 180.0f) * (TRACK_WIDTH_M * 0.5f);
                cmd_limit_ms = (uint32_t)((2000.0f * arc) / cmd.speed) + 2000U;
            }
            else
            {
                cmd_limit_ms = 0U;
            }
        }

        /* ---- set-points per state ---- */
        switch (ms)
        {
            case MS_STRAIGHT:
            {
                float v = cmd.speed;
                bool  fresh = (t - line_rx_ms) < LINE_MSG_MAX_AGE_MS;

                if (cmd.follow_line && fresh)
                {
                    /* Steer TOWARD the sensor that sees black (LINE_STEER_DIR
                     * = +1), and slow down while correcting:
                     *   pos -1 (left black)  -> slow left wheel:  steer left
                     *   pos +1 (right black) -> slow right wheel: steer right */
                    float steer = (float)(LINE_STEER_DIR * line.pos) * K_LINE_STEER;
                    if (0 != line.pos)
                    {
                        v = fminf(v, SPEED_STEER_MPS);
                    }
                    tgt_l = v + steer;
                    tgt_r = v - steer;
                }
                else
                {
                    float d_err = (dl - dl0) - (dr - dr0);   /* + = drifting right */
                    float h_err = wrap180(st.heading_deg - hdg0);
                    float corr  = (K_DIST_EQ * d_err) + (K_HEADING * h_err);
                    tgt_l = v - corr;
                    tgt_r = v + corr;
                }
                if ((cmd.dist > 0.0f) &&
                    ((((dl - dl0) + (dr - dr0)) * 0.5f) >= cmd.dist))
                {
                    done = true;
                }
                break;
            }

            case MS_REVERSE:
            {
                float corr = K_DIST_EQ * ((dl - dl0) - (dr - dr0));
                tgt_l = -cmd.speed - corr;
                tgt_r = -cmd.speed + corr;
                if ((((dl0 - dl) + (dr0 - dr)) * 0.5f) >= cmd.dist)
                {
                    done = true;
                }
                break;
            }

            case MS_TURN_L:
            case MS_TURN_R:
            case MS_UTURN:
            {
                float arc  = (cmd.angle * PI_F / 180.0f) * (TRACK_WIDTH_M * 0.5f);
                float trav = (fabsf(dl - dl0) + fabsf(dr - dr0)) * 0.5f;
                float frac = (arc > 0.0f) ? (trav / arc) : 1.0f;
                bool  left = (MS_TURN_R != ms);   /* U-turn spins left */

                tgt_l = left ? -cmd.speed : cmd.speed;
                tgt_r = left ? cmd.speed : -cmd.speed;

                if (frac >= 1.0f)
                {
                    done = true;
                }
                else if (cmd.end_on_line && (frac >= TURN_EARLY_FRACTION) &&
                         line.line_seen && ((t - line_rx_ms) < LINE_MSG_MAX_AGE_MS))
                {
                    done = true;                  /* new branch found */
                }
                else
                {
                    /* keep turning */
                }

                if (done)
                {
                    /* IMU cross-check (magnetometer heading) */
                    float imu_turn = fabsf(wrap180(st.heading_deg - hdg0));
                    float enc_turn = frac * cmd.angle;
                    if (fabsf(imu_turn - enc_turn) > TURN_XCHECK_DEG)
                    {
                        telemetry_post_event(EV_TURN_XCHECK, 0U, (int16_t)enc_turn, (int16_t)imu_turn);
                    }
                }
                break;
            }

            case MS_STOPPED:
            case MS_SAFE_STOP:
            case MS_FAULT:
            default:
                break;
        }

        /* ---- outputs ---- */
        if (MS_MOTOR_TEST == ms)
        {
            /* Bench test, open loop, 1 s per phase, wheels off the ground:
             *   phase 0: left  +duty   phase 1: left  -duty
             *   phase 2: right +duty   phase 3: right -duty
             * A negative duty drives the OTHER input pin of that channel,
             * so the four results show the driver's real truth table.
             * Each phase reports the raw counts of both encoders. */
            if ((t - cmd_start_ms) >= MOTOR_TEST_MS)
            {
                telemetry_post_event(EV_MOTOR_TEST, (uint8_t)(test_phase + 1U),
                            (int16_t)(cl - cal_l0), (int16_t)(cr - cal_r0));
                hal_motor_brake();
                if (test_phase < 3U)
                {
                    test_phase++;
                    cal_l0 = cl;
                    cal_r0 = cr;
                    cmd_start_ms = t;
                }
                else
                {
                    ms = MS_STOPPED;
                }
                duty_l = 0.0f;
                duty_r = 0.0f;
            }
            else if ((t - cmd_start_ms) < 300U)
            {
                duty_l = 0.0f;                    /* let the last wheel stop */
                duty_r = 0.0f;
                hal_motor_brake();
            }
            else
            {
                float d = ((test_phase & 1U) == 0U) ? MOTOR_TEST_DUTY : -MOTOR_TEST_DUTY;

                duty_l = (test_phase < 2U) ? d : 0.0f;
                duty_r = (test_phase < 2U) ? 0.0f : d;
                hal_motor_set(duty_l, duty_r);
            }
        }
        else if (MS_ENC_CAL == ms)
        {
            /* Open-loop forward drive on both wheels; the sign of each raw
             * encoder delta tells which way that encoder counts "forward".
             * Motor polarity (LEFT/RIGHT_FWD_ON_A) is taken as correct. */
            int32_t raw_l = cl - cal_l0;
            int32_t raw_r = cr - cal_r0;
            bool    ok_l = (raw_l >= ENC_CAL_COUNTS) || (raw_l <= -ENC_CAL_COUNTS);
            bool    ok_r = (raw_r >= ENC_CAL_COUNTS) || (raw_r <= -ENC_CAL_COUNTS);

            if (ok_l && ok_r)
            {
                float spin = fabsf(wrap180(st.heading_deg - hdg0));

                hal_motor_brake();
                duty_l = 0.0f;
                duty_r = 0.0f;
                if (spin > ENC_CAL_SPIN_DEG)
                {
                    /* Equal forward drive made the car rotate: one motor is
                     * wired backwards (fix *_FWD_ON_A). Keep the old signs. */
                    ms = MS_FAULT;
                    telemetry_post_event(EV_ENC_CAL, 4U, (int16_t)spin, 0);
                    post_evt(MEVT_STALL, 0.0f);
                }
                else
                {
                    g_sign_l = (raw_l > 0) ? 1 : -1;
                    g_sign_r = (raw_r > 0) ? 1 : -1;
                    ms = MS_STOPPED;
                    telemetry_post_event(EV_ENC_CAL, 0U, (int16_t)g_sign_l, (int16_t)g_sign_r);
                    post_evt(MEVT_DONE, 0.0f);
                }
            }
            else if ((t - cmd_start_ms) > ENC_CAL_TIMEOUT_MS)
            {
                /* code: bit0 = left encoder silent, bit1 = right silent */
                uint8_t silent = (uint8_t)((ok_l ? 0U : 1U) | (ok_r ? 0U : 2U));

                hal_motor_brake();
                duty_l = 0.0f;
                duty_r = 0.0f;
                ms = MS_FAULT;
                telemetry_post_event(EV_ENC_CAL, silent, (int16_t)raw_l, (int16_t)raw_r);
                post_evt(MEVT_STALL, 0.0f);
            }
            else
            {
                duty_l = ENC_CAL_DUTY;
                duty_r = ENC_CAL_DUTY;
                hal_motor_set(duty_l, duty_r);
            }
        }
        else if ((MS_STOPPED == ms) || (MS_SAFE_STOP == ms) || (MS_FAULT == ms))
        {
            duty_l = 0.0f;
            duty_r = 0.0f;
            if (MS_SAFE_STOP != ms)
            {
                hal_motor_brake();     /* Safety owns the outputs in SAFE_STOP */
            }
        }
        else if (done)
        {
            duty_l = 0.0f;
            duty_r = 0.0f;
            hal_motor_brake();
            ms = MS_STOPPED;
            post_evt(MEVT_DONE, 0.0f);
        }
        else
        {
            uint32_t edge_l = hal_enc_last_edge_ms_left();
            uint32_t edge_r = hal_enc_last_edge_ms_right();

            duty_l = clampf(pid_step(&pid_l, tgt_l, vl), -1.0f, 1.0f);
            duty_r = clampf(pid_step(&pid_r, tgt_r, vr), -1.0f, 1.0f);

            if (edge_l < cmd_start_ms) { edge_l = cmd_start_ms; }
            if (edge_r < cmd_start_ms) { edge_r = cmd_start_ms; }
            /* The ISR can stamp an edge AFTER t was sampled at the top of
             * this loop, so edge may be t+1. Unsigned (t - edge) would then
             * wrap to ~4e9 ms and look like a 300 ms stall: clamp it.     */
            if ((int32_t)(t - edge_l) < 0) { edge_l = t; }
            if ((int32_t)(t - edge_r) < 0) { edge_r = t; }

            if (((fabsf(duty_l) > STALL_MIN_DUTY) && ((t - edge_l) > STALL_TIMEOUT_MS)) ||
                ((fabsf(duty_r) > STALL_MIN_DUTY) && ((t - edge_r) > STALL_TIMEOUT_MS)) ||
                ((0U != cmd_limit_ms) && ((t - cmd_start_ms) > cmd_limit_ms)))
            {
                /* v1 bits: 1 = left wheel no edges, 2 = right wheel no
                 * edges, 4 = command time limit; v2 = motion state.   */
                uint16_t why = (uint16_t)(
                    (((fabsf(duty_l) > STALL_MIN_DUTY) && ((t - edge_l) > STALL_TIMEOUT_MS)) ? 1U : 0U) |
                    (((fabsf(duty_r) > STALL_MIN_DUTY) && ((t - edge_r) > STALL_TIMEOUT_MS)) ? 2U : 0U) |
                    (((0U != cmd_limit_ms) && ((t - cmd_start_ms) > cmd_limit_ms)) ? 4U : 0U));

                hal_motor_brake();
                duty_l = 0.0f;
                duty_r = 0.0f;
                post_evt(MEVT_STALL, 0.0f);
                telemetry_post_event(EV_FAULT, (uint8_t)FC_STALL, (int16_t)why, (int16_t)ms);
                ms = MS_FAULT;
            }
            else
            {
                hal_motor_set(duty_l, duty_r);
            }
        }

        /* ---- publish to state record ---- */
        state_lock();
        state_raw()->duty_l = duty_l;
        state_raw()->duty_r = duty_r;
        state_raw()->speed_l_mps = vl;
        state_raw()->speed_r_mps = vr;
        state_raw()->dist_m = (dl + dr) * 0.5f;
        state_unlock();
    }
}

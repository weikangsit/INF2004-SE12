/**
 * @file    ultrasonic.c
 * @brief   Ultrasonic Scan Task (Buddy 5) - Section 6.3.
 *
 *  STANDBY -> COARSE_SCAN -> FINE_SCAN -> PROFILE -> PLAN
 *  (BYPASS_EXEC and LINE_SEARCH are driven by the Navigation FSM.)
 *
 *  Servo convention: 0 deg = right, 90 = straight ahead, 180 = left.
 *  Lateral x (cm) = r * cos(theta): positive to the right.
 */
#include <math.h>

#include "port.h"

#include "app.h"
#include "hal.h"
#include "ultrasonic.h"

#define DEG2RAD          (0.01745329f)
#define NO_ECHO_CM       (999.0f)
#define COARSE_N         (5U)
#define FINE_MAX         (7U)
#define MIN_SIDE_CLEAR   (30.0f)

static const int16_t g_coarse_deg[COARSE_N] = { 30, 60, 90, 120, 150 };
static volatile float g_last_front_cm = NO_ECHO_CM;

float us_front_cm(void) { return g_last_front_cm; }

/* One ping. Returns range in cm or NO_ECHO_CM. */
static float ping_cm(void)
{
    UINT     ptn;
    uint32_t us = 0U;
    ER       er;

    (void)tk_clr_flg(g_evt_flg, ~EVT_ECHO_DONE);
    hal_us_trigger();
    er = tk_wai_flg(g_evt_flg, EVT_ECHO_DONE, TWF_ORW | TWF_BITCLR, &ptn,
                    (TMO)(US_ECHO_TIMEOUT_US / 1000U));
    if ((E_OK != er) || !hal_us_echo_us(&us) || (us >= US_ECHO_TIMEOUT_US))
    {
        return NO_ECHO_CM;
    }
    return (float)us / 58.0f;
}

static float ping_at(int16_t deg)
{
    float r;

    hal_servo_set_deg(deg);
    (void)tk_dly_tsk(SERVO_SETTLE_MS);
    r = ping_cm();
    (void)tk_dly_tsk(US_MIN_GAP_MS);
    heartbeat(HB_US);
    return r;
}

static void post(const obstacle_msg_t *p_m)
{
    (void)tk_snd_mbf(g_obstacle_q, (void *)p_m, (INT)sizeof(*p_m), TMO_POL);
}

/* Full coarse + fine scan and plan. Returns the route message kind. */
static obstacle_kind_t scan_and_plan(bool retry_done)
{
    float          coarse[COARSE_N];
    int16_t        fine_deg[FINE_MAX];
    float          fine[FINE_MAX];
    uint32_t       n_fine = 0U;
    uint32_t       i;
    uint32_t       best = 0U;
    obstacle_msg_t m;
    float          x_min = 1e6f;
    float          x_max = -1e6f;
    float          half = (ROBOT_WIDTH_CM * 0.5f) + CLEAR_MARGIN_CM;
    bool           any_hit = false;

    /* ---- COARSE_SCAN ---- */
    for (i = 0U; i < COARSE_N; i++)
    {
        coarse[i] = ping_at(g_coarse_deg[i]);
        if (coarse[i] < coarse[best])
        {
            best = i;
        }
    }

    /* ---- FINE_SCAN: +/-15 deg around the closest coarse hit ---- */
    {
        int16_t c = g_coarse_deg[best];
        int16_t a;

        for (a = (int16_t)(c - SERVO_FINE_SPAN_DEG);
             (a <= (c + SERVO_FINE_SPAN_DEG)) && (n_fine < FINE_MAX);
             a = (int16_t)(a + SERVO_FINE_STEP_DEG))
        {
            if ((a >= 0) && (a <= 180))
            {
                fine_deg[n_fine] = a;
                fine[n_fine] = ping_at(a);
                n_fine++;
            }
        }
    }
    hal_servo_set_deg(SERVO_CENTRE_DEG);

    /* ---- PROFILE ---- */
    m.kind = OBS_PROFILE;
    m.range_cm = NO_ECHO_CM;
    m.angle_deg = SERVO_CENTRE_DEG;
    for (i = 0U; i < n_fine; i++)
    {
        if (fine[i] < US_SCAN_RANGE_CM)
        {
            float x = fine[i] * cosf((float)fine_deg[i] * DEG2RAD);

            any_hit = true;
            x_min = fminf(x_min, x);
            x_max = fmaxf(x_max, x);
            if (fine[i] < m.range_cm)
            {
                m.range_cm = fine[i];
                m.angle_deg = fine_deg[i];
            }
        }
    }
    m.width_cm = any_hit ? (x_max - x_min) : 0.0f;
    m.clear_left_cm  = coarse[COARSE_N - 1U];   /* ray at 150 deg */
    m.clear_right_cm = coarse[0];               /* ray at 30 deg  */
    post(&m);
    /* code = closest-point bearing (deg), v1 = range, v2 = width (0.1 cm) */
    telem_event(EV_OBSTACLE, (uint8_t)m.angle_deg,
                (int16_t)fminf(m.range_cm * 10.0f, 32767.0f), (int16_t)(m.width_cm * 10.0f));

    /* ---- PLAN ---- */
    if (!any_hit || (x_min > half) || (x_max < -half))
    {
        return OBS_ROUTE_CONTINUE;          /* not in our path        */
    }
    if ((m.clear_left_cm >= MIN_SIDE_CLEAR) || (m.clear_right_cm >= MIN_SIDE_CLEAR))
    {
        return (m.clear_left_cm >= m.clear_right_cm) ? OBS_ROUTE_LEFT : OBS_ROUTE_RIGHT;
    }
    return retry_done ? OBS_NO_ROUTE : OBS_ROUTE_REVERSE;
}

void ultrasonic_task(INT stacd, void *exinf)
{
    uint64_t next_us = time_us_64();
    bool     near_reported = false;
    bool     retry_done = false;

    (void)stacd;
    (void)exinf;

    hal_servo_set_deg(SERVO_CENTRE_DEG);

    for (;;)
    {
        scan_req_t req;
        float      r;

        /* ---- scan request from Navigation FSM? ---- */
        if (tk_rcv_mbf(g_scan_req_q, &req, TMO_POL) > 0)
        {
            if (SCAN_REQ_PROFILE == req)
            {
                obstacle_msg_t route = { OBS_NO_ROUTE, 0.0f, 0, 0.0f, 0.0f, 0.0f };

                route.kind = scan_and_plan(retry_done);
                retry_done = (OBS_ROUTE_REVERSE == route.kind);
                post(&route);
                next_us = time_us_64();
            }
            else
            {
                retry_done = false;
                hal_servo_set_deg(SERVO_CENTRE_DEG);
            }
            near_reported = false;
        }

        /* ---- STANDBY: forward ping every 100 ms ---- */
        period_wait(&next_us, PERIOD_US_MS);
        heartbeat(HB_US);
        r = ping_cm();
        g_last_front_cm = r;

        if (r < US_OBSTACLE_CM)
        {
            if (!near_reported)
            {
                obstacle_msg_t m = { OBS_NEAR, r, SERVO_CENTRE_DEG, 0.0f, 0.0f, 0.0f };
                post(&m);
                near_reported = true;
            }
        }
        else if (r > (US_OBSTACLE_CM + 5.0f))
        {
            near_reported = false;          /* hysteresis */
            retry_done = false;
        }
        else
        {
            /* in hysteresis band */
        }
    }
}

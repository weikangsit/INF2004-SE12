/** @file    ultrasonic_scan.c
 *  @brief   Ultrasonic Scan task (Buddy 5), see ultrasonic_scan.h.
 *
 *  STANDBY -> COARSE_SCAN -> FINE_SCAN -> PROFILE -> PLAN (Week 6 6.3).
 *  The task reports its heartbeat after every ping, so a scan (about
 *  12 x 180 ms) never looks like a stalled task to the Safety Monitor.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <string.h>

#include "port.h"

#include "car_config.h"
#include "hal.h"
#include "hal_irq.h"
#include "obstacle_profile.h"
#include "proto.h"
#include "safety.h"
#include "systime.h"
#include "telemetry.h"
#include "ultrasonic_scan.h"

#define US_PER_CM            (58U)      /* round trip at 343 m/s         */
#define ECHO_WAIT_MS         ((US_ECHO_TIMEOUT_US / 1000U) + 1U)
#define NEAR_HYSTERESIS_CM   (5U)
#define FINE_MAX             (((2U * SERVO_FINE_SPAN_DEG) /                \
                               SERVO_FINE_STEP_DEG) + 1U)
#define RAY_MAX              (SCAN_COARSE_RAYS + FINE_MAX)
#define TENTHS               (10U)
#define TENTHS_NO_ECHO       (5110)     /* 511 cm = no echo in EV_OBSTACLE */
#define Q_DEPTH              (4U)
#define MBF_SLOT_BYTES       (8U)       /* 4-byte header + 1 byte         */

typedef enum
{
    SCAN_REQ_PROFILE = 0,
    SCAN_REQ_ABORT
} scan_req_t;

static ID                g_req_q      = 0;
static ID                g_result_q   = 0;
static volatile uint16_t g_front_cm   = US_RANGE_NO_ECHO;
static volatile bool     g_b_ever_ok  = false;

static uint16_t         ping_cm (void);
static uint16_t         ping_at (uint32_t angle_deg);
static obstacle_route_t scan_and_plan (bool b_retry_done);
static uint32_t         sweep_coarse (obstacle_ray_t * p_rays);
static uint32_t         sweep_fine (obstacle_ray_t * p_rays, uint32_t count,
                                    uint32_t centre_deg);
static void             result_post (us_result_t result);
static void             profile_report (obstacle_profile_t const * p_prof);

/*!
 * @brief  Create the request and result queues. Call from usermain()
 *         before the tasks start.
 */
void
ultrasonic_scan_init (void)
{
    T_CMBF create;

    (void) memset(&create, 0, sizeof(create));
    create.mbfatr = TA_TFIFO;
    create.bufsz  = (SZ) (Q_DEPTH * MBF_SLOT_BYTES);           /* 32 B    */
    create.maxmsz = (INT) sizeof(uint8_t);
    g_req_q       = tk_cre_mbf(&create);
    g_result_q    = tk_cre_mbf(&create);
}

/*!
 * @brief  Ultrasonic Scan task body (priority PRI_US, 100 ms standby).
 * @param  stacd    Start code (unused).
 * @param  p_exinf  Extended information (unused).
 */
void
ultrasonic_scan_task (INT stacd, void * p_exinf)
{
    uint64_t next_us        = time_us_64();
    bool     b_near_sent    = false;
    bool     b_retry_done   = false;

    (void) stacd;
    (void) p_exinf;

    hal_servo_set_deg(SERVO_CENTRE_DEG);

    for (;;)
    {
        uint8_t  req   = 0U;
        uint16_t range = US_RANGE_NO_ECHO;

        if (tk_rcv_mbf(g_req_q, &req, TMO_POL) > 0)
        {
            if (SCAN_REQ_PROFILE == req)
            {
                obstacle_route_t const route = scan_and_plan(b_retry_done);

                b_retry_done = (OBSTACLE_ROUTE_REVERSE == route);
                /* Route values 0..5 are the same as us_result_t 0..5. */
                result_post((us_result_t) route);
                next_us = time_us_64();
            }
            else
            {
                b_retry_done = false;
                hal_servo_set_deg(SERVO_CENTRE_DEG);
            }

            b_near_sent = false;
        }

        /* STANDBY: one ping straight ahead per period. */
        systime_period_wait(&next_us, PERIOD_US_MS);
        safety_heartbeat(SAFETY_HB_US);
        range      = ping_cm();
        g_front_cm = range;

        if ((US_RANGE_NO_ECHO != range) && (range < US_OBSTACLE_CM))
        {
            if (!b_near_sent)
            {
                result_post(US_RESULT_NEAR);
                b_near_sent = true;
            }
        }
        else if ((US_RANGE_NO_ECHO != range) &&
                 (range > (US_OBSTACLE_CM + NEAR_HYSTERESIS_CM)))
        {
            b_near_sent  = false;                 /* hysteresis           */
            b_retry_done = false;
        }
        else
        {
            /* In the hysteresis band, or no echo: no change. */
        }
    }
}

/*!
 * @brief  Ask for a profile scan (Navigation FSM, car stopped).
 */
void
ultrasonic_scan_request (void)
{
    uint8_t const req = (uint8_t) SCAN_REQ_PROFILE;

    (void) tk_snd_mbf(g_req_q, &req, (INT) sizeof(req), TMO_POL);
}

/*!
 * @brief  Cancel the retry state and centre the servo (Navigation FSM).
 */
void
ultrasonic_scan_abort (void)
{
    uint8_t const req = (uint8_t) SCAN_REQ_ABORT;

    (void) tk_snd_mbf(g_req_q, &req, (INT) sizeof(req), TMO_POL);
}

/*!
 * @brief  Take one result, if any (Navigation FSM).
 * @param  p_result  Output result.
 * @return True if a result was taken.
 */
bool
ultrasonic_scan_result_take (us_result_t * p_result)
{
    uint8_t    msg   = 0U;
    bool const b_got = (tk_rcv_mbf(g_result_q, &msg, TMO_POL) > 0);

    if (b_got)
    {
        /* Only us_result_t values (0..6) are queued. */
        *p_result = (us_result_t) msg;
    }

    return (b_got);
}

/*!
 * @brief  Latest standby range straight ahead.
 * @return Centimetres, or US_RANGE_NO_ECHO.
 */
uint16_t
ultrasonic_scan_front_cm (void)
{
    return (g_front_cm);
}

/*!
 * @brief  Report whether the sensor has ever answered (self-test).
 * @return True after at least one valid echo.
 */
bool
ultrasonic_scan_is_ok (void)
{
    return (g_b_ever_ok);
}

/*!
 * @brief  One ping.
 * @return Range in cm, or US_RANGE_NO_ECHO if the echo is missing or
 *         longer than US_ECHO_TIMEOUT_US.
 */
static uint16_t
ping_cm (void)
{
    uint32_t echo_us = 0U;
    uint16_t range   = US_RANGE_NO_ECHO;

    hal_echo_arm();
    hal_us_trigger();

    if (hal_echo_wait(ECHO_WAIT_MS) && hal_us_echo_us(&echo_us) &&
        (echo_us < US_ECHO_TIMEOUT_US))
    {
        /* echo_us < 30000, so the range is under 520 cm. */
        range       = (uint16_t) (echo_us / US_PER_CM);
        g_b_ever_ok = true;
    }

    return (range);
}

/*!
 * @brief  Turn the servo, let it settle, ping, and keep the minimum gap
 *         between pings. Reports the heartbeat.
 * @param  angle_deg  Servo angle.
 * @return Range in cm, or US_RANGE_NO_ECHO.
 */
static uint16_t
ping_at (uint32_t angle_deg)
{
    uint16_t range = US_RANGE_NO_ECHO;

    hal_servo_set_deg(angle_deg);
    (void) tk_dly_tsk((RELTIM) SERVO_SETTLE_MS);
    range = ping_cm();
    (void) tk_dly_tsk((RELTIM) US_MIN_GAP_MS);
    safety_heartbeat(SAFETY_HB_US);

    return (range);
}

/*!
 * @brief  Full coarse + fine sweep, profile and plan.
 * @param  b_retry_done  True if the car already backed off once.
 * @return Suggested route.
 */
static obstacle_route_t
scan_and_plan (bool b_retry_done)
{
    static obstacle_limits_t const limits =
    {
        US_SCAN_RANGE_CM,
        (ROBOT_WIDTH_CM / 2U) + CLEAR_MARGIN_CM,
        MIN_SIDE_CLEAR_CM
    };
    obstacle_ray_t     rays[RAY_MAX];
    obstacle_profile_t coarse;
    obstacle_profile_t profile;
    uint32_t           count = 0U;

    count = sweep_coarse(rays);
    obstacle_profile_build(rays, count, &limits, &coarse);
    count = sweep_fine(rays, count, coarse.closest_deg);
    hal_servo_set_deg(SERVO_CENTRE_DEG);

    obstacle_profile_build(rays, count, &limits, &profile);
    profile_report(&profile);

    return (obstacle_profile_plan(&profile, &limits, b_retry_done));
}

/*!
 * @brief  Coarse sweep: SCAN_COARSE_RAYS rays spread evenly over
 *         SCAN_MIN_DEG..SCAN_MAX_DEG.
 * @param  p_rays  Output rays.
 * @return Number of rays.
 */
static uint32_t
sweep_coarse (obstacle_ray_t * p_rays)
{
    uint32_t const step = (SCAN_MAX_DEG - SCAN_MIN_DEG) /
                          (SCAN_COARSE_RAYS - 1U);
    uint32_t       idx  = 0U;

    for (idx = 0U; idx < SCAN_COARSE_RAYS; idx++)
    {
        uint32_t const angle = SCAN_MIN_DEG + (idx * step);

        /* Angles are 0..180, so they fit a uint16_t. */
        p_rays[idx].angle_deg = (uint16_t) angle;
        p_rays[idx].range_cm  = ping_at(angle);
    }

    return (SCAN_COARSE_RAYS);
}

/*!
 * @brief  Fine sweep: +/-SERVO_FINE_SPAN_DEG around the closest coarse
 *         hit (or straight ahead), clipped to the sweep limits.
 * @param  p_rays      Ray array; the fine rays are appended.
 * @param  count       Rays already in the array.
 * @param  centre_deg  Centre of the fine sweep.
 * @return New number of rays.
 */
static uint32_t
sweep_fine (obstacle_ray_t * p_rays, uint32_t count, uint32_t centre_deg)
{
    uint32_t const centre = (centre_deg < SCAN_MIN_DEG) ? SCAN_MIN_DEG :
                                                          centre_deg;
    uint32_t const low    = ((centre - SCAN_MIN_DEG) > SERVO_FINE_SPAN_DEG) ?
                            (centre - SERVO_FINE_SPAN_DEG) : SCAN_MIN_DEG;
    uint32_t       angle  = 0U;
    uint32_t       total = count;

    for (angle = low;
         (angle <= (centre + SERVO_FINE_SPAN_DEG)) &&
         (angle <= SCAN_MAX_DEG) && (total < RAY_MAX);
         angle += SERVO_FINE_STEP_DEG)
    {
        /* Angles are 0..180, so they fit a uint16_t. */
        p_rays[total].angle_deg = (uint16_t) angle;
        p_rays[total].range_cm  = ping_at(angle);
        total++;
    }

    return (total);
}

/*!
 * @brief  Queue a result for the Navigation FSM (1 byte).
 * @param  result  Result.
 */
static void
result_post (us_result_t result)
{
    /* us_result_t values are 0..6. */
    uint8_t const msg = (uint8_t) result;

    (void) tk_snd_mbf(g_result_q, &msg, (INT) sizeof(msg), TMO_POL);
}

/*!
 * @brief  Report the profile summary as an EV_OBSTACLE event (raw rays
 *         stay on the car, Tribunal 1 curveball response).
 * @param  p_prof  Profile.
 */
static void
profile_report (obstacle_profile_t const * p_prof)
{
    int32_t const range_x10 = (OBSTACLE_NO_ECHO == p_prof->closest_cm) ?
                              TENTHS_NO_ECHO :
                              ((int32_t) p_prof->closest_cm * (int32_t) TENTHS);

    /* Angle <= 180 fits a byte; range and width are under 3277 cm. */
    telemetry_post_event(EV_OBSTACLE, (uint8_t) p_prof->closest_deg,
                         (int16_t) range_x10,
                         (int16_t) ((int32_t) p_prof->width_cm *
                                    (int32_t) TENTHS));
}

/*** end of file ***/

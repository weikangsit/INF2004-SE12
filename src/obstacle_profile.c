/** @file    obstacle_profile.c
 *  @brief   Obstacle profile and route planning (see obstacle_profile.h).
 */
#include "obstacle_profile.h"

#define SCALE                (1000)     /* cos in thousandths            */
#define AHEAD_DEG            (90U)
#define STRAIGHT_DEG         (180U)
#define CENTRE_SPAN_DEG      (15U)
#define BHASKARA_A           (4)
#define BHASKARA_B           (40500)
#define NO_HIT_MIN           (INT32_MAX)
#define NO_HIT_MAX           (INT32_MIN)

static int32_t  sin1000 (uint32_t angle_deg);
static uint32_t angle_distance (uint32_t angle_deg, uint32_t ref_deg);

/*!
 * @brief  Build the profile of one sweep.
 * @param  p_rays    Rays of the sweep, in any order.
 * @param  count     Number of rays (at least 1).
 * @param  p_limits  Planning limits.
 * @param  p_out     Output profile.
 */
void
obstacle_profile_build (obstacle_ray_t const * p_rays, uint32_t count,
                        obstacle_limits_t const * p_limits,
                        obstacle_profile_t * p_out)
{
    int32_t  x_min    = NO_HIT_MIN;
    int32_t  x_max    = NO_HIT_MAX;
    uint32_t left_i   = 0U;
    uint32_t right_i  = 0U;
    uint32_t idx      = 0U;

    p_out->closest_cm     = OBSTACLE_NO_ECHO;
    p_out->closest_deg    = AHEAD_DEG;
    p_out->width_cm       = 0U;
    p_out->valid_rays     = 0U;
    p_out->b_in_path      = false;
    p_out->b_centre_known = false;

    for (idx = 0U; idx < count; idx++)
    {
        obstacle_ray_t const * const p_ray = &p_rays[idx];

        /* Edge rays: largest angle is the left edge, smallest the right. */
        left_i  = (p_ray->angle_deg > p_rays[left_i].angle_deg) ? idx : left_i;
        right_i = (p_ray->angle_deg < p_rays[right_i].angle_deg) ? idx :
                                                                  right_i;

        if (OBSTACLE_NO_ECHO != p_ray->range_cm)
        {
            /* count of rays is small (at most a few dozen). */
            p_out->valid_rays = (uint8_t) (p_out->valid_rays + 1U);

            if (angle_distance(p_ray->angle_deg, AHEAD_DEG) <=
                CENTRE_SPAN_DEG)
            {
                p_out->b_centre_known = true;
            }

            if (p_ray->range_cm < p_limits->scan_range_cm)
            {
                /* range < scan range (a few hundred cm), so the product
                 * fits an int32_t. */
                int32_t const lateral = ((int32_t) p_ray->range_cm *
                                   obstacle_profile_cos1000(p_ray->angle_deg))
                                  / SCALE;

                x_min = (lateral < x_min) ? lateral : x_min;
                x_max = (lateral > x_max) ? lateral : x_max;

                /* half_corridor_cm is a uint16_t, so it fits int32_t. */
                if ((lateral <= (int32_t) p_limits->half_corridor_cm) &&
                    (lateral >= -(int32_t) p_limits->half_corridor_cm))
                {
                    p_out->b_in_path = true;
                }

                if (p_ray->range_cm < p_out->closest_cm)
                {
                    p_out->closest_cm  = p_ray->range_cm;
                    p_out->closest_deg = p_ray->angle_deg;
                }
            }
        }
    }

    if (x_max >= x_min)
    {
        /* Both come from ranges under scan_range_cm, so the difference
         * is small and positive. */
        p_out->width_cm = (uint16_t) (x_max - x_min);
    }

    p_out->clear_left_cm  = p_rays[left_i].range_cm;
    p_out->clear_right_cm = p_rays[right_i].range_cm;
}

/*!
 * @brief  Choose a route from a profile.
 *
 *  No valid echo at all, or a free path with no valid ray ahead, cannot be
 *  trusted: UNSAFE. A blocked path takes the side whose edge ray shows at
 *  least min_side_clear_cm (the larger one; left on a tie). A side whose
 *  edge ray had no echo is unknown and never chosen. If no side is both
 *  known and clear, the scan is UNSAFE: from the stop point the 15 / 165
 *  deg edge rays pass the obstacle's face, so a short edge ray is a real
 *  wall beside the car, not the obstacle itself.
 *
 * @param  p_prof        Profile of the sweep.
 * @param  p_limits      Planning limits.
 * @return Route.
 */
obstacle_route_t
obstacle_profile_plan (obstacle_profile_t const * p_prof,
                       obstacle_limits_t const * p_limits)
{
    /* A no-echo edge (OBSTACLE_NO_ECHO) is unknown, never clear. */
    bool const b_left_clear  = (OBSTACLE_NO_ECHO != p_prof->clear_left_cm) &&
                               (p_prof->clear_left_cm >=
                                p_limits->min_side_clear_cm);
    bool const b_right_clear = (OBSTACLE_NO_ECHO != p_prof->clear_right_cm) &&
                               (p_prof->clear_right_cm >=
                                p_limits->min_side_clear_cm);
    obstacle_route_t route   = OBSTACLE_ROUTE_UNSAFE;

    if (0U == p_prof->valid_rays)
    {
        route = OBSTACLE_ROUTE_UNSAFE;
    }
    else if (!p_prof->b_in_path)
    {
        route = p_prof->b_centre_known ? OBSTACLE_ROUTE_CONTINUE :
                                         OBSTACLE_ROUTE_UNSAFE;
    }
    else if (b_left_clear && b_right_clear)
    {
        route = (p_prof->clear_left_cm >= p_prof->clear_right_cm) ?
                OBSTACLE_ROUTE_LEFT : OBSTACLE_ROUTE_RIGHT;
    }
    else if (b_left_clear || b_right_clear)
    {
        route = b_left_clear ? OBSTACLE_ROUTE_LEFT : OBSTACLE_ROUTE_RIGHT;
    }
    else
    {
        route = OBSTACLE_ROUTE_UNSAFE;
    }

    return (route);
}

/*!
 * @brief  Cosine of an integer angle in thousandths (0..180 deg).
 * @param  angle_deg  Angle, degrees (values above 180 are clamped).
 * @return cos(angle) x 1000, within about 2 thousandths.
 */
int32_t
obstacle_profile_cos1000 (uint32_t angle_deg)
{
    uint32_t const angle = (angle_deg > STRAIGHT_DEG) ? STRAIGHT_DEG :
                                                        angle_deg;
    int32_t        result = 0;

    if (angle <= AHEAD_DEG)
    {
        result = sin1000(AHEAD_DEG - angle);      /* cos x = sin(90 - x)  */
    }
    else
    {
        result = -sin1000(angle - AHEAD_DEG);     /* cos x = -sin(x - 90) */
    }

    return (result);
}

/*!
 * @brief  Sine of 0..180 degrees in thousandths (Bhaskara I's formula,
 *         error under 0.2 %).
 * @param  angle_deg  Angle, 0..180 degrees.
 * @return sin(angle) x 1000.
 */
static int32_t
sin1000 (uint32_t angle_deg)
{
    /* angle <= 180, so the products stay far below INT32_MAX. */
    int32_t const angle   = (int32_t) angle_deg;
    int32_t const product = angle * ((int32_t) STRAIGHT_DEG - angle);

    return ((BHASKARA_A * product * SCALE) / (BHASKARA_B - product));
}

/*!
 * @brief  Absolute difference of two angles.
 * @return |angle - ref| in degrees.
 */
static uint32_t
angle_distance (uint32_t angle_deg, uint32_t ref_deg)
{
    return ((angle_deg > ref_deg) ? (angle_deg - ref_deg) :
                                    (ref_deg - angle_deg));
}

/*** end of file ***/

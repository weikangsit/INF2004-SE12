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
#define SIDES                (2U)
#define SIDE_LEFT            (0U)
#define NO_RAY               (0xFFFFFFFFU)

static int32_t  sin1000 (uint32_t angle_deg);
static uint16_t width_measure (obstacle_ray_t const * p_rays, uint32_t count,
                               obstacle_limits_t const * p_limits,
                               obstacle_profile_t const * p_prof);
static uint32_t next_ray (obstacle_ray_t const * p_rays, uint32_t count,
                          uint32_t angle_deg, bool b_left);
static uint32_t range_step (uint32_t range_cm, uint32_t prev_cm);
static uint32_t face_offset_cm (uint32_t range_cm, uint32_t closest_cm);
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

    if (OBSTACLE_NO_ECHO != p_out->closest_cm)
    {
        p_out->width_cm = width_measure(p_rays, count, p_limits, p_out);
    }

    p_out->clear_left_cm  = p_rays[left_i].range_cm;
    p_out->clear_right_cm = p_rays[right_i].range_cm;
}

/*!
 * @brief  Choose a route from a profile.
 *
 *  No valid echo at all, or a free path with no valid ray ahead, cannot be
 *  trusted: UNSAFE. A blocked path checks each side's edge ray (the
 *  sweep's end rays, which pass beside the obstacle's face from the stop
 *  point):
 *   - known and under min_side_clear_cm: that side is blocked;
 *   - known and at least min_side_clear_cm: that side is clear;
 *   - no echo: unknown. Open space beside the car often gives no echo
 *     at all, so unknown is not blocked; the bypass then checks it as it
 *     goes (centre ping on every sideways step, 3-ray check after it).
 *  Both blocked: UNSAFE. Otherwise a clear side beats an unknown one (the
 *  larger clearance if both are clear, left on a tie); with both unknown
 *  the car goes away from the obstacle's closest point.
 *
 * @param  p_prof        Profile of the sweep.
 * @param  p_limits      Planning limits.
 * @return Route.
 */
obstacle_route_t
obstacle_profile_plan (obstacle_profile_t const * p_prof,
                       obstacle_limits_t const * p_limits)
{
    bool const b_left_known    = (OBSTACLE_NO_ECHO != p_prof->clear_left_cm);
    bool const b_right_known   = (OBSTACLE_NO_ECHO != p_prof->clear_right_cm);
    bool const b_left_blocked  = b_left_known &&
                                 (p_prof->clear_left_cm <
                                  p_limits->min_side_clear_cm);
    bool const b_right_blocked = b_right_known &&
                                 (p_prof->clear_right_cm <
                                  p_limits->min_side_clear_cm);
    bool const b_left_clear    = b_left_known && !b_left_blocked;
    bool const b_right_clear   = b_right_known && !b_right_blocked;
    obstacle_route_t route     = OBSTACLE_ROUTE_UNSAFE;

    if (0U == p_prof->valid_rays)
    {
        route = OBSTACLE_ROUTE_UNSAFE;
    }
    else if (!p_prof->b_in_path)
    {
        route = p_prof->b_centre_known ? OBSTACLE_ROUTE_CONTINUE :
                                         OBSTACLE_ROUTE_UNSAFE;
    }
    else if (b_left_blocked && b_right_blocked)
    {
        route = OBSTACLE_ROUTE_UNSAFE;
    }
    else if (b_left_blocked || b_right_blocked)
    {
        route = b_left_blocked ? OBSTACLE_ROUTE_RIGHT : OBSTACLE_ROUTE_LEFT;
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
        /* Both unknown: closest hit left of ahead (> 90 deg) -> right. */
        route = (p_prof->closest_deg > AHEAD_DEG) ? OBSTACLE_ROUTE_RIGHT :
                                                    OBSTACLE_ROUTE_LEFT;
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
    uint32_t const angle  = (angle_deg > STRAIGHT_DEG) ? STRAIGHT_DEG :
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
 * @brief  Width of the obstacle face, from ranges only, not ray angles.
 *
 *  The ~30 deg beam (and strong corner echoes, heard out to the sweep's
 *  end rays) returns the nearest point anywhere in the cone, so
 *  range x cos(ray angle) put a 16 cm box's corners 14.5 cm each side
 *  (measured: read 30 cm). Every point on the face, corners included,
 *  lies sqrt(range^2 - closest^2) along it from the closest point,
 *  whichever ray heard it. Which echoes are the face is found by angle
 *  order: from the closest ray, step to the next ray outwards on each
 *  side while the range changes by at most edge_jump_cm. Along a face it
 *  changes smoothly; past the edge it jumps to the background (a wall,
 *  another object) or to no echo. Counting every echo in range made the
 *  width far too wide, and a fixed depth behind the closest point cut a
 *  wide obstacle's own edges (measured both). The widest offset reached
 *  on each side is summed.
 *
 * @param  p_rays    Rays of the sweep, in any order.
 * @param  count     Number of rays.
 * @param  p_limits  Planning limits.
 * @param  p_prof    Profile with a valid closest_cm and closest_deg.
 * @return Width, cm.
 */
static uint16_t
width_measure (obstacle_ray_t const * p_rays, uint32_t count,
               obstacle_limits_t const * p_limits,
               obstacle_profile_t const * p_prof)
{
    uint32_t width = 0U;
    uint32_t side  = 0U;

    for (side = 0U; side < SIDES; side++)
    {
        bool const b_left = (SIDE_LEFT == side);   /* left = larger angle */
        uint32_t   range  = p_prof->closest_cm;
        uint32_t   half   = 0U;
        uint32_t   next   = next_ray(p_rays, count, p_prof->closest_deg,
                                     b_left);

        /* Each step moves to a strictly larger (smaller) angle, so the
         * walk visits each ray at most once. */
        while ((NO_RAY != next) &&
               (OBSTACLE_NO_ECHO != p_rays[next].range_cm) &&
               (p_rays[next].range_cm < p_limits->scan_range_cm) &&
               (range_step(p_rays[next].range_cm, range) <=
                p_limits->edge_jump_cm))
        {
            uint32_t const off = face_offset_cm(p_rays[next].range_cm,
                                                p_prof->closest_cm);

            half  = (off > half) ? off : half;
            range = p_rays[next].range_cm;
            next  = next_ray(p_rays, count, p_rays[next].angle_deg, b_left);
        }

        width += half;
    }

    /* Each half is under scan_range_cm, so the sum fits. */
    return ((uint16_t) width);
}

/*!
 * @brief  The ray next to an angle, outwards on one side.
 * @param  p_rays     Rays, in any order.
 * @param  count      Number of rays.
 * @param  angle_deg  Angle to step from.
 * @param  b_left     True: the smallest angle above it; false: the
 *                    largest angle below it.
 * @return Index of that ray (the first one if several share the angle),
 *         or NO_RAY.
 */
static uint32_t
next_ray (obstacle_ray_t const * p_rays, uint32_t count, uint32_t angle_deg,
          bool b_left)
{
    uint32_t best = NO_RAY;
    uint32_t idx  = 0U;

    for (idx = 0U; idx < count; idx++)
    {
        uint32_t const angle = p_rays[idx].angle_deg;
        bool const     b_out = b_left ? (angle > angle_deg) :
                                        (angle < angle_deg);

        if (b_out &&
            ((NO_RAY == best) ||
             (b_left ? (angle < p_rays[best].angle_deg) :
                       (angle > p_rays[best].angle_deg))))
        {
            best = idx;
        }
    }

    return (best);
}

/*!
 * @brief  Size of the range change between two neighbouring rays.
 * @param  range_cm  Range of the new ray, cm.
 * @param  prev_cm   Range of the ray before it, cm.
 * @return |range - prev|, cm.
 */
static uint32_t
range_step (uint32_t range_cm, uint32_t prev_cm)
{
    return ((range_cm > prev_cm) ? (range_cm - prev_cm) :
                                   (prev_cm - range_cm));
}

/*!
 * @brief  Distance along the obstacle face from its closest point to a
 *         point heard at range_cm: sqrt(range^2 - closest^2), rounded.
 * @param  range_cm    Range of the point, cm (under the scan range).
 * @param  closest_cm  Closest range of the sweep, cm (<= range_cm).
 * @return Offset along the face, cm.
 */
static uint32_t
face_offset_cm (uint32_t range_cm, uint32_t closest_cm)
{
    /* Ranges are under scan_range_cm (a few hundred cm), so the squares
     * fit easily. */
    uint32_t const diff_sq = (range_cm * range_cm) -
                             (closest_cm * closest_cm);
    uint32_t       root    = 0U;

    /* Nearest integer root: the largest root with root^2 - root <
     * diff_sq. */
    while (((root * root) + root) < diff_sq)
    {
        root++;
    }

    return (root);
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

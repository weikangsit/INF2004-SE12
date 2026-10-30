/** @file    obstacle_profile.h
 *  @brief   Obstacle profile and route planning from one servo sweep
 *           (Buddy 5, Week 6 Section 6.3 PROFILE / PLAN).
 *
 *  Servo angle convention: 0 deg = right, 90 = straight ahead, 180 = left.
 *  Lateral offset x = r * cos(angle), positive to the right.
 *  A ray with no valid echo is unknown, never clear (Tribunal 1).
 *
 *  Pure integer C with no kernel or SDK calls, so web/test_proto.py runs
 *  it in a Cortex-M0+ emulator.
 */
#ifndef OBSTACLE_PROFILE_H
#define OBSTACLE_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#define OBSTACLE_NO_ECHO     (0xFFFFU)  /* range: no valid echo          */

/** One ultrasonic ray. */
typedef struct
{
    uint16_t angle_deg;      /* servo angle, 0..180                       */
    uint16_t range_cm;       /* OBSTACLE_NO_ECHO = invalid                */
} obstacle_ray_t;

/** Planning limits (from car_config.h, passed in to keep this pure). */
typedef struct
{
    uint16_t scan_range_cm;      /* hits beyond this are clear            */
    uint16_t half_corridor_cm;   /* half robot width + margin             */
    uint16_t min_side_clear_cm;  /* free distance needed beside it        */
} obstacle_limits_t;

/** Result of one sweep. */
typedef struct
{
    uint16_t closest_cm;         /* OBSTACLE_NO_ECHO if nothing in range  */
    uint16_t closest_deg;
    uint16_t width_cm;           /* lateral extent of the hits            */
    uint16_t clear_left_cm;      /* left-edge ray; NO_ECHO = unknown      */
    uint16_t clear_right_cm;     /* right-edge ray; NO_ECHO = unknown     */
    uint8_t  valid_rays;
    bool     b_in_path;          /* a hit lies inside the corridor        */
    bool     b_centre_known;     /* a valid ray within 15 deg of ahead    */
} obstacle_profile_t;

/** Planned route (3 bits on the scan result queue). */
typedef enum
{
    OBSTACLE_ROUTE_CONTINUE = 0, /* nothing in the path                   */
    OBSTACLE_ROUTE_LEFT,         /* bypass on the left                    */
    OBSTACLE_ROUTE_RIGHT,        /* bypass on the right                   */
    OBSTACLE_ROUTE_REVERSE,      /* blocked: back off and scan again      */
    OBSTACLE_ROUTE_NO_ROUTE,     /* still blocked after the retry         */
    OBSTACLE_ROUTE_UNSAFE        /* too little valid data to decide       */
} obstacle_route_t;

void             obstacle_profile_build (obstacle_ray_t const * p_rays,
                                         uint32_t count,
                                         obstacle_limits_t const * p_limits,
                                         obstacle_profile_t * p_out);
obstacle_route_t obstacle_profile_plan (obstacle_profile_t const * p_prof,
                                        obstacle_limits_t const * p_limits,
                                        bool b_retry_done);
int32_t          obstacle_profile_cos1000 (uint32_t angle_deg);

#endif /* OBSTACLE_PROFILE_H */

/*** end of file ***/

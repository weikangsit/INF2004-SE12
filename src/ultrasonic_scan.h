/** @file    ultrasonic_scan.h
 *  @brief   Ultrasonic Scan task (Buddy 5): HC-SR04 on the SG90 servo.
 *
 *  STANDBY pings straight ahead every 100 ms and reports an obstacle
 *  nearer than US_OBSTACLE_CM. On request it sweeps SCAN_MIN_DEG to
 *  SCAN_MAX_DEG (coarse, then fine around the closest hit), builds the
 *  obstacle profile and suggests a route (obstacle_profile.h). During a
 *  bypass, PEEK pings SCAN_MIN_DEG, centre and SCAN_MAX_DEG and answers
 *  whether the obstacle is still within BYPASS_PEEK_CM. The scan
 *  never drives the motors; the Navigation FSM acts on the suggestion
 *  (Tribunal 1 interface rule). A missing echo is invalid, never clear.
 */
#ifndef ULTRASONIC_SCAN_H
#define ULTRASONIC_SCAN_H

#include <stdbool.h>
#include <stdint.h>

#include <stddef.h>          /* size_t first (PROHIBIT_DEF_SIZE_T) */
#include <tk/tkernel.h>

#include "obstacle_profile.h"

/** Messages for the Navigation FSM (1 byte, 3 bits). The route results
 *  take their values from obstacle_route_t, so a route converts to a
 *  result unchanged; NEAR is the standby detection; PEEK_* answer
 *  ultrasonic_scan_peek(). */
typedef enum
{
    US_RESULT_CONTINUE = OBSTACLE_ROUTE_CONTINUE,
    US_RESULT_LEFT     = OBSTACLE_ROUTE_LEFT,
    US_RESULT_RIGHT    = OBSTACLE_ROUTE_RIGHT,
    US_RESULT_UNSAFE   = OBSTACLE_ROUTE_UNSAFE,
    US_RESULT_NEAR,
    US_RESULT_PEEK_CLEAR,
    US_RESULT_PEEK_BLOCKED
} us_result_t;

#define US_RANGE_NO_ECHO     (0xFFFFU)

void     ultrasonic_scan_init (void);
void     ultrasonic_scan_task (INT stacd, void * p_exinf);
void     ultrasonic_scan_request (void);
void     ultrasonic_scan_abort (void);
bool     ultrasonic_scan_result_take (us_result_t * p_result);
uint16_t ultrasonic_scan_front_cm (void);
void     ultrasonic_scan_peek (void);
bool     ultrasonic_scan_is_ok (void);

#endif /* ULTRASONIC_SCAN_H */

/*** end of file ***/

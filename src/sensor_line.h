/** @file    sensor_line.h
 *  @brief   Line / Barcode task (Buddy 3): samples the two digital line
 *           sensors every 5 ms and runs the barcode reader (barcode.h).
 *
 *  Tribunal 1 Version C: digital D0 outputs, one bit per sensor. The two
 *  sensors straddle the line:
 *    00 both white  -> centred, drive straight
 *    01 left black  -> line drifting left, steer left
 *    10 right black -> line drifting right, steer right
 *    11 both black  -> junction
 *  Motion Control reads the newest 2-bit reading with sensor_line_read();
 *  a reading older than LINE_MAX_AGE_MS is reported as stale (Tribunal 1:
 *  newest valid reading, timestamped).
 */
#ifndef SENSOR_LINE_H
#define SENSOR_LINE_H

#include <stdbool.h>
#include <stdint.h>

#include <stddef.h>          /* size_t first (PROHIBIT_DEF_SIZE_T) */
#include <tk/tkernel.h>

/* Event bits for the Navigation FSM (line_evt_flg). */
#define SENSOR_LINE_EVT_JUNCTION     (1U << 0)
#define SENSOR_LINE_EVT_REGAINED     (1U << 1)
#define SENSOR_LINE_EVT_SENSOR_FAIL  (1U << 2)

void     sensor_line_init (void);
void     sensor_line_task (INT stacd, void * p_exinf);
bool     sensor_line_read (uint8_t * p_bits);
uint32_t sensor_line_black_age_ms (void);
uint32_t sensor_line_events_take (void);
void     sensor_line_request_calibration (void);
bool     sensor_line_is_ready (void);

#endif /* SENSOR_LINE_H */

/*** end of file ***/

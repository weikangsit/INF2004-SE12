/** @file    imu_terrain.h
 *  @brief   IMU Terrain task (Buddy 4): GY-511 (LSM303DLHC) on I2C0.
 *
 *  Publishes pitch, roll, heading, turn rate, hump peak, motion class, a
 *  valid flag, a calibration flag and a time stamp in the shared state
 *  record (consumers ignore data older than IMU_MAX_AGE_MS, Tribunal 1).
 *  Hump start / end go to the Navigation FSM as 1-byte events; collision
 *  and over-tilt go to the Safety Monitor.
 */
#ifndef IMU_TERRAIN_H
#define IMU_TERRAIN_H

#include <stdbool.h>
#include <stdint.h>

#include <stddef.h>          /* size_t first (PROHIBIT_DEF_SIZE_T) */
#include <tk/tkernel.h>

/** Terrain events for the Navigation FSM (1 byte each). */
typedef enum
{
    IMU_EVT_HUMP_START = 0,
    IMU_EVT_HUMP_END
} imu_terrain_event_t;

void imu_terrain_init (void);
void imu_terrain_task (INT stacd, void * p_exinf);
bool imu_terrain_is_ok (void);
bool imu_terrain_event_take (imu_terrain_event_t * p_event);
void imu_terrain_mag_cal_start (void);
void imu_terrain_mag_cal_finish (void);

#endif /* IMU_TERRAIN_H */

/*** end of file ***/

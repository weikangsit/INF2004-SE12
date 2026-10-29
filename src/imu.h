/**
 * @file    imu.h
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

bool imu_is_ok(void);
void imu_calibrate_mag(uint32_t duration_ms);

#endif /* IMU_H */

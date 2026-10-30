/** @file    motor_driver.h
 *  @brief   Motor driver (Buddy 2): Robo Pico motor PWM, the two quadrature
 *           wheel encoders, and the Safety motor-inhibit latch.
 *
 *  SDK side (hardware registers, no kernel calls; see port.h). Only Motion
 *  Control drives the motors; the Safety Monitor can only brake and
 *  inhibit them (Tribunal 1 shared-resource matrix, GP8-GP11).
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/* Duty is in per mille: -1000..+1000, positive = that wheel forward.   */
#define MOTOR_DUTY_MAX_PM    (1000)

void     motor_driver_init (void);
void     motor_driver_set (int32_t duty_l_pm, int32_t duty_r_pm);
void     motor_driver_brake (void);
void     motor_driver_inhibit (bool b_inhibit);
bool     motor_driver_is_inhibited (void);

int32_t  motor_driver_count_left (void);
int32_t  motor_driver_count_right (void);
uint32_t motor_driver_edge_ms_left (void);
uint32_t motor_driver_edge_ms_right (void);

/* Called only by the shared GPIO interrupt dispatcher (hal.c). */
void     motor_driver_encoder_isr (uint32_t now_ms);

#endif /* MOTOR_DRIVER_H */

/*** end of file ***/

/** @file    port.h
 *  @brief   Plain-C bridge between the kernel side and the SDK side (team).
 *
 *  The template cannot mix Pico SDK hardware headers and <tk/tkernel.h> in
 *  one file (both define size_t and the register base macros), so the car
 *  code is split the same way as the template's own WiFi glue:
 *    - SDK side    : hal.c, motor_driver.c, i2c_drv.c, port.c
 *    - kernel side : every task file
 *  This header only declares functions and a type, so both sides may
 *  include it.
 */
#ifndef PORT_H
#define PORT_H

#include <stdbool.h>
#include <stdint.h>

/* Barr-C 5.4: single-precision floating-point type name. */
typedef float float32_t;

/* 1 MHz system timer. time_us_64() is provided by the template
 * (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c).              */
uint64_t time_us_64 (void);
uint32_t port_time_us_32 (void);
void     port_busy_wait_us (uint32_t delay_us);
bool     port_gpio_get (uint32_t pin);

#endif /* PORT_H */

/*** end of file ***/

/**
 * @file    port.h
 * @brief   Plain-C bridge between the kernel side and the SDK side.
 *
 *  The template cannot mix Pico SDK hardware headers and <tk/tkernel.h>
 *  in one file (both define size_t and the register base macros), so the
 *  car code is split the same way the template's own WiFi glue is:
 *    - SDK side    : hal.c, i2c_drv.c  (hardware/... headers, no kernel)
 *    - kernel side : every task file   (tk/tkernel.h, no hardware/...)
 *  This header only declares functions, so both sides may include it.
 */
#ifndef PORT_H
#define PORT_H

#include <stdint.h>
#include <stdbool.h>

/* Barr-C 5.4: single-precision floating point type name. */
typedef float float32_t;

/* 1 MHz system timer. time_us_64() is provided by the template
 * (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c).            */
uint64_t time_us_64(void);
uint32_t port_time_us_32(void);

/* Spin for `us` microseconds - only for very short waits. */
void     port_busy_wait_us(uint32_t us);

/* Level of a GPIO input. */
bool     port_gpio_get(uint32_t pin);

#endif /* PORT_H */

/**
 * @file    i2c_drv.h
 * @brief   Minimal blocking I2C0 master for the GY-511 (register level).
 */
#ifndef I2C_DRV_H
#define I2C_DRV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Configure I2C0 on the given pins at `baud` Hz (100k / 400k). */
void i2c_drv_init(uint32_t pin_sda, uint32_t pin_scl, uint32_t baud);

/* Write `len` bytes; nostop = repeated start follows. true on ACK.   */
bool i2c_drv_write(uint8_t addr, const uint8_t *p_src, size_t len, bool nostop,
                   uint32_t timeout_us);

/* Read `len` bytes after a write with nostop = true. true on success. */
bool i2c_drv_read(uint8_t addr, uint8_t *p_dst, size_t len, uint32_t timeout_us);

#endif /* I2C_DRV_H */

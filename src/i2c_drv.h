/** @file    i2c_drv.h
 *  @brief   Minimal blocking I2C0 master for the GY-511 (team, SDK side).
 */
#ifndef I2C_DRV_H
#define I2C_DRV_H

#include <stdbool.h>
#include <stdint.h>

void i2c_drv_init (uint32_t pin_sda, uint32_t pin_scl, uint32_t baud);
bool i2c_drv_write (uint8_t addr, uint8_t const * p_src, uint32_t len,
                    bool b_nostop, uint32_t timeout_us);
bool i2c_drv_read (uint8_t addr, uint8_t * p_dst, uint32_t len,
                   uint32_t timeout_us);

#endif /* I2C_DRV_H */

/*** end of file ***/

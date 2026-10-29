/**
 * @file    i2c_drv.c
 * @brief   Minimal blocking I2C0 master (DW_apb_i2c on the RP2040).
 *
 *  Follows the sequence of the Pico SDK's hardware_i2c driver, with the
 *  timeout done on the 1 MHz system timer so it needs nothing beyond the
 *  register headers the template already provides.
 */
#include "hardware/gpio.h"
#include "hardware/resets.h"
#include "hardware/structs/i2c.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"

#include "port.h"
#include "i2c_drv.h"

#define I2C_HW   (i2c0_hw)

static bool g_restart_next;

void i2c_drv_init(uint32_t pin_sda, uint32_t pin_scl, uint32_t baud)
{
    uint32_t freq = clock_get_hz(clk_sys);
    uint32_t period = (freq + (baud / 2U)) / baud;
    uint32_t lcnt = (period * 3U) / 5U;
    uint32_t hcnt = period - lcnt;

    reset_block_num(RESET_I2C0);
    unreset_block_num_wait_blocking(RESET_I2C0);

    I2C_HW->enable = 0U;
    I2C_HW->con = (I2C_IC_CON_SPEED_VALUE_FAST << I2C_IC_CON_SPEED_LSB) |
                  I2C_IC_CON_MASTER_MODE_BITS | I2C_IC_CON_IC_SLAVE_DISABLE_BITS |
                  I2C_IC_CON_IC_RESTART_EN_BITS | I2C_IC_CON_TX_EMPTY_CTRL_BITS;
    I2C_HW->tx_tl = 0U;
    I2C_HW->rx_tl = 0U;
    I2C_HW->fs_scl_hcnt = hcnt;
    I2C_HW->fs_scl_lcnt = lcnt;
    I2C_HW->fs_spklen = (lcnt < 16U) ? 1U : (lcnt / 16U);
    hw_write_masked(&I2C_HW->sda_hold,
                    (((freq * 3U) / 10000000U) + 1U) << I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_LSB,
                    I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_BITS);
    I2C_HW->enable = 1U;
    g_restart_next = false;

    gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_sda);
    gpio_pull_up(pin_scl);
}

static void set_target(uint8_t addr)
{
    I2C_HW->enable = 0U;
    I2C_HW->tar = addr;
    I2C_HW->enable = 1U;
}

static bool expired(uint32_t t0, uint32_t timeout_us)
{
    return (time_us_32() - t0) > timeout_us;
}

bool i2c_drv_write(uint8_t addr, const uint8_t *p_src, size_t len, bool nostop,
                   uint32_t timeout_us)
{
    uint32_t t0 = time_us_32();
    bool     ok = true;
    size_t   i;

    set_target(addr);
    for (i = 0U; (i < len) && ok; i++)
    {
        bool first = (0U == i);
        bool last = ((len - 1U) == i);

        I2C_HW->data_cmd = ((first && g_restart_next) ? I2C_IC_DATA_CMD_RESTART_BITS : 0U) |
                           ((last && !nostop) ? I2C_IC_DATA_CMD_STOP_BITS : 0U) |
                           p_src[i];
        while (0U == (I2C_HW->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS))
        {
            if (expired(t0, timeout_us)) { ok = false; break; }
        }
        if (ok && (0U != I2C_HW->tx_abrt_source))
        {
            (void)I2C_HW->clr_tx_abrt;           /* NACK: read clears it */
            ok = false;
        }
        if (!ok || (last && !nostop))
        {
            while (0U == (I2C_HW->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS))
            {
                if (expired(t0, timeout_us)) { ok = false; break; }
            }
            (void)I2C_HW->clr_stop_det;
        }
    }
    g_restart_next = nostop && ok;
    return ok;
}

bool i2c_drv_read(uint8_t addr, uint8_t *p_dst, size_t len, uint32_t timeout_us)
{
    uint32_t t0 = time_us_32();
    size_t   i;

    set_target(addr);
    for (i = 0U; i < len; i++)
    {
        bool first = (0U == i);
        bool last = ((len - 1U) == i);

        while (I2C_HW->txflr >= 16U)                  /* TX FIFO full */
        {
            if (expired(t0, timeout_us)) { g_restart_next = false; return false; }
        }
        I2C_HW->data_cmd = ((first && g_restart_next) ? I2C_IC_DATA_CMD_RESTART_BITS : 0U) |
                           (last ? I2C_IC_DATA_CMD_STOP_BITS : 0U) |
                           I2C_IC_DATA_CMD_CMD_BITS;            /* read */
        while (0U == I2C_HW->rxflr)
        {
            if (0U != (I2C_HW->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS))
            {
                (void)I2C_HW->clr_tx_abrt;
                g_restart_next = false;
                return false;
            }
            if (expired(t0, timeout_us)) { g_restart_next = false; return false; }
        }
        p_dst[i] = (uint8_t)I2C_HW->data_cmd;
    }
    g_restart_next = false;
    return true;
}

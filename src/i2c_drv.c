/** @file    i2c_drv.c
 *  @brief   Minimal blocking I2C0 master (DW_apb_i2c on the RP2040), see
 *           i2c_drv.h.
 *
 *  Follows the sequence of the Pico SDK's hardware_i2c driver (RP2040
 *  datasheet section 4.3), with timeouts on the 1 MHz system timer so it
 *  needs only the register headers the template already provides.
 */
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/resets.h"
#include "hardware/structs/i2c.h"
#include "hardware/timer.h"

#include "i2c_drv.h"

#define I2C_HW               (i2c0_hw)
#define LOW_PERIOD_NUM       (3U)        /* SCL low = 3/5 of the period   */
#define LOW_PERIOD_DEN       (5U)
#define SPIKE_DIV            (16U)
#define HOLD_NS_DIV          (10000000U) /* 300 ns SDA hold at sys clock  */
#define HOLD_NS_MUL          (3U)
#define TX_FIFO_DEPTH        (16U)

static bool g_b_restart_next = false;

static void set_target (uint8_t addr);
static bool expired (uint32_t start_us, uint32_t timeout_us);

/*!
 * @brief  Configure I2C0 on the given pins.
 * @param  pin_sda  SDA pin.
 * @param  pin_scl  SCL pin.
 * @param  baud     Bus speed, Hz (100 k or 400 k).
 */
void
i2c_drv_init (uint32_t pin_sda, uint32_t pin_scl, uint32_t baud)
{
    uint32_t const freq   = clock_get_hz(clk_sys);
    uint32_t const period = (freq + (baud / 2U)) / baud;
    uint32_t const lcnt   = (period * LOW_PERIOD_NUM) / LOW_PERIOD_DEN;
    uint32_t const hcnt   = period - lcnt;

    reset_block_num(RESET_I2C0);
    unreset_block_num_wait_blocking(RESET_I2C0);

    I2C_HW->enable      = 0U;
    I2C_HW->con         = (I2C_IC_CON_SPEED_VALUE_FAST <<
                           I2C_IC_CON_SPEED_LSB) |
                          I2C_IC_CON_MASTER_MODE_BITS |
                          I2C_IC_CON_IC_SLAVE_DISABLE_BITS |
                          I2C_IC_CON_IC_RESTART_EN_BITS |
                          I2C_IC_CON_TX_EMPTY_CTRL_BITS;
    I2C_HW->tx_tl       = 0U;
    I2C_HW->rx_tl       = 0U;
    I2C_HW->fs_scl_hcnt = hcnt;
    I2C_HW->fs_scl_lcnt = lcnt;
    I2C_HW->fs_spklen   = (lcnt < SPIKE_DIV) ? 1U : (lcnt / SPIKE_DIV);
    hw_write_masked(&I2C_HW->sda_hold,
                    (((freq * HOLD_NS_MUL) / HOLD_NS_DIV) + 1U) <<
                    I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_LSB,
                    I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_BITS);
    I2C_HW->enable      = 1U;
    g_b_restart_next    = false;

    gpio_set_function(pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(pin_sda);
    gpio_pull_up(pin_scl);
}

/*!
 * @brief  Write bytes to a device.
 * @param  addr        7-bit address.
 * @param  p_src       Bytes to send.
 * @param  len         Number of bytes (at least 1).
 * @param  b_nostop    True if a repeated start (read) follows.
 * @param  timeout_us  Longest time for the whole transfer.
 * @return True if every byte was acknowledged in time.
 */
bool
i2c_drv_write (uint8_t addr, uint8_t const * p_src, uint32_t len,
               bool b_nostop, uint32_t timeout_us)
{
    uint32_t const start = time_us_32();
    bool           b_ok  = true;
    uint32_t       idx   = 0U;

    set_target(addr);

    for (idx = 0U; (idx < len) && b_ok; idx++)
    {
        bool const b_first = (0U == idx);
        bool const b_last  = ((len - 1U) == idx);

        I2C_HW->data_cmd = ((b_first && g_b_restart_next) ?
                            I2C_IC_DATA_CMD_RESTART_BITS : 0U) |
                           ((b_last && (!b_nostop)) ?
                            I2C_IC_DATA_CMD_STOP_BITS : 0U) |
                           (uint32_t) p_src[idx];

        while (0U == (I2C_HW->raw_intr_stat &
                      I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS))
        {
            if (expired(start, timeout_us))
            {
                b_ok = false;
                break;
            }
        }

        if (b_ok && (0U != I2C_HW->tx_abrt_source))
        {
            (void) I2C_HW->clr_tx_abrt;           /* NACK: read clears it */
            b_ok = false;
        }

        if ((!b_ok) || (b_last && (!b_nostop)))
        {
            while (0U == (I2C_HW->raw_intr_stat &
                          I2C_IC_RAW_INTR_STAT_STOP_DET_BITS))
            {
                if (expired(start, timeout_us))
                {
                    b_ok = false;
                    break;
                }
            }

            (void) I2C_HW->clr_stop_det;
        }
    }

    g_b_restart_next = b_nostop && b_ok;

    return (b_ok);
}

/*!
 * @brief  Read bytes from a device (after a write with b_nostop = true).
 * @param  addr        7-bit address.
 * @param  p_dst       Output bytes.
 * @param  len         Number of bytes (at least 1).
 * @param  timeout_us  Longest time for the whole transfer.
 * @return True on success.
 */
bool
i2c_drv_read (uint8_t addr, uint8_t * p_dst, uint32_t len,
              uint32_t timeout_us)
{
    uint32_t const start = time_us_32();
    bool           b_ok  = true;
    uint32_t       idx   = 0U;

    set_target(addr);

    for (idx = 0U; (idx < len) && b_ok; idx++)
    {
        bool const b_first = (0U == idx);
        bool const b_last  = ((len - 1U) == idx);

        while (b_ok && (I2C_HW->txflr >= TX_FIFO_DEPTH))
        {
            b_ok = !expired(start, timeout_us);
        }

        if (b_ok)
        {
            I2C_HW->data_cmd = ((b_first && g_b_restart_next) ?
                                I2C_IC_DATA_CMD_RESTART_BITS : 0U) |
                               (b_last ? I2C_IC_DATA_CMD_STOP_BITS : 0U) |
                               I2C_IC_DATA_CMD_CMD_BITS;      /* read     */
        }

        while (b_ok && (0U == I2C_HW->rxflr))
        {
            if (0U != (I2C_HW->raw_intr_stat &
                       I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS))
            {
                (void) I2C_HW->clr_tx_abrt;
                b_ok = false;
            }
            else
            {
                b_ok = !expired(start, timeout_us);
            }
        }

        if (b_ok)
        {
            /* The low 8 bits of data_cmd hold the received byte. */
            p_dst[idx] = (uint8_t) I2C_HW->data_cmd;
        }
    }

    g_b_restart_next = false;

    return (b_ok);
}

/*!
 * @brief  Select the target address (the block must be disabled).
 * @param  addr  7-bit address.
 */
static void
set_target (uint8_t addr)
{
    I2C_HW->enable = 0U;
    I2C_HW->tar    = addr;
    I2C_HW->enable = 1U;
}

/*!
 * @brief  Check a timeout on the 1 MHz system timer.
 * @param  start_us    Start time.
 * @param  timeout_us  Allowed time.
 * @return True once more than timeout_us has passed.
 */
static bool
expired (uint32_t start_us, uint32_t timeout_us)
{
    return ((time_us_32() - start_us) > timeout_us);
}

/*** end of file ***/

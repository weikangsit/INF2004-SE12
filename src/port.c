/** @file    port.c
 *  @brief   SDK-side services for kernel-side code (see port.h).
 */
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "port.h"

/*!
 * @brief  Low 32 bits of the 1 MHz system timer.
 * @return Microseconds (wraps after 71 minutes).
 */
uint32_t
port_time_us_32 (void)
{
    return (time_us_32());
}

/*!
 * @brief  Busy-wait; only for very short delays (e.g. a 10 us trigger).
 * @param  delay_us  Delay in microseconds.
 */
void
port_busy_wait_us (uint32_t delay_us)
{
    uint32_t const start = time_us_32();

    while ((time_us_32() - start) < delay_us)
    {
        /* Spin until the delay has passed. */
    }
}

/*!
 * @brief  Level of a GPIO input.
 * @param  pin  GPIO number.
 * @return True if the pin reads high.
 */
bool
port_gpio_get (uint32_t pin)
{
    return (gpio_get(pin));
}

/*** end of file ***/

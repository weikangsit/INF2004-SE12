/** @file    systime.c
 *  @brief   System time helpers (see systime.h).
 *
 *  All times come from the RP2040 1 MHz system timer (time_us_64(), see
 *  port.h), so every task and ISR uses the same clock.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <stddef.h>          /* size_t first (PROHIBIT_DEF_SIZE_T) */
#include <tk/tkernel.h>

#include "port.h"
#include "systime.h"

#define US_PER_MS            (1000U)

/*!
 * @brief  Milliseconds since boot (wraps after 49 days).
 * @return Current time in ms.
 */
uint32_t
systime_now_ms (void)
{
    /* Deliberate truncation to 32 bits: callers only use differences. */
    return ((uint32_t) (time_us_64() / US_PER_MS));
}

/*!
 * @brief  Fixed-rate wait: the next deadline is the previous one plus the
 *         period, so the period does not drift. An overrun resynchronises
 *         to now and still yields the processor.
 * @param  p_next_us  Deadline of the previous period (updated).
 * @param  period_ms  Period of the task.
 */
void
systime_period_wait (uint64_t * p_next_us, uint32_t period_ms)
{
    uint64_t const now = time_us_64();

    *p_next_us += (uint64_t) period_ms * US_PER_MS;

    if (*p_next_us <= now)
    {
        *p_next_us = now;                         /* overrun: resync        */
        (void) tk_dly_tsk(0U);
    }
    else
    {
        /* Rounded up to whole ms; at most period_ms, so it fits. */
        uint64_t const wait_ms = ((*p_next_us - now) + US_PER_MS - 1U) /
                                 US_PER_MS;

        (void) tk_dly_tsk((RELTIM) wait_ms);
    }
}

/*!
 * @brief  A 16-bit millisecond time stamp, for data whose age is checked
 *         over short intervals (under 65 s).
 * @return Low 16 bits of the millisecond clock.
 */
uint16_t
systime_stamp (void)
{
    /* Deliberate truncation: only differences of stamps are used. */
    return ((uint16_t) systime_now_ms());
}

/*!
 * @brief  Age of a systime_stamp() value.
 * @param  stamp  Earlier stamp.
 * @return Milliseconds since the stamp, modulo 65536.
 */
uint32_t
systime_stamp_age (uint16_t stamp)
{
    /* Modular 16-bit difference, widened for unsigned comparisons. */
    return ((uint32_t) (uint16_t) (systime_stamp() - stamp));
}

/*** end of file ***/

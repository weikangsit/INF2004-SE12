/** @file    stub_kernel.h  (TEST STUB)
 *  @brief   What the tests can see and set in the fake kernel and clock.
 */
#ifndef STUB_KERNEL_H
#define STUB_KERNEL_H

#include <stdbool.h>
#include <stdint.h>

/** Forget every flag, message buffer, log line and recorded call. */
void stub_kernel_reset (void);

/** Fake 1 MHz clock behind time_us_64(). */
void stub_time_set_ms (uint32_t ms);
void stub_time_advance_ms (uint32_t ms);

/** Fake GPIO levels behind port_gpio_get(). */
extern bool stub_gpio[32];

/** Number of flag or message-buffer calls made with a wait time other
 *  than TMO_POL (the code is meant never to wait). */
extern int stub_nonpoll_calls;

/** True if tm_printf() has printed text containing the given string. */
bool stub_log_contains (char const * p_text);

#endif /* STUB_KERNEL_H */

/** @file    hal_irq.h
 *  @brief   Kernel side of the shared GPIO interrupt (team): registers the
 *           IO_IRQ_BANK0 handler with micro T-Kernel and turns its
 *           signals into an event flag the tasks can wait on.
 */
#ifndef HAL_IRQ_H
#define HAL_IRQ_H

#include <stdbool.h>
#include <stdint.h>

void hal_isr_init (void);
void hal_echo_arm (void);
bool hal_echo_wait (uint32_t timeout_ms);

#endif /* HAL_IRQ_H */

/*** end of file ***/

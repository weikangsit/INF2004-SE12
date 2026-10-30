/** @file    systime.h
 *  @brief   System time helpers for the tasks: millisecond clock, drift-free
 *           periodic wait and modular 16-bit time stamps.
 */
#ifndef SYSTIME_H
#define SYSTIME_H

#include <stdint.h>

uint32_t systime_now_ms (void);
void     systime_period_wait (uint64_t * p_next_us, uint32_t period_ms);
uint16_t systime_stamp (void);
uint32_t systime_stamp_age (uint16_t stamp);

#endif /* SYSTIME_H */

/*** end of file ***/

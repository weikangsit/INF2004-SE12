/** @file    safety.h
 *  @brief   Safety Monitor task (team, priority 1): FR7, NFR3.
 *
 *  Fault events (collision, over-tilt, line lost, e-stop) wake the task
 *  at once; it also checks every supervised task's heartbeat against its
 *  own deadline (Week 6 feedback). On any fault it inhibits and brakes the
 *  motors directly, latches, stops Motion Control and reports the fault.
 */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include <stddef.h>          /* size_t first (PROHIBIT_DEF_SIZE_T) */
#include <tk/tkernel.h>

/* Heartbeat bits, one per supervised task (also the EV_FAULT detail of a
 * missed heartbeat, see web/static/proto.js HB_TASKS). */
#define SAFETY_HB_LINE       (1U << 0)
#define SAFETY_HB_MOTION     (1U << 1)
#define SAFETY_HB_IMU        (1U << 2)
#define SAFETY_HB_US         (1U << 3)
#define SAFETY_HB_NAV        (1U << 4)
#define SAFETY_HB_COMMS      (1U << 5)   /* from the radio poll counter  */
#define SAFETY_HB_TELEM      (1U << 6)

/* Fault events that wake the task. */
#define SAFETY_EVT_COLLISION (1U << 0)
#define SAFETY_EVT_OVERTILT  (1U << 1)
#define SAFETY_EVT_LINE_LOST (1U << 2)
#define SAFETY_EVT_ESTOP     (1U << 3)

void safety_init (void);
void safety_task (INT stacd, void * p_exinf);
void safety_heartbeat (uint32_t task_bit);
void safety_report (uint32_t events);
bool safety_is_latched (void);
void safety_clear (void);

#endif /* SAFETY_H */

/*** end of file ***/

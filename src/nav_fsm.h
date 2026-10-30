/** @file    nav_fsm.h
 *  @brief   Navigation FSM task (team): system-level statechart, Week 6
 *           Section 5, updated by Tribunal 1.
 */
#ifndef NAV_FSM_H
#define NAV_FSM_H

#include <stdint.h>

#include <stddef.h>          /* size_t first (PROHIBIT_DEF_SIZE_T) */
#include <tk/tkernel.h>

/** System states (4 bits in the shared record and telemetry). */
typedef enum
{
    NAV_INIT = 0,
    NAV_READY,
    NAV_LINE_FOLLOW,
    NAV_BARCODE_ACT,
    NAV_HUMP_REPORT,
    NAV_OBSTACLE_AVOID,
    NAV_LINE_REACQUIRE,
    NAV_MISSION_DONE,
    NAV_FAULT,
    NAV_STATE_COUNT
} nav_fsm_state_t;

void nav_fsm_init (void);
void nav_fsm_task (INT stacd, void * p_exinf);
void nav_fsm_post_remote (uint8_t cmd);

#endif /* NAV_FSM_H */

/*** end of file ***/

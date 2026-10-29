/**
 * @file    nav_fsm.h
 */
#ifndef NAV_FSM_H
#define NAV_FSM_H

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
} nav_state_t;

#endif /* NAV_FSM_H */

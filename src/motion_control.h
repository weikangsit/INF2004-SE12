/** @file    motion_control.h
 *  @brief   Motion Control task (Buddy 2): turns movement commands into
 *           motor outputs (Week 6 Section 6.2, Tribunal 1 interface map).
 *
 *  Commands go through motion_cmd_q as one packed 32-bit word (id, 3-bit
 *  sequence number, flags, speed cm/s, distance cm, angle deg). Every
 *  command except STOP reports completion on motion_done_q with its
 *  sequence number and a result (Tribunal 1 Version C), so the Navigation
 *  FSM can reject a late completion of an older command.
 *
 *  NOTE: Only the Navigation FSM issues commands other than STOP, so the
 *  sequence counter has a single writer. STOP (sequence 0) may also come
 *  from the Safety Monitor and never reports completion.
 */
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include <stddef.h>          /* size_t first (PROHIBIT_DEF_SIZE_T) */
#include <tk/tkernel.h>

/** Result of a finished command (2 bits on motion_done_q). */
typedef enum
{
    MOTION_RESULT_DONE = 0,     /* target reached                         */
    MOTION_RESULT_TIMEOUT,      /* command time limit expired             */
    MOTION_RESULT_FAULT         /* stall or failed self-test              */
} motion_result_t;

/** One completion report (unpacked from its 1-byte message). */
typedef struct
{
    uint8_t         seq;        /* 1..7, as returned when it was issued   */
    motion_result_t result;
} motion_done_t;

void    motion_control_init (void);
void    motion_control_task (INT stacd, void * p_exinf);

uint8_t motion_control_forward (uint32_t speed_cms, uint32_t dist_cm,
                                bool b_follow_line);
uint8_t motion_control_reverse (uint32_t speed_cms, uint32_t dist_cm,
                                bool b_follow_line);
uint8_t motion_control_turn (bool b_left, uint32_t speed_cms,
                             uint32_t angle_deg, bool b_end_on_line);
uint8_t motion_control_uturn (void);
uint8_t motion_control_self_test (void);
uint8_t motion_control_motor_test (void);
void    motion_control_stop (void);

bool    motion_control_done_take (motion_done_t * p_done);
bool    motion_control_is_open_loop (void);

#endif /* MOTION_CONTROL_H */

/*** end of file ***/

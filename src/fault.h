/** @file    fault.h
 *  @brief   Shared fault bitmask: one bit per fault condition.
 *
 *  Change Requirement 1 (minimum-bit data representation): every fault is
 *  one bit of a single shared 16-bit mask instead of a separate variable.
 *  The mask lives in a micro T-Kernel event flag, so each set or clear is
 *  one atomic kernel call. That keeps the read-modify-write safe between
 *  tasks on both RP2040 cores and from interrupt handlers.
 */
#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>

/** Fault bit numbers. The value of each code is its bit position in the
 *  mask, and the same 4-bit code is sent in the EVENT frame (proto.h). */
typedef enum
{
    FC_STALL = 0,       /* motion: no encoder edges while driving         */
    FC_ENCODER,         /* motion self-test: an encoder stayed silent     */
    FC_IMU,             /* IMU: I2C reads failing                         */
    FC_NO_ECHO,         /* ultrasonic: last front ping had no echo        */
    FC_WIFI_LOSS,       /* comms: MQTT link down (warning, never brakes)  */
    FC_COLLISION,       /* IMU impact                                     */
    FC_OVERTILT,        /* IMU pitch or roll beyond the limit             */
    FC_LINE_LOST,       /* line: line-lost timeout                        */
    FC_ESTOP,           /* remote or local emergency stop                 */
    FC_HEARTBEAT,       /* safety: a supervised task missed its deadline  */
    FC_SENSOR,          /* line: calibration failed                       */
    FC_NO_ROUTE,        /* obstacle: no way around                        */
    FC_REACQUIRE,       /* obstacle: line not found again after a bypass  */
    FC_SELFTEST,        /* nav: power-on self-test timed out              */
    FC_TELEM_LATE,      /* safety: telemetry missed its deadline (warning)*/
    FC_SOFTWARE,        /* nav: impossible state reached                  */
    FC_COUNT
} fault_code_t;

/** Width of the fault mask in bits. */
#define FAULT_MASK_BITS      (16U)

/** Faults that only describe the live state of a sensor or link. They are
 *  refreshed every telemetry period instead of being latched. */
#define FAULT_LIVE_MASK      ((1U << FC_IMU) | (1U << FC_NO_ECHO) | \
                              (1U << FC_WIFI_LOSS))

void     fault_init (void);
void     fault_set (fault_code_t code);
void     fault_clear (fault_code_t code);
void     fault_clear_all (void);
uint16_t fault_get_mask (void);

#endif /* FAULT_H */

/*** end of file ***/

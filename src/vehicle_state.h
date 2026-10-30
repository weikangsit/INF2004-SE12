/** @file    vehicle_state.h
 *  @brief   Shared vehicle state record (Week 6 Section 4: state_mtx, one
 *           writer per field group).
 *
 *  Change Requirement 1: the record is stored packed, each field in only
 *  the bits its range needs (132 bits = 17 bytes, was 52 bytes of floats).
 *  A writer packs its group and a reader unpacks a whole snapshot, both
 *  under the priority-inheritance mutex, so a read-modify-write of the
 *  shared bytes can never interleave with another task's.
 *
 *  Writers: Motion Control (speeds, duties, distance), IMU Terrain
 *  (attitude, heading, hump, class, validity), Line / Barcode (raw line
 *  sensors, last barcode), Navigation FSM (system state).
 */
#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include <stdbool.h>
#include <stdint.h>

/* Bits of ir_black: raw digital line sensors, 1 = black. Same order as
 * the telemetry line field (proto.h PROTO_LINE_*).                      */
#define VEHICLE_IR_LEFT      (1U << 0)
#define VEHICLE_IR_RIGHT     (1U << 1)

/* last_barcode: bits 0..1 slot (= action wire code), bit 2 valid (same
 * as telemetry).                                                        */
#define VEHICLE_BAR_VALID    (1U << 2)
#define VEHICLE_BAR_SLOT     (0x3U)

/** Unpacked snapshot. The comment gives each field's stored width. */
typedef struct
{
    /* Motion Control */
    int16_t  speed_l_mms;        /* mm/s, 12 bits signed                  */
    int16_t  speed_r_mms;        /* mm/s, 12 bits signed                  */
    int8_t   duty_l_pct;         /* percent, 8 bits signed                */
    int8_t   duty_r_pct;         /* percent, 8 bits signed                */
    int32_t  dist_mm;            /* mean wheel travel, 18 bits signed     */
    /* IMU Terrain */
    int16_t  pitch_half_deg;     /* 0.5 deg, 9 bits signed, nose-up +     */
    int16_t  roll_deg;           /* 1 deg, 9 bits signed                  */
    uint16_t heading_half_deg;   /* 0.5 deg, 0..719, 10 bits              */
    int16_t  turn_rate_dps;      /* deg/s, 9 bits signed                  */
    uint16_t hump_peak_half_mm;  /* 0.5 mm, 8 bits                        */
    uint8_t  motion_class;       /* 0 still, 1 moving, 2 turning, 3 hump  */
    bool     b_imu_valid;        /* IMU reads succeeding, 1 bit           */
    bool     b_mag_calibrated;   /* hard-iron offsets measured, 1 bit     */
    uint16_t imu_stamp;          /* systime_stamp() of the IMU fields     */
    /* Line / Barcode */
    uint8_t  ir_black;           /* VEHICLE_IR_*, 2 bits                  */
    uint8_t  last_barcode;       /* VEHICLE_BAR_*, 3 bits                 */
    /* Navigation FSM */
    uint8_t  nav_state;          /* nav_state_t, 4 bits                   */
} vehicle_state_t;

void vehicle_state_init (void);
void vehicle_state_read (vehicle_state_t * p_out);
void vehicle_state_set_motion (int32_t speed_l_mms, int32_t speed_r_mms,
                               int32_t duty_l_pct, int32_t duty_r_pct,
                               int32_t dist_mm);
void vehicle_state_set_imu (vehicle_state_t const * p_imu);
void vehicle_state_set_line (uint8_t ir_black);
void vehicle_state_set_barcode (uint8_t last_barcode);
void vehicle_state_set_nav (uint8_t nav_state);

#endif /* VEHICLE_STATE_H */

/*** end of file ***/

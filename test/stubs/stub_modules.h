/** @file    stub_modules.h  (TEST STUB)
 *  @brief   Fake versions of the firmware modules that a test does not
 *           exercise. They record what the code under test asked them to
 *           do, so a test can check it.
 */
#ifndef STUB_MODULES_H
#define STUB_MODULES_H

#include <stdbool.h>
#include <stdint.h>

#include "vehicle_state.h"

#define STUB_MAX_RECORDS (64)

/** One event given to telemetry_post_event(). */
typedef struct
{
    int     kind;           /* proto_event_t */
    uint8_t code;
    int16_t value1;
    int16_t value2;
} stub_event_t;

extern stub_event_t stub_events[STUB_MAX_RECORDS];
extern int          stub_event_count;

/** Number of recorded events with this kind (and, for EV_FAULT, code). */
int stub_events_of_kind (int kind);

/** Events given to safety_report(). */
extern uint32_t stub_safety_reports[STUB_MAX_RECORDS];
extern int      stub_safety_report_count;

/** Motor inhibit and Motion Control stop. */
extern int  stub_inhibit_calls;
extern bool stub_inhibit_state;
extern int  stub_motion_stop_calls;

/** Last Motion Control command given by the code under test. */
extern int      stub_forward_calls;
extern uint32_t stub_forward_speed_cms;
extern uint32_t stub_forward_dist_cm;
extern int      stub_turn_calls;
extern bool     stub_turn_left;
extern uint32_t stub_turn_deg;

/** Values the stubs return. */
extern bool            stub_radio_alive;
extern bool            stub_mag_calibrated;
extern bool            stub_latched;
extern vehicle_state_t stub_vstate;       /* vehicle_state_read() result */
extern vehicle_state_t stub_imu_out;      /* last vehicle_state_set_imu() */

void stub_modules_reset (void);

#endif /* STUB_MODULES_H */

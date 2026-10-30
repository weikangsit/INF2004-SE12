/** @file    telemetry.h
 *  @brief   Telemetry Publisher task (Buddy 1): packs events and the 10 Hz
 *           telemetry frame and hands them to the MQTT link (comms.h).
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#include <stddef.h>          /* size_t first (PROHIBIT_DEF_SIZE_T) */
#include <tk/tkernel.h>

#include "proto.h"

void telemetry_init (void);
void telemetry_task (INT stacd, void * p_exinf);
void telemetry_post_event (proto_event_t kind, uint8_t code, int16_t value1,
                           int16_t value2);
void telemetry_post_debug (uint8_t const * p_frame, uint32_t len);

#endif /* TELEMETRY_H */

/*** end of file ***/

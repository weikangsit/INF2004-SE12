/** @file    telemetry.h
 *  @brief   Telemetry Publisher task (Buddy 1): packs events and the 10 Hz
 *           telemetry frame and hands them to the MQTT link (comms.h).
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#include <tk/tkernel.h>

#include "proto.h"

void telemetry_task (INT stacd, void * p_exinf);
void telemetry_post_event (proto_event_t kind, uint8_t code, int16_t value1,
                           int16_t value2);

#endif /* TELEMETRY_H */

/*** end of file ***/

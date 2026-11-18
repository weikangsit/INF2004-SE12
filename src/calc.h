/** @file    calc.h
 *  @brief   Small arithmetic helpers shared by several tasks.
 *
 *  No kernel or SDK calls, so web/test_proto.py runs them in a
 *  Cortex-M0+ emulator.
 */
#ifndef CALC_H
#define CALC_H

#include <stdint.h>

#include "port.h"

int32_t calc_round (float32_t value);
int32_t calc_heading_diff_half (uint16_t from_hdg, uint16_t to_hdg);

#endif /* CALC_H */

/*** end of file ***/

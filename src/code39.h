/** @file    code39.h
 *  @brief   Code 39 decoder for one '*X*' symbol, X in A..D (Buddy 3).
 *
 *  Pure C with no kernel or SDK calls, so web/test_proto.py runs it in a
 *  Cortex-M0+ emulator with synthetic edge times.
 */
#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>

#define CODE39_NONE          (0xFFU)   /* no valid symbol                */
#define CODE39_SYMBOL_EDGES  (30U)     /* 29 elements = 30 edges         */

uint8_t code39_decode (uint32_t const * p_edges_us, uint32_t n_edges,
                       uint32_t min_ratio_x10);

#endif /* CODE39_H */

/*** end of file ***/

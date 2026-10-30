/** @file    bar_filter.h
 *  @brief   Glitch filter for barcode edges (Buddy 3).
 *
 *  An IR module without hysteresis chatters when a bar edge passes the
 *  sensor slowly: one black/white change arrives as a burst of edges a
 *  few tens of us apart, and the level read in the interrupt can be
 *  either one. bar_filter_add() builds a clean capture from the raw
 *  edges, one edge at a time:
 *   - a capture starts on a white-to-black edge;
 *   - an edge that does not change the level is ignored;
 *   - an element shorter than the glitch time is removed together with
 *     the edge that started it (the next edge restores the level).
 *  The edge kept for a burst is its last one, so each element is timed
 *  from the moment the level settled.
 *
 *  Pure C with no kernel or SDK calls, so web/test_proto.py runs it in a
 *  Cortex-M0+ emulator with synthetic chattering edges.
 */
#ifndef BAR_FILTER_H
#define BAR_FILTER_H

#include <stdint.h>

#define BAR_FILTER_LEVEL_BIT (1U)   /* bit 0 of an edge: 1 = black after */

uint32_t bar_filter_add (uint32_t * p_edges, uint32_t count,
                         uint32_t capacity, uint32_t edge,
                         uint32_t glitch_us);

#endif /* BAR_FILTER_H */

/*** end of file ***/

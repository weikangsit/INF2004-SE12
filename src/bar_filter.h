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
 *  bar_filter_positions() turns the encoder counts stamped on each kept
 *  edge into the sensor's travel along the car's direction, in the same
 *  edge format, so code39_decode() measures bars in distance and the
 *  car's speed may change during a read. bar_filter_travel() gives the
 *  same travel for any change in the counts (barcode.c ends a moving read
 *  by distance with it). bar_filter_is_done() decides when a read ends.
 *
 *  Pure C with no kernel or SDK calls, so web/test_proto.py runs it in a
 *  Cortex-M0+ emulator with synthetic chattering edges.
 */
#ifndef BAR_FILTER_H
#define BAR_FILTER_H

#include <stdbool.h>
#include <stdint.h>

/** The state of a capture now, for bar_filter_is_done(). */
typedef struct
{
    uint32_t n_edges;        /* edges in the capture (at least 1)         */
    uint32_t idle_us;        /* time since the last edge                  */
    uint32_t moved;          /* travel since the first edge, 1/16 counts
                                (0 when the encoders are not used)        */
    uint32_t quiet;          /* travel since the last edge, 1/16 counts   */
    bool     b_white;        /* level after the last edge is white        */
} bar_end_state_t;

/** End-of-read limits (car_config.h values), for bar_filter_is_done(). */
typedef struct
{
    uint32_t moved_min;      /* travel that counts as driving, 1/16 counts */
    uint32_t quiet_end;      /* white travel that ends a driving read      */
    uint32_t stall_us;       /* driving: no edge for this long ends it     */
    uint32_t edges_min;      /* hand swipe: edges before the quiet time    */
    uint32_t quiet_us;       /* hand swipe: white time that ends it        */
    uint32_t idle_us;        /* hand swipe: no edge for this long ends it  */
} bar_end_limits_t;

#define BAR_FILTER_LEVEL_BIT (1U)   /* bit 0 of an edge: 1 = black after */

uint32_t bar_filter_add (uint32_t * p_edges, uint32_t count,
                         uint32_t capacity, uint32_t edge,
                         uint32_t glitch_us);
uint32_t bar_filter_positions (uint32_t const * p_edges,
                               int32_t const * p_cl, int32_t const * p_cr,
                               uint32_t count, int32_t const * p_setup,
                               uint32_t * p_out);
int32_t  bar_filter_travel (int32_t delta_l, int32_t delta_r,
                            int32_t const * p_setup);
bool     bar_filter_is_done (bar_end_state_t const * p_state,
                             bar_end_limits_t const * p_limits);

#endif /* BAR_FILTER_H */

/*** end of file ***/

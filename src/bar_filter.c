/** @file    bar_filter.c
 *  @brief   Glitch filter for barcode edges (see bar_filter.h).
 */
#include "bar_filter.h"

#define TIME_MASK            (~BAR_FILTER_LEVEL_BIT)
#define SUB_COUNTS           (16)      /* positions in 1/16 count        */
#define HALF_SUB             (8)
#define YAW_SCALE            (1000)
#define SETUP_SIGN_L         (0U)
#define SETUP_SIGN_R         (1U)
#define SETUP_YAW            (2U)

static int32_t sensor_travel (int32_t const * p_cl, int32_t const * p_cr,
                              uint32_t idx, int32_t const * p_setup);

/*!
 * @brief  Add one raw edge to a capture.
 * @param  p_edges    Capture: edge times in us, bit 0 = level after the
 *                    edge (1 = black). Holds count edges.
 * @param  count      Edges in the capture; 0 = waiting for a start.
 * @param  capacity   Size of p_edges (at least 1).
 * @param  edge       New raw edge, same format.
 * @param  glitch_us  Elements shorter than this are glitches.
 * @return New number of edges. 0 means waiting for a start (also when a
 *         glitch removed the start edge). A full capture keeps its size
 *         and drops the edge.
 */
uint32_t
bar_filter_add (uint32_t * p_edges, uint32_t count, uint32_t capacity,
                uint32_t edge, uint32_t glitch_us)
{
    uint32_t const level  = edge & BAR_FILTER_LEVEL_BIT;
    uint32_t       result = count;

    if (0U == count)
    {
        if ((0U != level) && (capacity > 0U))     /* white -> black       */
        {
            p_edges[0] = edge;
            result     = 1U;
        }
    }
    else
    {
        uint32_t const last = p_edges[count - 1U];

        if (level == (last & BAR_FILTER_LEVEL_BIT))
        {
            /* No level change (a burst ended on the same level): ignore. */
        }
        else if (((edge & TIME_MASK) - (last & TIME_MASK)) < glitch_us)
        {
            /* The element that began at the last edge is a glitch: drop
             * that edge; this edge restores the level before it. */
            result = count - 1U;
        }
        else if (count < capacity)
        {
            p_edges[count] = edge;
            result         = count + 1U;
        }
        else
        {
            /* Full: drop the edge (the capture is past a full symbol). */
        }
    }

    return (result);
}

/*!
 * @brief  Positions of a capture's edges along the car's direction of
 *         travel, from the encoder counts stamped on each edge.
 *
 *  The sensor's travel is the mean wheel travel minus the turning
 *  correction for a sensor off the centreline:
 *      travel = (dl + dr) / 2 - y * (dr - dl) / track
 *  with y the sensor offset to the left. A read made while reversing is
 *  mirrored, so positions always increase for steady travel.
 *
 * @param  p_edges  Capture edges (bit 0 = level after the edge).
 * @param  p_cl     Left encoder count at each edge (raw direction).
 * @param  p_cr     Right encoder count at each edge (raw direction).
 * @param  count    Edges in the capture.
 * @param  p_setup  { left sign, right sign (+1 / -1 per forward count),
 *                  y / track x 1000 }.
 * @param  p_out    Output: positions in 1/16 count, shifted left by one,
 *                  with the level in bit 0 (the code39_decode() format).
 * @return Travel over the capture, 1/16 counts (absolute).
 */
uint32_t
bar_filter_positions (uint32_t const * p_edges, int32_t const * p_cl,
                      int32_t const * p_cr, uint32_t count,
                      int32_t const * p_setup, uint32_t * p_out)
{
    int32_t  end    = 0;
    int32_t  mirror = 1;
    uint32_t idx    = 0U;

    if (count > 0U)
    {
        end    = sensor_travel(p_cl, p_cr, count - 1U, p_setup);
        mirror = (end < 0) ? -1 : 1;              /* reversing: mirror    */
        end    = mirror * end;
    }

    for (idx = 0U; idx < count; idx++)
    {
        int32_t const pos = mirror * sensor_travel(p_cl, p_cr, idx, p_setup);

        /* Two's complement keeps the differences right even for a
         * negative start; bit 0 carries the level. */
        p_out[idx] = ((uint32_t) pos << 1) |
                     (p_edges[idx] & BAR_FILTER_LEVEL_BIT);
    }

    /* end >= 0 after the mirror, so it fits a uint32_t. */
    return ((uint32_t) end);
}

/*!
 * @brief  Sensor travel from the first edge of a capture to edge idx.
 * @param  p_cl     Left encoder counts (raw direction).
 * @param  p_cr     Right encoder counts (raw direction).
 * @param  idx      Edge.
 * @param  p_setup  See bar_filter_positions().
 * @return Travel, 1/16 counts (+ forward).
 */
static int32_t
sensor_travel (int32_t const * p_cl, int32_t const * p_cr, uint32_t idx,
               int32_t const * p_setup)
{
    /* Counts within one read stay below ~20 000, so the products fit. */
    int32_t const left  = p_setup[SETUP_SIGN_L] * (p_cl[idx] - p_cl[0]);
    int32_t const right = p_setup[SETUP_SIGN_R] * (p_cr[idx] - p_cr[0]);

    return ((HALF_SUB * (left + right)) -
            ((SUB_COUNTS * p_setup[SETUP_YAW] * (right - left)) / YAW_SCALE));
}

/*** end of file ***/

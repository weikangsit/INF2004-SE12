/** @file    bar_filter.c
 *  @brief   Glitch filter for barcode edges (see bar_filter.h).
 */
#include "bar_filter.h"

#define TIME_MASK            (~BAR_FILTER_LEVEL_BIT)

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

/*** end of file ***/

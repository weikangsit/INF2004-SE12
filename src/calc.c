/** @file    calc.c
 *  @brief   Small arithmetic helpers shared by several tasks (see calc.h).
 */
#include <math.h>

#include "calc.h"

#define ROUND_LIMIT          (1.0e6f)
#define FULL_TURN_HALF_DEG   (720)
#define HALF_TURN_HALF_DEG   (360)

/*!
 * @brief  Round to the nearest integer; 0 for NaN or infinity (Barr-C
 *         5.4.b.v), limited to +/-1e6.
 * @param  value  Value to round.
 * @return Rounded value.
 */
int32_t
calc_round (float32_t value)
{
    int32_t result = 0;

    if (isfinite(value))
    {
        /* Clamped to +/-1e6 first, so lroundf() fits an int32_t. */
        result = (int32_t) lroundf(fmaxf(fminf(value, ROUND_LIMIT),
                                         -ROUND_LIMIT));
    }

    return (result);
}

/*!
 * @brief  Angle between two compass headings.
 * @param  from_hdg  Heading, 0.5 deg (0..719).
 * @param  to_hdg    Heading, 0.5 deg (0..719).
 * @return Unsigned difference, 0.5 deg (0..360).
 */
int32_t
calc_heading_diff_half (uint16_t from_hdg, uint16_t to_hdg)
{
    int32_t diff = ((int32_t) to_hdg - (int32_t) from_hdg) %
                   FULL_TURN_HALF_DEG;

    if (diff > HALF_TURN_HALF_DEG)
    {
        diff -= FULL_TURN_HALF_DEG;
    }
    else if (diff < -HALF_TURN_HALF_DEG)
    {
        diff += FULL_TURN_HALF_DEG;
    }
    else
    {
        /* Already -360..360 half degrees. */
    }

    return ((diff < 0) ? -diff : diff);
}

/*** end of file ***/

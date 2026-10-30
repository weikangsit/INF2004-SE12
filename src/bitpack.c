/** @file    bitpack.c
 *  @brief   Minimum-bit field packing (see bitpack.h).
 *
 *  Pure C with no kernel or SDK calls, so web/test_proto.py runs it in a
 *  Cortex-M0+ emulator.
 */
#include "bitpack.h"

#define BITS_PER_BYTE        (8U)

/*!
 * @brief  Clamp a value to a field's range and store it LSB first. The
 *         field's old bits are cleared first, so the buffer need not be
 *         zeroed.
 * @param  p_buf    Packed buffer.
 * @param  pos      Bit position of the field.
 * @param  value    Value to store; clamped to the field range.
 * @param  p_field  Width and signedness of the field.
 * @return Bit position after the field.
 */
uint32_t
bitpack_put (uint8_t * p_buf, uint32_t pos, int32_t value,
             bitpack_field_t const * p_field)
{
    uint32_t const width   = p_field->width;    /* 1..31 bits            */
    uint32_t const span    = 1U << width;
    int32_t        low     = 0;
    int32_t        high    = 0;
    int32_t        clamped = value;
    uint32_t       raw     = 0U;
    uint32_t       bit     = 0U;

    if (p_field->b_signed)
    {
        /* span <= 2^31, so span / 2 fits an int32_t. */
        high = (int32_t) (span / 2U) - 1;
        low  = -high - 1;
    }
    else
    {
        /* span - 1 < 2^31 for widths up to 31 bits. */
        high = (int32_t) (span - 1U);
    }

    if (value < low)
    {
        clamped = low;
    }
    else if (value > high)
    {
        clamped = high;
    }
    else
    {
        /* Already in range. */
    }

    /* Two's complement within the field: add span to a negative value.
     * The result is 0..span-1, so it fits a uint32_t. */
    raw = (clamped < 0) ? (uint32_t) ((int64_t) clamped + (int64_t) span) :
                          (uint32_t) clamped;

    for (bit = 0U; bit < width; bit++)
    {
        uint32_t const index = (pos + bit) / BITS_PER_BYTE;
        uint32_t const mask  = 1U << ((pos + bit) % BITS_PER_BYTE);

        /* mask is one bit of 0..7, so both casts keep their value. */
        if (0U != ((raw >> bit) & 1U))
        {
            p_buf[index] = (uint8_t) (p_buf[index] | mask);
        }
        else
        {
            p_buf[index] = (uint8_t) (p_buf[index] & ~mask);
        }
    }

    return (pos + width);
}

/*!
 * @brief  Read a field.
 * @param  p_buf    Packed buffer.
 * @param  pos      Bit position of the field.
 * @param  p_field  Width and signedness of the field.
 * @return The field value (sign-extended for a signed field).
 */
int32_t
bitpack_get (uint8_t const * p_buf, uint32_t pos,
             bitpack_field_t const * p_field)
{
    uint32_t const width = p_field->width;
    uint32_t       raw   = 0U;
    uint32_t       bit   = 0U;
    int32_t        value = 0;

    for (bit = 0U; bit < width; bit++)
    {
        uint32_t const index = (pos + bit) / BITS_PER_BYTE;
        uint32_t const shift = (pos + bit) % BITS_PER_BYTE;

        raw |= (((uint32_t) p_buf[index] >> shift) & 1U) << bit;
    }

    if (p_field->b_signed && (0U != (raw >> (width - 1U))))
    {
        uint32_t const span = 1U << width;

        /* Negative: subtract span. The result is -span/2..-1, so it fits
         * an int32_t; the int64_t steps avoid unsigned wrap-around. */
        value = (int32_t) ((int64_t) raw - (int64_t) span);
    }
    else
    {
        /* raw < 2^31, so it fits an int32_t. */
        value = (int32_t) raw;
    }

    return (value);
}

/*!
 * @brief  Pack a list of fields starting at a bit position.
 * @param  p_buf      Packed buffer.
 * @param  start_pos  Bit position of the first field.
 * @param  p_values   One value per field.
 * @param  p_fields   Field widths and signedness.
 * @param  count      Number of fields.
 * @return Bit position after the last field.
 */
uint32_t
bitpack_put_list (uint8_t * p_buf, uint32_t start_pos,
                  int32_t const * p_values, bitpack_field_t const * p_fields,
                  uint32_t count)
{
    uint32_t pos = start_pos;
    uint32_t idx = 0U;

    for (idx = 0U; idx < count; idx++)
    {
        pos = bitpack_put(p_buf, pos, p_values[idx], &p_fields[idx]);
    }

    return (pos);
}

/*!
 * @brief  Bit position of field `index` in a layout that starts at bit 0.
 * @param  p_fields  Layout.
 * @param  index     Field number.
 * @return Sum of the widths of the fields before it.
 */
uint32_t
bitpack_offset (bitpack_field_t const * p_fields, uint32_t index)
{
    uint32_t pos = 0U;
    uint32_t idx = 0U;

    for (idx = 0U; idx < index; idx++)
    {
        pos += p_fields[idx].width;
    }

    return (pos);
}

/*** end of file ***/

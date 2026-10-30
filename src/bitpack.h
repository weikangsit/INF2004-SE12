/** @file    bitpack.h
 *  @brief   Minimum-bit field packing (Change Requirement 1).
 *
 *  A packed buffer is a little-endian bit stream: bit 0 of byte 0 first,
 *  each field least significant bit first, signed fields in two's
 *  complement of their own width. Fields are written with explicit shifts
 *  and masks, never C bit-fields, so the layout does not depend on the
 *  compiler (Barr-C 5.5). Used for the MQTT payloads (proto.c) and the
 *  shared vehicle state record (vehicle_state.c).
 */
#ifndef BITPACK_H
#define BITPACK_H

#include <stdbool.h>
#include <stdint.h>

/** One field of a packed layout. */
typedef struct
{
    uint8_t width;      /* 1..31 bits                                    */
    bool    b_signed;   /* two's complement if true                      */
} bitpack_field_t;

uint32_t bitpack_put (uint8_t * p_buf, uint32_t pos, int32_t value,
                      bitpack_field_t const * p_field);
int32_t  bitpack_get (uint8_t const * p_buf, uint32_t pos,
                      bitpack_field_t const * p_field);
uint32_t bitpack_put_list (uint8_t * p_buf, uint32_t start_pos,
                           int32_t const * p_values,
                           bitpack_field_t const * p_fields, uint32_t count);
uint32_t bitpack_offset (bitpack_field_t const * p_fields, uint32_t index);

#endif /* BITPACK_H */

/*** end of file ***/

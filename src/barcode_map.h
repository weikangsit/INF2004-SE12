/**
 * @file    barcode_map.h
 * @brief   Letter -> action table for decoded Code 39 barcodes (A-Z),
 *          loaded from the binary CONFIG frame (see proto.h).
 */
#ifndef BARCODE_MAP_H
#define BARCODE_MAP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BA_NONE = 0,     /* not configured: logged, robot keeps following */
    BA_STRAIGHT,     /* wire code 00 */
    BA_LEFT,         /* wire code 01 */
    BA_RIGHT,        /* wire code 10 */
    BA_UTURN         /* wire code 11 */
} barcode_action_t;

void             barcode_map_init(void);
barcode_action_t barcode_map_get(char letter);

/* Apply a 12-byte CONFIG frame. Returns letters set, 0 if malformed.
 * The frame replaces the whole table (unset letters become BA_NONE).  */
uint32_t         barcode_map_apply(const uint8_t *p_frame, uint32_t len);

uint32_t         barcode_map_mask(void);     /* bit n = 'A'+n set */
bool             barcode_map_is_loaded(void);

#endif /* BARCODE_MAP_H */

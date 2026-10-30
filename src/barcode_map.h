/** @file    barcode_map.h
 *  @brief   Barcode table: the letter (A-Z) that triggers each action.
 *
 *  For each action (straight, left, right, U-turn) the table holds the
 *  Code 39 letter that triggers it, or none. The whole table is one
 *  32-bit word (the 24-bit PROTO_CFG_* layout in proto.h, plus a
 *  "received over MQTT" bit), so a reader always sees a complete table.
 *  Built-in defaults follow Week 6 FR2 (left A, right B, straight C,
 *  U-turn D), so the car can run without WiFi. A table received over
 *  MQTT replaces the defaults.
 */
#ifndef BARCODE_MAP_H
#define BARCODE_MAP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BA_NONE = 0,     /* no action has this letter: logged, keep following */
    BA_STRAIGHT,     /* wire code 00 */
    BA_LEFT,         /* wire code 01 */
    BA_RIGHT,        /* wire code 10 */
    BA_UTURN         /* wire code 11 */
} barcode_map_action_t;

void                 barcode_map_init (void);
barcode_map_action_t barcode_map_get (char letter);
void                 barcode_map_apply (uint32_t table);
uint32_t             barcode_map_table (void);
bool                 barcode_map_is_loaded (void);

#endif /* BARCODE_MAP_H */

/*** end of file ***/

/** @file    barcode_map.h
 *  @brief   Barcode letter -> action table for the letters A-D.
 *
 *  The whole table is one 16-bit word (see the PROTO_CFG_* layout in
 *  proto.h, plus a "received over MQTT" bit), so a reader always sees a
 *  complete table. Built-in defaults follow Week 6 FR2 (A left, B right,
 *  C straight, D U-turn), so the car can run without WiFi. A table
 *  received over MQTT replaces the defaults.
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
} barcode_map_action_t;

void                 barcode_map_init (void);
barcode_map_action_t barcode_map_get (char letter);
void                 barcode_map_apply (uint16_t table);
uint16_t             barcode_map_table (void);
bool                 barcode_map_is_loaded (void);

#endif /* BARCODE_MAP_H */

/*** end of file ***/

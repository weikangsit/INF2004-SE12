/** @file    barcode.h
 *  @brief   Barcode reader (Buddy 3): collects barcode-sensor edges from the
 *           GPIO interrupt, decodes one Code 39 symbol at a time and queues
 *           the letter for the Navigation FSM (bar_cmd_q, Week 6 Section 6.1).
 *
 *  A read counts only after the '*' start and stop characters check out
 *  (Tribunal 1 interface rule). The decoded letter (A..Z) is looked up in
 *  the barcode table (barcode_map.h); only a letter that is in the table
 *  is queued, as its 2-bit slot (Tribunal 1: 2-bit symbol). The slot is
 *  the action's wire code. Each queue message is 1 byte.
 */
#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

void barcode_init (void);
void barcode_poll (void);
bool barcode_take (uint8_t * p_slot);
void barcode_log_table (char const * p_why);

#endif /* BARCODE_H */

/*** end of file ***/

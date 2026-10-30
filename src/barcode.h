/** @file    barcode.h
 *  @brief   Barcode reader (Buddy 3): collects barcode-sensor edges from the
 *           GPIO interrupt, decodes one Code 39 symbol at a time and queues
 *           the letter for the Navigation FSM (bar_cmd_q, Week 6 Section 6.1).
 *
 *  A letter is queued only after the '*' start and stop characters check
 *  out (Tribunal 1 interface rule). Each queue message is 1 byte: the
 *  letter index 0..3 for A..D (2 bits).
 */
#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

void barcode_init (void);
void barcode_poll (void);
bool barcode_take (uint8_t * p_letter);

#endif /* BARCODE_H */

/*** end of file ***/

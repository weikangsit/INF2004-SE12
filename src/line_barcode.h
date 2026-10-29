/**
 * @file    line_barcode.h
 */
#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <stdbool.h>

void line_request_calibration(void);
bool line_is_ready(void);

#endif /* LINE_BARCODE_H */

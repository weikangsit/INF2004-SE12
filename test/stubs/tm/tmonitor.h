/** @file    tmonitor.h  (TEST STUB)
 *  @brief   tm_printf() for the code under test. Output goes to a log that
 *           the tests can search; it is printed only when the environment
 *           variable CAR_TEST_VERBOSE is set.
 */
#ifndef STUB_TMONITOR_H
#define STUB_TMONITOR_H

#include <stddef.h>
#include <tk/tkernel.h>

int tm_printf (UB const * p_format, ...);

#endif /* STUB_TMONITOR_H */

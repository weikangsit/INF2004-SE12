/** @file  cyw43_utk.h  (TEST STUB) radio task status read by comms.c. */
#ifndef STUB_CYW43_UTK_H
#define STUB_CYW43_UTK_H

#include <stdint.h>

typedef struct
{
    uint32_t init_complete;
    int      init_result;
    uint32_t poll_count;
} T_CYW43_UTK_STATUS;

void cyw43_utk_get_status (T_CYW43_UTK_STATUS * p_status);

#endif /* STUB_CYW43_UTK_H */

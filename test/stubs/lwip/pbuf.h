/** @file  pbuf.h  (TEST STUB) lwIP packet buffer calls used by comms.c. */
#ifndef STUB_LWIP_PBUF_H
#define STUB_LWIP_PBUF_H

#include <stdint.h>

struct pbuf
{
    uint16_t tot_len;
    uint8_t  data[16];
};

uint16_t pbuf_copy_partial (struct pbuf const * p_buf, void * p_dst,
                            uint16_t len, uint16_t offset);
uint8_t  pbuf_free (struct pbuf * p_buf);

#endif /* STUB_LWIP_PBUF_H */

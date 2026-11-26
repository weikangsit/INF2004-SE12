/** @file  udp.h  (TEST STUB) lwIP UDP calls used by the beacon listener. */
#ifndef STUB_LWIP_UDP_H
#define STUB_LWIP_UDP_H

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"

struct udp_pcb { int unused; };

typedef void (*udp_recv_fn) (void * p_arg, struct udp_pcb * p_pcb,
                             struct pbuf * p_buf, ip_addr_t const * p_addr,
                             u16_t port);

struct udp_pcb * udp_new_ip_type (u8_t type);
err_t            udp_bind (struct udp_pcb * p_pcb, ip_addr_t const * p_addr,
                           u16_t port);
void             udp_recv (struct udp_pcb * p_pcb, udp_recv_fn recv, void * p_arg);
void             udp_remove (struct udp_pcb * p_pcb);

#endif /* STUB_LWIP_UDP_H */

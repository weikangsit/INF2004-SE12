/** @file  ip_addr.h  (TEST STUB) IPv4 address helpers used by comms.c. */
#ifndef STUB_LWIP_IP_ADDR_H
#define STUB_LWIP_IP_ADDR_H

#include <stdint.h>

typedef struct { uint32_t addr; } ip4_addr_t;
typedef struct { ip4_addr_t ip4; uint8_t type; } ip_addr_t;

#define IPADDR_TYPE_V4        (0U)
#define ip_2_ip4(p_ip)        ((ip4_addr_t *) (p_ip))
#define IP_SET_TYPE(p_ip, t)  ((p_ip)->type = (uint8_t) (t))
#define IP_IS_V4(p_ip)        (IPADDR_TYPE_V4 == (p_ip)->type)
#define ip_addr_copy(dst, src) ((dst) = (src))
#define ip4_addr_get_u32(p)   ((p)->addr)

extern ip_addr_t const stub_ip_any;
#define IP4_ADDR_ANY          (&stub_ip_any)

int          ip4addr_aton (char const * p_text, ip4_addr_t * p_addr);
char const * ipaddr_ntoa (ip_addr_t const * p_addr);

#endif /* STUB_LWIP_IP_ADDR_H */

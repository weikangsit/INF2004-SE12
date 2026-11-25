/** @file  mqtt.h  (TEST STUB) lwIP MQTT client API used by comms.c. */
#ifndef STUB_LWIP_MQTT_H
#define STUB_LWIP_MQTT_H

#include <stdint.h>
#include "lwip/ip_addr.h"

typedef uint8_t  u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t   err_t;

#define ERR_OK       ((err_t) 0)
#define ERR_MEM      ((err_t) -1)
#define ERR_ARG      ((err_t) -16)
#define ERR_ISCONN   ((err_t) -10)

#define MQTT_DATA_FLAG_LAST  (1U)

typedef struct mqtt_client_s { int connected; } mqtt_client_t;

struct mqtt_connect_client_info_t
{
    char const * client_id;
    char const * client_user;
    char const * client_pass;
    u16_t        keep_alive;
};

typedef enum
{
    MQTT_CONNECT_ACCEPTED = 0,
    MQTT_CONNECT_REFUSED_PROTOCOL_VERSION = 1,
    MQTT_CONNECT_DISCONNECTED = 256
} mqtt_connection_status_t;

typedef void (*mqtt_connection_cb_t) (mqtt_client_t * p_client, void * p_arg,
                                      mqtt_connection_status_t status);
typedef void (*mqtt_request_cb_t) (void * p_arg, err_t err);
typedef void (*mqtt_incoming_publish_cb_t) (void * p_arg, char const * p_topic,
                                            u32_t tot_len);
typedef void (*mqtt_incoming_data_cb_t) (void * p_arg, u8_t const * p_data,
                                         u16_t len, u8_t flags);

mqtt_client_t * mqtt_client_new (void);
err_t           mqtt_client_connect (mqtt_client_t * p_client,
                                     ip_addr_t const * p_addr, u16_t port,
                                     mqtt_connection_cb_t cb, void * p_arg,
                                     struct mqtt_connect_client_info_t const * p_info);
u8_t            mqtt_client_is_connected (mqtt_client_t * p_client);
void            mqtt_disconnect (mqtt_client_t * p_client);
void            mqtt_set_inpub_callback (mqtt_client_t * p_client,
                                         mqtt_incoming_publish_cb_t pub_cb,
                                         mqtt_incoming_data_cb_t data_cb,
                                         void * p_arg);
err_t           mqtt_subscribe (mqtt_client_t * p_client, char const * p_topic,
                                u8_t qos, mqtt_request_cb_t cb, void * p_arg);
err_t           mqtt_publish (mqtt_client_t * p_client, char const * p_topic,
                              void const * p_payload, u16_t len, u8_t qos,
                              u8_t retain, mqtt_request_cb_t cb, void * p_arg);

#endif /* STUB_LWIP_MQTT_H */

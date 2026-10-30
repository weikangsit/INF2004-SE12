/** @file    comms.h
 *  @brief   WiFi / MQTT link (Buddy 1): the only module that calls lwIP.
 *
 *  Other tasks never touch the network. They copy a packed frame into the
 *  publish queue with comms_publish(), which never blocks (NFR7). The link
 *  itself runs in the template's CYW43 task through the mqtt_utk_start()
 *  hook (declared in the template's mqtt_utk.h).
 */
#ifndef COMMS_H
#define COMMS_H

#include <stdbool.h>
#include <stdint.h>

/** Topics other tasks may publish on (2 bits). */
typedef enum
{
    COMMS_TOPIC_TELEMETRY = 0,
    COMMS_TOPIC_EVENT,
    COMMS_TOPIC_BAR_DEBUG,
    COMMS_TOPIC_COUNT
} comms_topic_t;

void comms_init (void);
bool comms_publish (comms_topic_t topic, uint8_t const * p_data,
                    uint32_t len);
bool comms_is_online (void);
bool comms_radio_is_alive (void);

#endif /* COMMS_H */

/*** end of file ***/

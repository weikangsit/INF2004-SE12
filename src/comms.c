/** @file    comms.c
 *  @brief   WiFi / MQTT link (Buddy 1) on the INF2004 micro T-Kernel
 *           template (FR6, FR7, NFR4, NFR7).
 *
 *  NOTE: The template's CYW43 task owns the radio and lwIP (NO_SYS = 1),
 *  and lwIP may only be called from that one task. While the WiFi link is
 *  up the task calls mqtt_utk_start() every poll (about 2 ms). This file
 *  replaces the template's demo mqtt_utk.c, so the hook below is the
 *  Comms task context of the Week 6 design: every lwIP call happens here,
 *  and other tasks only copy frames into the publish queue.
 *
 *  Tribunal 1 decisions implemented here:
 *   - A remote e-stop goes to the Safety Monitor directly when it
 *     arrives (safety_report()), which wakes it at once (FR7, 100 ms).
 *   - The radio task's liveness is reported separately from the link
 *     (Version C): a stalled radio task brakes, a WiFi drop does not.
 *   - The publish queue holds 1 s of telemetry plus events and drops the
 *     oldest frame when full; publishing never blocks a caller.
 *   - WiFi loss is only reported (FC_WIFI_LOSS); it never stops the car.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 *  (UB *) casts pass string literals to tm_printf(), which takes UB *,
 *  and (int) casts match its %d / %x conversions.
 *
 *  NOTE: The bit-field structs below keep flags in bool bit-fields and
 *  small numbers in uint8_t bit-fields (an implementation-defined base
 *  type in C99 6.7.2.1 that GCC supports), so each struct is one byte.
 *  They are RAM-only and never sent, so their bit order does not matter
 *  (Barr-C 5.5 applies to data that leaves the processor, see proto.c).
 *  A uint8_t bit-field reads as an int, so it is copied into an unsigned
 *  local before any unsigned arithmetic (Barr-C 5.3c).
 */
#include <string.h>

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include <tm/tmonitor.h>

#include "port.h"

#include "barcode_map.h"
#include "car_config.h"
#include "comms.h"
#include "cyw43_utk.h"
#include "mqtt_utk.h"
#include "nav_fsm.h"
#include "proto.h"
#include "safety.h"
#include "systime.h"
#include "telemetry.h"

/* Timing (ms). */
#define HEARTBEAT_MS         (1000U)
#define CFG_RETRY_MS         (5000U)
#define CONNECT_TIMEOUT_MS   (8000U)
#define HOOK_STALE_MS        (500U)   /* hook not called -> link is down */
#define BACKOFF_BASE_MS      (500U)   /* doubles per failed attempt      */
#define BACKOFF_STEP_MAX     (3U)     /* 500 << 3 = MQTT_BACKOFF_MAX_MS  */
#define KEEPALIVE_S          (10U)
#define SUBSCRIBE_QOS        (1U)
#define PUBLISH_QOS          (0U)
#define PUBLISH_RETAIN       (0U)
#define DRAIN_PER_POLL       (4U)     /* keeps the radio poll short      */

/* Publish queue: 1-byte topic header + packed frame per message.
 * NOTE: A T-Kernel message buffer stores each message as a 4-byte length
 * header plus the message rounded up to 4 bytes (messagebuf.h), so a
 * 19-byte telemetry message takes 24 bytes and a 5-byte event 12. */
#define PUB_HEADER_LEN       (1U)
#define PUB_MSG_MAX_LEN      (PUB_HEADER_LEN + PROTO_MSG_MAX_LEN)
#define MBF_TELEM_SLOT       (24U)
#define MBF_EVENT_SLOT       (12U)
#define PUB_Q_BYTES          (MQTT_OFFLINE_BUF * (MBF_TELEM_SLOT +        \
                                                  MBF_EVENT_SLOT))

#define RX_BUF_LEN           (PROTO_CONFIG_LEN)   /* largest inbound frame */

/* Positions and weights of the digits in __TIME__ ("HH:MM:SS"). */
#define BUILD_HOUR_POS       (0U)
#define BUILD_MIN_POS        (3U)
#define BUILD_SEC_POS        (6U)
#define DIGIT_BASE           (10)
#define HHMM_HOUR_WEIGHT     (100)

#define TOPIC_CMD            MQTT_TOPIC_ROOT "/cmd"
#define TOPIC_CONFIG         MQTT_TOPIC_ROOT "/config/barcode"
#define TOPIC_HEARTBEAT      MQTT_TOPIC_ROOT "/heartbeat"
#define TOPIC_CFG_GET        MQTT_TOPIC_ROOT "/config/get"
#define TOPIC_CFG_ACK        MQTT_TOPIC_ROOT "/config/ack"

/** Link states (2 bits). */
typedef enum
{
    LINK_IDLE = 0,
    LINK_CONNECTING,
    LINK_ONLINE,
    LINK_BACKOFF
} link_state_t;

/** Link status, packed into one byte (Change Requirement 1).
 *  NOTE: Only the CYW43 task writes it (the hook and the lwIP callbacks
 *  it runs). Other tasks only read b_connected, and a byte load is a
 *  single access, so no lock is needed. */
typedef struct
{
    uint8_t state         : 2;   /* link_state_t                          */
    bool    b_connected   : 1;   /* MQTT CONNACK accepted                 */
    bool    b_failed      : 1;   /* connect attempt rejected              */
    bool    b_cfg_ack_due : 1;   /* a table arrived: send config/ack      */
    bool    b_has_pending : 1;   /* g_pending holds an unsent frame       */
    uint8_t backoff_step  : 2;   /* retry delay = 500 ms << step          */
} link_status_t;

/** Inbound topics (2 bits). */
typedef enum
{
    RX_NONE = 0,
    RX_CMD,
    RX_CONFIG
} rx_topic_t;

/** Inbound message being collected (one byte, CYW43 task only). */
typedef struct
{
    uint8_t topic      : 2;      /* rx_topic_t                            */
    uint8_t len        : 2;      /* bytes in g_rx_buf (0..RX_BUF_LEN)     */
    bool    b_overflow : 1;      /* longer than any valid frame           */
} rx_status_t;

static char const * const g_pub_topics[COMMS_TOPIC_COUNT] =
{
    MQTT_TOPIC_ROOT "/telemetry",
    MQTT_TOPIC_ROOT "/event"
};

static ID                       g_pub_q         = 0;
static mqtt_client_t *          gp_mqtt         = NULL;
static volatile link_status_t   g_link          =
    { LINK_IDLE, false, false, false, false, 0U };
static rx_status_t              g_rx            = { RX_NONE, 0U, false };
static uint8_t                  g_rx_buf[RX_BUF_LEN];
static uint8_t                  g_pending[PUB_MSG_MAX_LEN];
static uint8_t                  g_pending_len   = 0U;
static uint8_t                  g_hb_seq        = 0U;
static volatile uint8_t         g_drop_count    = 0U;   /* wraps         */
static uint8_t                  g_drop_reported = 0U;

/* NOTE: Hook-only time stamps are 16-bit: every interval they measure is
 * under 65 s and the hook runs every 2 ms while they are in use. The
 * last-call time stays 32-bit because other tasks compare it after a WiFi
 * outage of any length, where a 16-bit value could wrap and look recent. */
static volatile uint32_t        g_last_hook_ms  = 0U;
static uint16_t                 g_state_ms      = 0U;
static uint16_t                 g_hb_ms         = 0U;
static uint16_t                 g_cfg_req_ms    = 0U;

static void     link_reset (void);
static void     link_idle (void);
static void     link_connecting (void);
static void     link_go_online (void);
static void     link_online (void);
static void     link_backoff (void);
static void     link_start_backoff (void);
static void     link_connect (void);
static void     link_disconnect (void);
static bool     link_is_connected (void);
static void     queue_drain (void);
static void     housekeeping_send (void);
static bool     frame_publish (char const * p_topic, uint8_t const * p_data,
                               uint32_t len);
static void     boot_event_post (void);
static int32_t  digits_read (char const * p_text);
static void     rx_handle_cmd (void);
static void     rx_handle_config (void);
static uint16_t stamp_now (void);
static uint32_t stamp_elapsed (uint16_t stamp);
static void     on_connection (mqtt_client_t * p_client, void * p_arg,
                               mqtt_connection_status_t status);
static void     on_incoming_publish (void * p_arg, char const * p_topic,
                                     u32_t total_len);
static void     on_incoming_data (void * p_arg, u8_t const * p_data,
                                  u16_t len, u8_t flags);

/*!
 * @brief  Create the publish queue. Call from usermain() before the tasks
 *         start.
 */
void
comms_init (void)
{
    T_CMBF create;

    (void) memset(&create, 0, sizeof(create));
    create.mbfatr = TA_TFIFO;
    create.bufsz  = (SZ) PUB_Q_BYTES;             /* 360: fits SZ          */
    create.maxmsz = (INT) PUB_MSG_MAX_LEN;        /* 19: fits INT          */
    g_pub_q       = tk_cre_mbf(&create);
}

/*!
 * @brief  Queue a packed frame for publishing. Never blocks (NFR7): when
 *         the queue is full the oldest frame is dropped.
 *
 *  NOTE: The telemetry task is the only caller, so the drop counter has a
 *  single writer.
 *
 * @param  topic   Topic to publish on.
 * @param  p_data  Packed frame (copied).
 * @param  len     Frame length, 1..PROTO_MSG_MAX_LEN.
 * @return True if the frame was queued.
 */
bool
comms_publish (comms_topic_t topic, uint8_t const * p_data, uint32_t len)
{
    uint8_t msg[PUB_MSG_MAX_LEN];
    bool    b_queued = false;

    if ((g_pub_q > 0) && (topic < COMMS_TOPIC_COUNT) && (len > 0U) &&
        (len <= PROTO_MSG_MAX_LEN))
    {
        /* len <= PROTO_MSG_MAX_LEN, so msg_len fits an INT. */
        INT const msg_len = (INT) (PUB_HEADER_LEN + len);

        msg[0] = (uint8_t) topic;                 /* 0 or 1                */
        (void) memcpy(&msg[PUB_HEADER_LEN], p_data, len);

        if (E_OK == tk_snd_mbf(g_pub_q, msg, msg_len, TMO_POL))
        {
            b_queued = true;
        }
        else
        {
            uint8_t discard[PUB_MSG_MAX_LEN];

            /* Full: drop the oldest frame and keep the newest. */
            (void) tk_rcv_mbf(g_pub_q, discard, TMO_POL);
            g_drop_count++;
            b_queued = (E_OK == tk_snd_mbf(g_pub_q, msg, msg_len, TMO_POL));
        }
    }

    return (b_queued);
}

/*!
 * @brief  Report whether the MQTT session is up.
 * @return True while the session is up and the radio task is running.
 */
bool
comms_is_online (void)
{
    uint32_t const idle_ms     = systime_now_ms() - g_last_hook_ms;
    bool const     b_connected = g_link.b_connected;

    return (b_connected && (idle_ms < HOOK_STALE_MS));
}

/*!
 * @brief  Liveness of the template's radio (Comms) task, for the Safety
 *         Monitor. Its poll counter keeps counting while WiFi is down, so
 *         a counter that stops means the task itself has stalled.
 *
 *  NOTE: Before the radio has finished starting, and if its start-up
 *  failed (the template then parks the task on purpose), the radio is
 *  reported alive: that is "no WiFi" (FC_WIFI_LOSS, warning only), not a
 *  stalled task. Only the Safety Monitor calls this (single caller).
 *
 * @return True if the radio task has polled since the previous call.
 */
bool
comms_radio_is_alive (void)
{
    static uint32_t    last_count = 0U;
    T_CYW43_UTK_STATUS status;
    bool               b_alive    = true;

    cyw43_utk_get_status(&status);

    if ((0U != status.init_complete) && (0 == status.init_result))
    {
        b_alive    = (status.poll_count != last_count);
        last_count = status.poll_count;
    }

    return (b_alive);
}

/*!
 * @brief  Hook called by the template's CYW43 task on every poll while the
 *         WiFi link is up. Runs the link state machine.
 *
 *  NOTE: The name and signature are fixed by the template (mqtt_utk.h),
 *  so this public function does not carry the comms_ prefix.
 */
void
mqtt_utk_start (void)
{
    uint32_t const now     = systime_now_ms();
    uint32_t const idle_ms = now - g_last_hook_ms;

    /* A gap in the calls means the WiFi link dropped: the TCP session is
     * gone, so start again from IDLE. */
    if ((LINK_IDLE != g_link.state) && (idle_ms > HOOK_STALE_MS))
    {
        link_reset();
    }

    g_last_hook_ms = now;

    switch (g_link.state)
    {
        case LINK_IDLE:
            link_idle();
        break;

        case LINK_CONNECTING:
            link_connecting();
        break;

        case LINK_ONLINE:
            link_online();
        break;

        case LINK_BACKOFF:
            link_backoff();
        break;

        default:
            link_reset();
        break;
    }
}

/*!
 * @brief  Drop the session and return to IDLE.
 */
static void
link_reset (void)
{
    link_disconnect();
    g_link.state = LINK_IDLE;
}

/*!
 * @brief  IDLE: start a connection attempt.
 */
static void
link_idle (void)
{
    link_connect();
    g_link.state = LINK_CONNECTING;
    g_state_ms   = stamp_now();
}

/*!
 * @brief  CONNECTING: wait for the CONNACK, or give up and back off.
 */
static void
link_connecting (void)
{
    /* is_connected also catches a CONNACK the callback reported late. */
    bool const b_connected = g_link.b_connected;
    bool const b_failed    = g_link.b_failed;

    if (b_connected || link_is_connected())
    {
        link_go_online();
    }
    else if (b_failed || (stamp_elapsed(g_state_ms) > CONNECT_TIMEOUT_MS))
    {
        uint32_t const step = g_link.backoff_step;

        link_disconnect();                        /* abandon this attempt  */
        tm_printf((UB *) "mqtt: connect failed, retry in %d ms\n",
                  (int) (BACKOFF_BASE_MS << step));
        link_start_backoff();
    }
    else
    {
        /* Still waiting for the CONNACK. */
    }
}

/*!
 * @brief  Connected: subscribe, announce the build and go ONLINE. If a
 *         subscribe request cannot be queued the session is useless (no
 *         commands, no table), so it is dropped and retried.
 */
static void
link_go_online (void)
{
    err_t const cmd_err = mqtt_subscribe(gp_mqtt, TOPIC_CMD, SUBSCRIBE_QOS,
                                         NULL, NULL);
    err_t const cfg_err = mqtt_subscribe(gp_mqtt, TOPIC_CONFIG,
                                         SUBSCRIBE_QOS, NULL, NULL);

    if ((ERR_OK == cmd_err) && (ERR_OK == cfg_err))
    {
        /* Wrapping difference of two 8-bit counters. */
        uint8_t const dropped = (uint8_t) (g_drop_count - g_drop_reported);

        g_link.b_connected   = true;
        g_link.backoff_step  = 0U;
        g_drop_reported      = (uint8_t) (g_drop_reported + dropped);
        tm_printf((UB *) "mqtt: connected, %d frames dropped while offline\n",
                  (int) dropped);

        /* The retained CONFIG frame arrives after the subscribe. */
        boot_event_post();
        g_hb_ms      = stamp_now();
        g_cfg_req_ms = stamp_now();
        g_link.state = LINK_ONLINE;
    }
    else
    {
        tm_printf((UB *) "mqtt: subscribe failed\n");
        link_disconnect();
        link_start_backoff();
    }
}

/*!
 * @brief  ONLINE: publish queued frames and housekeeping messages.
 */
static void
link_online (void)
{
    if (link_is_connected())
    {
        queue_drain();
        housekeeping_send();
    }
    else
    {
        link_disconnect();
        link_start_backoff();
    }
}

/*!
 * @brief  BACKOFF: wait 500 ms, 1 s, 2 s, then 4 s between attempts.
 */
static void
link_backoff (void)
{
    uint32_t const step     = g_link.backoff_step;
    uint32_t const delay_ms = BACKOFF_BASE_MS << step;

    if (stamp_elapsed(g_state_ms) >= delay_ms)
    {
        if (step < BACKOFF_STEP_MAX)
        {
            /* step < 3, so step + 1 fits the 2-bit field. */
            g_link.backoff_step = (uint8_t) (step + 1U);
        }

        g_link.state = LINK_IDLE;
    }
}

/*!
 * @brief  Enter BACKOFF now.
 */
static void
link_start_backoff (void)
{
    g_link.state = LINK_BACKOFF;
    g_state_ms   = stamp_now();
}

/*!
 * @brief  Start a connection attempt to the broker.
 */
static void
link_connect (void)
{
    struct mqtt_connect_client_info_t info;
    ip_addr_t                         broker;
    err_t                             result = ERR_ARG;

    (void) memset(&info, 0, sizeof(info));
    (void) memset(&broker, 0, sizeof(broker));
    info.client_id  = MQTT_CLIENT_ID;
    info.keep_alive = KEEPALIVE_S;
#ifdef MQTT_PASSWORD
    info.client_user = MQTT_USER;
    info.client_pass = MQTT_PASSWORD;
#endif

    g_link.b_connected = false;
    g_link.b_failed    = false;

    if (NULL == gp_mqtt)
    {
        gp_mqtt = mqtt_client_new();
    }
    else
    {
        /* Drop any half-finished session first. Without this a late
         * CONNACK leaves lwIP connected while we retry: the retry gets
         * ERR_ISCONN and the live session is never used. */
        mqtt_disconnect(gp_mqtt);
    }

    if ((NULL != gp_mqtt) &&
        (0 != ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker))))
    {
        IP_SET_TYPE(&broker, IPADDR_TYPE_V4);
        mqtt_set_inpub_callback(gp_mqtt, on_incoming_publish,
                                on_incoming_data, NULL);
        result = mqtt_client_connect(gp_mqtt, &broker, MQTT_BROKER_PORT,
                                     on_connection, NULL, &info);
    }

    if (ERR_ISCONN == result)
    {
        g_link.b_connected = link_is_connected();
    }
    else if (ERR_OK != result)
    {
        g_link.b_failed = true;
    }
    else
    {
        /* The CONNACK arrives through on_connection(). */
    }

    tm_printf((UB *) "mqtt: connecting to " MQTT_BROKER_IP "\n");
}

/*!
 * @brief  Close the session, if there is a client.
 */
static void
link_disconnect (void)
{
    if (NULL != gp_mqtt)
    {
        mqtt_disconnect(gp_mqtt);
    }

    g_link.b_connected = false;
}

/*!
 * @brief  Ask lwIP whether the MQTT session is connected.
 * @return False if there is no client yet.
 */
static bool
link_is_connected (void)
{
    bool b_connected = false;

    if (NULL != gp_mqtt)
    {
        b_connected = (0U != mqtt_client_is_connected(gp_mqtt));
    }

    return (b_connected);
}

/*!
 * @brief  Publish up to DRAIN_PER_POLL queued frames. A frame lwIP cannot
 *         take yet (ERR_MEM) is kept in g_pending and retried next poll.
 */
static void
queue_drain (void)
{
    uint32_t count  = 0U;
    bool     b_stop = false;

    for (count = 0U; (count < DRAIN_PER_POLL) && (!b_stop); count++)
    {
        if (!g_link.b_has_pending)
        {
            INT const len = tk_rcv_mbf(g_pub_q, g_pending, TMO_POL);

            if (len > (INT) PUB_HEADER_LEN)       /* 1: fits INT           */
            {
                /* len <= PUB_MSG_MAX_LEN, so it fits a uint8_t. */
                g_pending_len        = (uint8_t) len;
                g_link.b_has_pending = true;
            }
            else
            {
                b_stop = true;                    /* queue empty           */
            }
        }

        if (g_link.b_has_pending)
        {
            uint32_t const topic    = g_pending[0];
            uint32_t const data_len = (uint32_t) g_pending_len -
                                      PUB_HEADER_LEN;  /* widen, then >= 1 */

            /* COMMS_TOPIC_COUNT is 2, so the cast keeps its value. */
            if ((topic >= (uint32_t) COMMS_TOPIC_COUNT) ||
                frame_publish(g_pub_topics[topic], &g_pending[PUB_HEADER_LEN],
                              data_len))
            {
                g_link.b_has_pending = false;     /* sent, or unknown: drop */
            }
            else
            {
                b_stop = true;                    /* lwIP buffer full      */
            }
        }
    }
}

/*!
 * @brief  Heartbeat (1 Hz), config request and config acknowledgement.
 */
static void
housekeeping_send (void)
{
    if (stamp_elapsed(g_hb_ms) >= HEARTBEAT_MS)
    {
        g_hb_ms = stamp_now();
        g_hb_seq++;                               /* 8-bit, wraps          */
        (void) frame_publish(TOPIC_HEARTBEAT, &g_hb_seq, PROTO_HB_LEN);
    }

    /* Still on the built-in table: ask the web app for one. */
    if ((!barcode_map_is_loaded()) &&
        (stamp_elapsed(g_cfg_req_ms) >= CFG_RETRY_MS))
    {
        g_cfg_req_ms = stamp_now();
        (void) frame_publish(TOPIC_CFG_GET, NULL, 0U);
    }

    if (g_link.b_cfg_ack_due)
    {
        uint8_t const ack = proto_pack_cfg_ack(barcode_map_table(),
                                               barcode_map_is_loaded());

        if (frame_publish(TOPIC_CFG_ACK, &ack, PROTO_CFG_ACK_LEN))
        {
            g_link.b_cfg_ack_due = false;
        }
    }
}

/*!
 * @brief  Publish one frame now (QoS 0, not retained).
 * @param  p_topic  Full topic name.
 * @param  p_data   Payload, or NULL when len is 0.
 * @param  len      Payload length, 0..PROTO_MSG_MAX_LEN.
 * @return False if lwIP could not take it (ERR_MEM): retry later.
 */
static bool
frame_publish (char const * p_topic, uint8_t const * p_data, uint32_t len)
{
    /* len <= PROTO_MSG_MAX_LEN, so it fits a u16_t. */
    err_t const result = mqtt_publish(gp_mqtt, p_topic, p_data, (u16_t) len,
                                      PUBLISH_QOS, PUBLISH_RETAIN, NULL, NULL);

    return (ERR_OK == result);
}

/*!
 * @brief  Report the firmware build time, so the web app shows which build
 *         is running.
 */
static void
boot_event_post (void)
{
    static char const build_time[] = __TIME__;   /* "HH:MM:SS"             */
    int32_t const     hhmm = (digits_read(&build_time[BUILD_HOUR_POS]) *
                              HHMM_HOUR_WEIGHT) +
                             digits_read(&build_time[BUILD_MIN_POS]);
    int32_t const     secs = digits_read(&build_time[BUILD_SEC_POS]);

    /* hhmm <= 2359 and secs <= 59, so both fit an int16_t. */
    telemetry_post_event(EV_BOOT, 0U, (int16_t) hhmm, (int16_t) secs);
}

/*!
 * @brief  Read two decimal digits.
 * @param  p_text  First of two '0'..'9' characters.
 * @return Their value, 0..99.
 */
static int32_t
digits_read (char const * p_text)
{
    return (((p_text[0] - '0') * DIGIT_BASE) + (p_text[1] - '0'));
}

/*!
 * @brief  A CMD frame arrived. An e-stop goes to Safety before anything
 *         else (FR7); every command is then passed to the Navigation FSM.
 *         A command that does not fit in the full queue is dropped, but
 *         an e-stop has already reached Safety through the flag.
 */
static void
rx_handle_cmd (void)
{
    proto_cmd_t cmd = PROTO_CMD_STOP;

    uint32_t const len = g_rx.len;

    if (proto_unpack_cmd(g_rx_buf, len, &cmd))
    {
        /* proto_cmd_t values match remote_cmd_t and fit a byte. */
        uint8_t const cmd_byte = (uint8_t) cmd;

        if (PROTO_CMD_ESTOP == cmd)
        {
            safety_report(SAFETY_EVT_ESTOP);
            tm_printf((UB *) "comms: e-stop received at %d us\n",
                      (int) port_time_us_32());
        }

        nav_fsm_post_remote(cmd_byte);
    }
}

/*!
 * @brief  A CONFIG frame arrived: replace the table and acknowledge it.
 */
static void
rx_handle_config (void)
{
    uint16_t table = 0U;

    uint32_t const len = g_rx.len;

    if (proto_unpack_config(g_rx_buf, len, &table))
    {
        barcode_map_apply(table);
        g_link.b_cfg_ack_due = true;
        tm_printf((UB *) "mqtt: barcode table received (0x%x)\n",
                  (int) table);
    }
}

/*!
 * @brief  Time stamp: the low 16 bits of the millisecond clock.
 * @return Milliseconds, modulo 65536.
 */
static uint16_t
stamp_now (void)
{
    /* Deliberate truncation: only differences of stamps are used. */
    return (systime_stamp());
}

/*!
 * @brief  Milliseconds since a stamp_now() value.
 * @param  stamp  Earlier stamp.
 * @return Elapsed time; correct for intervals under 65.5 s.
 */
static uint32_t
stamp_elapsed (uint16_t stamp)
{
    /* Modular 16-bit difference, widened for unsigned comparisons. */
    return ((uint32_t) (uint16_t) (stamp_now() - stamp));
}

/* ---------------- lwIP callbacks (run on the CYW43 task) -------------- */

/*!
 * @brief  CONNACK received, or the connection was refused or lost.
 * @param  p_client  MQTT client (unused).
 * @param  p_arg     Callback argument (unused).
 * @param  status    Connection result.
 */
static void
on_connection (mqtt_client_t * p_client, void * p_arg,
               mqtt_connection_status_t status)
{
    (void) p_client;
    (void) p_arg;

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_link.b_connected = true;
    }
    else
    {
        g_link.b_connected = false;
        g_link.b_failed    = true;
    }
}

/*!
 * @brief  Start of an inbound PUBLISH: remember which topic it is for.
 * @param  p_arg      Callback argument (unused).
 * @param  p_topic    Topic name.
 * @param  total_len  Payload length.
 */
static void
on_incoming_publish (void * p_arg, char const * p_topic, u32_t total_len)
{
    (void) p_arg;

    if (0 == strcmp(p_topic, TOPIC_CMD))
    {
        g_rx.topic = RX_CMD;
    }
    else if (0 == strcmp(p_topic, TOPIC_CONFIG))
    {
        g_rx.topic = RX_CONFIG;
    }
    else
    {
        g_rx.topic = RX_NONE;
    }

    g_rx.len        = 0U;
    g_rx.b_overflow = (total_len > RX_BUF_LEN);
}

/*!
 * @brief  Payload bytes of an inbound PUBLISH. They can arrive in several
 *         chunks: collect them until the LAST flag, then dispatch.
 * @param  p_arg   Callback argument (unused).
 * @param  p_data  Chunk of payload.
 * @param  len     Chunk length.
 * @param  flags   MQTT_DATA_FLAG_LAST on the final chunk.
 */
static void
on_incoming_data (void * p_arg, u8_t const * p_data, u16_t len, u8_t flags)
{
    uint32_t const held      = g_rx.len;         /* 0..RX_BUF_LEN          */
    uint32_t const chunk_len = len;
    uint32_t const room      = RX_BUF_LEN - held;
    uint32_t const count     = (chunk_len < room) ? chunk_len : room;
    bool const     b_last    = (0U != (flags & MQTT_DATA_FLAG_LAST));

    (void) p_arg;

    if (chunk_len > room)
    {
        g_rx.b_overflow = true;
    }

    (void) memcpy(&g_rx_buf[held], p_data, count);
    g_rx.len = (uint8_t) (held + count);          /* <= RX_BUF_LEN         */

    if (b_last && (!g_rx.b_overflow))
    {
        if (RX_CMD == g_rx.topic)
        {
            rx_handle_cmd();
        }
        else if (RX_CONFIG == g_rx.topic)
        {
            rx_handle_config();
        }
        else
        {
            /* Not ours. */
        }
    }

    if (b_last)
    {
        g_rx.len        = 0U;
        g_rx.b_overflow = false;
    }
}

/*** end of file ***/

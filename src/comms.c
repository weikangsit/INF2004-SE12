/**
 * @file    comms.c
 * @brief   WiFi/MQTT link (Buddy 1) on the INF2004 micro T-Kernel template.
 *
 *  The template owns the CYW43 radio and lwIP (NO_SYS=1) in its own task
 *  (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c, core 0, priority 4).
 *  That task calls mqtt_utk_start() every poll (~2 ms) while the link is
 *  up. This file REPLACES the template's demo mqtt_utk.c, so every lwIP /
 *  MQTT call below runs on the task that owns lwIP - which NO_SYS requires.
 *  Other tasks only copy frames into pub_q via comms_publish().
 *
 *  Every payload is a binary frame - see proto.h for the topic table.
 */
#include <string.h>

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include <tm/tmonitor.h>

#include "port.h"
#include "app.h"
#include "barcode_map.h"
#include "comms.h"
#include "mqtt_utk.h"

#define SUBTOPIC_LEN     (16U)
#define PAYLOAD_MAX      (32U)     /* largest frame = TELEM (32 bytes)  */
#define TOPIC_LEN        (48U)
#define IN_BUF_LEN       (32U)
#define HEARTBEAT_MS     (1000U)
#define CFG_RETRY_MS     (5000U)
#define CONNECT_TIMEOUT_MS (8000U)
#define HOOK_STALE_MS    (500U)    /* hook not called -> link is down   */
#define KEEPALIVE_S      (10U)

#define T_CMD            MQTT_TOPIC_ROOT "/cmd"
#define T_CONFIG         MQTT_TOPIC_ROOT "/config/barcode"

typedef struct
{
    char    sub[SUBTOPIC_LEN];
    uint8_t len;
    uint8_t data[PAYLOAD_MAX];
} pub_msg_t;

typedef enum { CS_IDLE = 0, CS_CONNECTING, CS_ONLINE, CS_BACKOFF } comms_state_t;

static ID                g_pub_q;
static mqtt_client_t    *g_mqtt;
static comms_state_t     g_cs = CS_IDLE;
static volatile bool     g_connected;
static volatile bool     g_failed;
static volatile uint32_t g_last_hook_ms;
static char              g_in_topic[TOPIC_LEN];
static uint8_t           g_in_buf[IN_BUF_LEN];
static uint32_t          g_in_len;
static bool              g_cfg_ack_due;
static pub_msg_t         g_pending;        /* frame lwIP could not take yet */
static bool              g_has_pending;

bool comms_is_online(void)
{
    return g_connected && ((now_ms() - g_last_hook_ms) < HOOK_STALE_MS);
}

void comms_init(void)
{
    T_CMBF cmbf;

    (void)memset(&cmbf, 0, sizeof(cmbf));
    cmbf.mbfatr = TA_TFIFO;
    /* ~1 s of telemetry + events while offline (NFR7) */
    cmbf.bufsz  = (SZ)((MQTT_OFFLINE_BUF * 2U) * (sizeof(pub_msg_t) + 8U));
    cmbf.maxmsz = (INT)sizeof(pub_msg_t);
    g_pub_q = tk_cre_mbf(&cmbf);
}

bool comms_publish(const char *p_subtopic, const uint8_t *p_data, uint32_t len)
{
    pub_msg_t m;

    if (len > PAYLOAD_MAX)
    {
        return false;
    }
    (void)strncpy(m.sub, p_subtopic, SUBTOPIC_LEN - 1U);
    m.sub[SUBTOPIC_LEN - 1U] = '\0';
    m.len = (uint8_t)len;
    (void)memcpy(m.data, p_data, len);
    /* never block a caller on the network (NFR7) */
    return (E_OK == tk_snd_mbf(g_pub_q, &m, (INT)sizeof(m), TMO_POL));
}

/* ---------------- lwIP MQTT callbacks (same task as the hook) ------- */
static void on_connection(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;
    g_connected = (MQTT_CONNECT_ACCEPTED == status);
    if (!g_connected)
    {
        g_failed = true;
    }
}

static void on_incoming_publish(void *arg, const char *topic, u32_t tot_len)
{
    (void)arg;
    (void)tot_len;
    (void)strncpy(g_in_topic, topic, TOPIC_LEN - 1U);
    g_in_topic[TOPIC_LEN - 1U] = '\0';
    g_in_len = 0U;
}

/* Payloads can arrive in several chunks: collect until the LAST flag. */
static void on_incoming_data(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    uint32_t room = IN_BUF_LEN - g_in_len;
    uint32_t n = (len < room) ? len : room;

    (void)arg;
    (void)memcpy(&g_in_buf[g_in_len], data, n);
    g_in_len += n;
    if (0U == (flags & MQTT_DATA_FLAG_LAST))
    {
        return;
    }

    if ((0 == strcmp(g_in_topic, T_CMD)) && (1U == g_in_len) &&
        (PROTO_CMD == (g_in_buf[0] & PROTO_CMD_MASK)))
    {
        uint8_t c = (uint8_t)(g_in_buf[0] & 0x0FU);

        if (c <= (uint8_t)RCMD_MOTOR_TEST)
        {
            remote_cmd_t rc = (remote_cmd_t)c;
            (void)tk_snd_mbf(g_remote_cmd_q, &rc, (INT)sizeof(rc), TMO_POL);
        }
    }
    else if (0 == strcmp(g_in_topic, T_CONFIG))
    {
        uint32_t n_set = barcode_map_apply(g_in_buf, g_in_len);

        tm_printf((UB *)"mqtt: barcode table received, %d letters\n", (int)n_set);
        g_cfg_ack_due = true;
    }
    else
    {
        /* not ours, or malformed */
    }
    g_in_len = 0U;
}

static void mqtt_connect(void)
{
    struct mqtt_connect_client_info_t ci;
    ip_addr_t broker;
    err_t     err;

    (void)memset(&ci, 0, sizeof(ci));
    ci.client_id   = MQTT_CLIENT_ID;
    ci.keep_alive  = KEEPALIVE_S;
#ifdef MQTT_PASSWORD
    ci.client_user = MQTT_USER;
    ci.client_pass = MQTT_PASSWORD;
#endif

    g_connected = false;
    g_failed = false;
    if (NULL == g_mqtt)
    {
        g_mqtt = mqtt_client_new();
    }
    else
    {
        /* Drop any half-finished or forgotten session first. Without this a
         * late CONNACK leaves lwIP connected while we retry: the retry gets
         * ERR_ISCONN, and the live session is never used for publishing. */
        mqtt_disconnect(g_mqtt);
    }
    if ((NULL == g_mqtt) || !ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker)))
    {
        g_failed = true;
        return;
    }
    IP_SET_TYPE(&broker, IPADDR_TYPE_V4);
    mqtt_set_inpub_callback(g_mqtt, on_incoming_publish, on_incoming_data, NULL);
    err = mqtt_client_connect(g_mqtt, &broker, MQTT_BROKER_PORT, on_connection, NULL, &ci);
    if (ERR_ISCONN == err)
    {
        g_connected = (0U != mqtt_client_is_connected(g_mqtt));   /* already up */
    }
    else if (ERR_OK != err)
    {
        g_failed = true;
    }
    else
    {
        /* CONNACK will arrive through on_connection() */
    }
    tm_printf((UB *)"mqtt: connecting to " MQTT_BROKER_IP "\n");
}

static bool publish_now(const char *p_sub, const uint8_t *p_data, uint32_t len)
{
    char   topic[TOPIC_LEN];
    size_t root = sizeof(MQTT_TOPIC_ROOT) - 1U;
    size_t sub = strlen(p_sub);

    if ((root + 1U + sub + 1U) > sizeof(topic))
    {
        return true;                       /* malformed: drop, don't retry */
    }
    (void)memcpy(topic, MQTT_TOPIC_ROOT, root);
    topic[root] = '/';
    (void)memcpy(&topic[root + 1U], p_sub, sub + 1U);
    /* ERR_MEM = lwIP's MQTT output buffer is full: caller retries later */
    return (ERR_OK == mqtt_publish(g_mqtt, topic, p_data, (u16_t)len, 0U, 0U, NULL, NULL));
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/* Hook called by the template's CYW43 task on every poll while the WiFi
 * link is up (replaces the template's demo mqtt_utk.c). */
void mqtt_utk_start(void)
{
    static uint32_t t_state;
    static uint32_t backoff = 500U;
    static uint32_t t_hb;
    static uint32_t hb_seq;
    static uint32_t t_cfg_req;
    uint32_t        t = now_ms();

    /* link was down for a while: the old TCP session is gone */
    if ((CS_IDLE != g_cs) && ((t - g_last_hook_ms) > HOOK_STALE_MS))
    {
        if ((NULL != g_mqtt) && mqtt_client_is_connected(g_mqtt))
        {
            mqtt_disconnect(g_mqtt);
        }
        g_connected = false;
        g_cs = CS_IDLE;
    }
    g_last_hook_ms = t;

    switch (g_cs)
    {
        case CS_IDLE:
            mqtt_connect();
            g_cs = CS_CONNECTING;
            t_state = t;
            break;

        case CS_CONNECTING:
            /* is_connected also catches a CONNACK the callback reported late */
            if (g_connected || (0U != mqtt_client_is_connected(g_mqtt)))
            {
                g_connected = true;
                /* the retained CONFIG frame arrives right after this */
                (void)mqtt_subscribe(g_mqtt, T_CMD, 1U, NULL, NULL);
                (void)mqtt_subscribe(g_mqtt, T_CONFIG, 1U, NULL, NULL);
                tm_printf((UB *)"mqtt: connected\n");
                {
                    /* firmware build time ("HH:MM:SS"), so the web app
                     * shows which build is actually running */
                    static const char bt[] = __TIME__;
                    int16_t hhmm = (int16_t)(((bt[0] - '0') * 1000) + ((bt[1] - '0') * 100) +
                                             ((bt[3] - '0') * 10) + (bt[4] - '0'));
                    int16_t ss = (int16_t)(((bt[6] - '0') * 10) + (bt[7] - '0'));
                    telem_event(EV_BOOT, 0U, hhmm, ss);
                }
                backoff = 500U;
                t_cfg_req = t;
                g_cs = CS_ONLINE;
            }
            else if (g_failed || ((t - t_state) > CONNECT_TIMEOUT_MS))
            {
                mqtt_disconnect(g_mqtt);           /* abandon this attempt cleanly */
                g_connected = false;
                tm_printf((UB *)"mqtt: connect failed, retry in %d ms\n", (int)backoff);
                g_cs = CS_BACKOFF;
                t_state = t;
            }
            else
            {
                /* waiting for CONNACK */
            }
            break;

        case CS_BACKOFF:
            if ((t - t_state) >= backoff)
            {
                backoff = ((backoff * 2U) > MQTT_BACKOFF_MAX_MS) ? MQTT_BACKOFF_MAX_MS : (backoff * 2U);
                g_cs = CS_IDLE;
            }
            break;

        case CS_ONLINE:
        {
            uint32_t  k;

            if (!mqtt_client_is_connected(g_mqtt))
            {
                mqtt_disconnect(g_mqtt);
                g_connected = false;
                g_cs = CS_BACKOFF;
                t_state = t;
                break;
            }
            /* drain at most a few per poll so the radio keeps being serviced;
             * a frame lwIP cannot take yet (ERR_MEM) is kept and retried */
            for (k = 0U; k < 4U; k++)
            {
                if (!g_has_pending)
                {
                    if (tk_rcv_mbf(g_pub_q, &g_pending, TMO_POL) <= 0)
                    {
                        break;
                    }
                    g_has_pending = true;
                }
                if (!publish_now(g_pending.sub, g_pending.data, g_pending.len))
                {
                    break;                        /* buffer full: next poll */
                }
                g_has_pending = false;
            }

            if ((t - t_hb) >= HEARTBEAT_MS)
            {
                uint8_t hb[5];

                t_hb = t;
                hb_seq++;
                hb[0] = PROTO_HB;
                put_u32(&hb[1], hb_seq);
                publish_now("heartbeat", hb, sizeof(hb));
            }

            /* no retained table yet: ask the web app for one */
            if (!barcode_map_is_loaded() && ((t - t_cfg_req) >= CFG_RETRY_MS))
            {
                const uint8_t get = PROTO_CFG_GET;

                t_cfg_req = t;
                publish_now("config/get", &get, 1U);
            }
            if (g_cfg_ack_due)
            {
                uint8_t ack[5];

                g_cfg_ack_due = false;
                ack[0] = PROTO_CFG_ACK;
                put_u32(&ack[1], barcode_map_mask());
                publish_now("config/ack", ack, sizeof(ack));
            }
            break;
        }

        default:
            g_cs = CS_IDLE;
            break;
    }
}

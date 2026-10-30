/** @file    proto.h
 *  @brief   Packed binary MQTT payloads shared by the car and the web app.
 *
 *  Change Requirement 1: every field uses only the bits its valid range
 *  needs. Fields are written into a little-endian bit stream (bit 0 of
 *  byte 0 first) with explicit shifts and masks. C bit-fields are not used
 *  for wire data because their bit order is compiler-defined (Barr-C 5.5).
 *  The full layout tables are in docs/mqtt_protocol.md and the decoder is
 *  web/static/proto.js.
 *
 *  Topic (robot/12/...)  Dir        Bytes  Content
 *  telemetry             car->web   18     proto_telem_t, 10 Hz
 *  event                 car->web   1..4   4-bit kind + per-kind fields
 *  heartbeat             car->web   1      8-bit sequence, 1 Hz
 *  config/barcode        web->car   3      letter A-Z per action (retained)
 *  config/get            car->web   0      request for the table
 *  config/ack            car->web   1      table now in use
 *  cmd                   web->car   1      5-bit tag + 3-bit command
 *  debug/barcode         car->web   3..16  one barcode capture, in frames
 *                                          of up to 10 element widths
 */
#ifndef PROTO_H
#define PROTO_H

#include <stdbool.h>
#include <stdint.h>

/* Payload lengths in bytes. */
#define PROTO_TELEM_LEN       (18U)
#define PROTO_EVENT_MAX_LEN   (4U)
#define PROTO_CONFIG_LEN      (3U)
#define PROTO_CFG_ACK_LEN     (1U)
#define PROTO_HB_LEN          (1U)
#define PROTO_CMD_LEN         (1U)
#define PROTO_MSG_MAX_LEN     (PROTO_TELEM_LEN)

/* Barcode actions on the wire; also the entry index in the CONFIG word. */
#define PROTO_ACT_STRAIGHT    (0U)
#define PROTO_ACT_LEFT        (1U)
#define PROTO_ACT_RIGHT       (2U)
#define PROTO_ACT_UTURN       (3U)

/* Letters a CONFIG table can hold: A..Z. Only the 4 letters in the
 * table are passed on inside the car, as a 2-bit slot (Tribunal 1
 * layout), see docs/mqtt_protocol.md. */
#define PROTO_LETTER_COUNT    (26U)
#define PROTO_ACTION_COUNT    (4U)

/* CONFIG word (24 bits, 3 bytes), one entry per action n = PROTO_ACT_*:
 * bit n set when that action has a letter, bits 4+5n..8+5n its letter -
 * 'A' (0..25; 26..31 are rejected). No two actions may share a letter.
 * The old 2-byte layouts fail the length check. */
#define PROTO_CFG_VALID_MASK  (0x00000FU)
#define PROTO_CFG_LETTER_SHIFT (4U)
#define PROTO_CFG_LETTER_BITS (5U)
#define PROTO_CFG_LETTER_MASK (0x1FU)
#define PROTO_CFG_TABLE_MASK  (0xFFFFFFU)

/* Telemetry status bits (2 bits). */
#define PROTO_ST_LATCHED      (1U << 0)   /* safety latch engaged          */
#define PROTO_ST_CFG_MQTT     (1U << 1)   /* barcode table came over MQTT  */

/* Telemetry line bits (one per digital line sensor, 1 = black). */
#define PROTO_LINE_LEFT       (1U << 0)
#define PROTO_LINE_RIGHT      (1U << 1)

/* Telemetry barcode field (Tribunal 1: 2-bit symbol + valid bit): bits
 * 0..1 slot of the last barcode, bit 2 valid. The slot is the table entry
 * whose letter was read, which is also its action (PROTO_ACT_*). */
#define PROTO_BAR_VALID       (1U << 2)

/* EV_BARCODE codes besides slot 0..3. */
#define PROTO_BAR_EV_UNKNOWN     (0xFEU)   /* letter read, not in the table */
#define PROTO_BAR_EV_UNREADABLE  (0xFFU)   /* no valid Code 39 symbol       */

/* debug/barcode frame: element widths of one capture in 250 us units
 * (11 bits: up to 511.75 ms, above BAR_IDLE_TIMEOUT_US),
 * PROTO_BARDBG_WIDTHS per frame (one Code 39 character + its gap). */
#define PROTO_BARDBG_WIDTHS   (10U)
#define PROTO_BARDBG_MAX_LEN  (16U)    /* 17 header bits + 10 x 11 bits */
#define PROTO_BARDBG_UNIT_US  (250U)
#define PROTO_BARDBG_SEQ_MASK (0x7U)

/* Front range value meaning "no valid echo". */
#define PROTO_RANGE_NO_ECHO   (511U)

/* Remote commands (3 bits). */
typedef enum
{
    PROTO_CMD_START = 0,
    PROTO_CMD_STOP,
    PROTO_CMD_ESTOP,
    PROTO_CMD_FINISH,
    PROTO_CMD_RESET,
    PROTO_CMD_MOTOR_TEST,
    PROTO_CMD_DEADBAND_TEST,
    PROTO_CMD_COUNT
} proto_cmd_t;

/* EVENT kinds (4 bits). The generic inputs code / value1 / value2 of
 * telemetry_post_event() are converted per kind in proto.c. */
typedef enum
{
    EV_STATE = 1,    /* code = new nav state                              */
    EV_BARCODE,      /* code = slot 0..3 (= action), PROTO_BAR_EV_UNKNOWN
                        or PROTO_BAR_EV_UNREADABLE; value2 = edge count  */
    EV_HUMP,         /* value1 = height x0.1 mm                           */
    EV_OBSTACLE,     /* code = bearing deg, value1 = range x0.1 cm,
                        value2 = width x0.1 cm                            */
    EV_FAULT,        /* code = fault_code_t, value1 = detail (stall: bit0
                        left, bit1 right wheel silent, bit2 time limit;
                        heartbeat: task bit), value2 = motion state 0..7 */
    EV_TURN_XCHECK,  /* value1 = encoder deg, value2 = IMU deg            */
    EV_SUMMARY,      /* value1 = hump peak x0.1 mm, value2 = distance cm  */
    EV_ENC_CAL,      /* code 0 ok, bit0/bit1 encoder silent, 4 = rotated;
                        value1 / value2 = left / right detail             */
    EV_MOTOR_TEST,   /* code = phase 1..4, value1 / value2 = counts L / R
                        (sent divided by 4)                               */
    EV_BOOT,         /* value1 = build HHMM, value2 = build SS            */
    EV_MAG_CAL,      /* code 0 ok, 1 failed (too little field change)     */
    EV_DEADBAND,     /* code = wheel (0 left, 1 right), value1 = dead band
                        percent (100 = never moved), value2 = KFF x100   */
    EV_BAR_STATUS,   /* code bit 0 = barcode sensor on black now, bit 1
                        = ring overflowed, bits 2..5 = reads finished;
                        value1 = raw edges, value2 = glitches removed
                        (each in the last report period)                  */
    EV_KIND_COUNT
} proto_event_t;

/* Telemetry values in their wire units, before packing. Out-of-range
 * values are clamped by proto_pack_telem(). */
typedef struct
{
    uint8_t  state;          /* nav_state_t,           4 bits             */
    uint8_t  motion_class;   /* 0..3,                  2 bits             */
    uint8_t  line_bits;      /* PROTO_LINE_*,          2 bits             */
    uint8_t  barcode;        /* PROTO_BAR_*,           3 bits             */
    uint16_t fault_mask;     /* fault.h bit per fault, 16 bits            */
    uint8_t  status;         /* PROTO_ST_*,            2 bits             */
    int16_t  speed_l_cms;    /* cm/s,                  8 bits signed      */
    int16_t  speed_r_cms;
    int16_t  duty_l_pct;     /* percent,               8 bits signed      */
    int16_t  duty_r_pct;
    int16_t  pitch_half_deg; /* 0.5 deg,               9 bits signed      */
    int16_t  roll_deg;       /* 1 deg,                 9 bits signed      */
    uint16_t heading_deg;    /* 0..359 deg,            9 bits             */
    int16_t  turn_rate_dps;  /* deg/s,                 9 bits signed      */
    uint16_t range_cm;       /* 0..510 cm, 511 none,   9 bits             */
    int16_t  distance_cm;    /* cm,                    14 bits signed     */
    uint16_t hump_half_mm;   /* 0.5 mm,                8 bits             */
    uint16_t uptime_ticks;   /* 100 ms ticks,          16 bits            */
} proto_telem_t;

/** One debug/barcode frame, before packing (widths clamp to 11 bits). */
typedef struct
{
    uint8_t  seq;            /* capture number, 3 bits, wraps             */
    uint8_t  chunk;          /* frame of this capture, 0..6, 3 bits       */
    bool     b_last;         /* last frame of the capture, 1 bit          */
    bool     b_letter;       /* a letter was decoded, 1 bit               */
    uint8_t  letter;         /* letter - 'A', 0..25, 5 bits               */
    uint8_t  count;          /* widths in this frame, 0..10, 4 bits       */
    uint16_t widths[PROTO_BARDBG_WIDTHS];  /* 250 us units, 11 bits each  */
} proto_bar_debug_t;

uint32_t proto_pack_telem (proto_telem_t const * p_telem, uint8_t * p_frame);
uint32_t proto_pack_bar_debug (proto_bar_debug_t const * p_dbg,
                               uint8_t * p_frame);
uint32_t proto_pack_event (proto_event_t kind, uint8_t code, int16_t value1,
                           int16_t value2, uint8_t * p_frame);
uint8_t  proto_pack_cfg_ack (uint32_t table, bool b_from_mqtt);
bool     proto_unpack_cmd (uint8_t const * p_frame, uint32_t len,
                           proto_cmd_t * p_cmd);
bool     proto_unpack_config (uint8_t const * p_frame, uint32_t len,
                              uint32_t * p_table);

#endif /* PROTO_H */

/*** end of file ***/

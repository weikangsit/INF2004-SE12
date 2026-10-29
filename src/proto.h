/**
 * @file    proto.h
 * @brief   Binary MQTT payloads shared by the car, the broker and the
 *          web app. No JSON or text - every payload is a packed frame.
 *          Multi-byte fields are little-endian.
 *
 *  Topic                    Dir        Frame
 *  robot/12/config/barcode  web->car   CONFIG   (retained, 12 bytes)
 *  robot/12/config/get      car->web   CFG_GET  (1 byte)
 *  robot/12/config/ack      car->web   CFG_ACK  (5 bytes)
 *  robot/12/telemetry       car->web   TELEM    (32 bytes, 10 Hz)
 *  robot/12/event           car->web   EVENT    (8 bytes)
 *  robot/12/heartbeat       car->web   HB       (5 bytes, 1 Hz)
 *  robot/12/cmd             web->car   CMD      (1 byte)
 *
 *  CONFIG  [0]     0x01
 *          [1..4]  uint32 valid mask, bit n = letter 'A'+n configured
 *          [5..11] 52-bit action field, 2 bits per letter, letter n in
 *                  bits 2n..2n+1 (LSB first across bytes 5..11):
 *                  00 STRAIGHT, 01 LEFT, 10 RIGHT, 11 U-TURN
 *  CFG_GET [0]     0x02
 *  CFG_ACK [0]     0x03, [1..4] uint32 mask of letters now configured
 *  TELEM   see proto_telem_t below (type 0x10)
 *  EVENT   [0] 0x20, [1] event kind, [2] code, [3] reserved,
 *          [4..5] int16 v1, [6..7] int16 v2
 *  HB      [0] 0x30, [1..4] uint32 sequence
 *  CMD     [0] 0x40 | command (0 start, 1 stop, 2 e-stop, 3 finish, 4 reset,
 *                              5 motor test)
 */
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

#define PROTO_CONFIG        (0x01U)
#define PROTO_CFG_GET       (0x02U)
#define PROTO_CFG_ACK       (0x03U)
#define PROTO_TELEM         (0x10U)
#define PROTO_EVENT         (0x20U)
#define PROTO_HB            (0x30U)
#define PROTO_CMD           (0x40U)
#define PROTO_CMD_MASK      (0xF0U)

#define PROTO_CONFIG_LEN    (12U)
#define PROTO_EVENT_LEN     (8U)

#define PROTO_ACT_STRAIGHT  (0U)
#define PROTO_ACT_LEFT      (1U)
#define PROTO_ACT_RIGHT     (2U)
#define PROTO_ACT_UTURN     (3U)

/* flags byte in TELEM */
#define TF_LINE_SEEN        (1U << 0)
#define TF_SAFETY_LATCHED   (1U << 1)
#define TF_IMU_OK           (1U << 2)
#define TF_CFG_LOADED       (1U << 3)
#define TF_IR_L_BLACK       (1U << 4)   /* left line sensor sees black  */
#define TF_IR_R_BLACK       (1U << 5)   /* right line sensor sees black */

/* EVENT kinds */
typedef enum
{
    EV_STATE = 1,      /* code = new nav state, v1 = reason code         */
    EV_BARCODE,        /* code = letter ('A'..'Z', '?' invalid), v1 = action (-1 unset) */
    EV_HUMP,           /* v1 = height x0.1 mm                            */
    EV_OBSTACLE,       /* code = route, v1 = range x0.1 cm, v2 = width x0.1 cm */
    EV_FAULT,          /* code = fault_code_t                            */
    EV_TURN_XCHECK,    /* v1 = encoder deg, v2 = IMU deg                 */
    EV_SUMMARY,        /* v1 = hump peak x0.1 mm, v2 = distance cm       */
    EV_ENC_CAL,        /* code 0: ok, v1/v2 = left/right sign (+1/-1);
                          code bit0/bit1: left/right encoder silent,
                          v1/v2 = raw counts seen;
                          code 4: car rotated v1 deg -> a motor is reversed */
    EV_MOTOR_TEST,     /* code 1/2: left +duty / -duty, 3/4: right +/-;
                          v1/v2 = raw left/right encoder counts          */
    EV_BOOT            /* sent on every broker connect: firmware build
                          time, v1 = HHMM, v2 = SS                       */
} proto_event_t;

typedef enum
{
    FC_NONE = 0,
    FC_STALL,
    FC_COLLISION,
    FC_OVERTILT,
    FC_LINE_LOST,
    FC_ESTOP,
    FC_HEARTBEAT,      /* v1 = missing heartbeat bit                      */
    FC_SELFTEST,
    FC_NO_ROUTE,
    FC_REACQUIRE,
    FC_SENSOR
} fault_code_t;

/* 32-byte telemetry frame */
typedef struct __attribute__((packed))
{
    uint8_t  type;          /* 0x10                                       */
    uint8_t  state;         /* nav_state_t                                */
    uint8_t  flags;         /* TF_*                                       */
    uint8_t  motion_class;  /* 0 still, 1 moving, 2 turning, 3 hump       */
    int16_t  speed_l;       /* mm/s                                       */
    int16_t  speed_r;       /* mm/s                                       */
    int16_t  pitch;         /* 0.1 deg                                    */
    int16_t  roll;          /* 0.1 deg                                    */
    uint16_t yaw;           /* 0.1 deg, magnetometer heading 0..3599      */
    int16_t  turn_rate;     /* 0.1 deg/s                                  */
    uint16_t obstacle;      /* mm, 0xFFFF = no echo                       */
    int8_t   duty_l;        /* percent                                    */
    int8_t   duty_r;        /* percent                                    */
    int32_t  distance;      /* mm travelled                               */
    int8_t   line_pos;      /* -1 / 0 / +1                                */
    uint8_t  barcode;       /* last letter, 0 = none                      */
    uint16_t hump_peak;     /* 0.1 mm                                     */
    uint32_t uptime_ms;
} proto_telem_t;

_Static_assert(sizeof(proto_telem_t) == 32U, "telemetry frame must be 32 bytes");

#endif /* PROTO_H */

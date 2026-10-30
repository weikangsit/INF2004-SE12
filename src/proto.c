/** @file    proto.c
 *  @brief   Packs and unpacks the binary MQTT payloads (see proto.h).
 *
 *  Every payload is a little-endian bit stream: the first field starts at
 *  bit 0 of byte 0, and each field is written least significant bit first.
 *  A signed field is stored as two's complement in its own width (see
 *  bitpack.h, which does the packing). Each
 *  layout below is mirrored in web/static/proto.js and checked by
 *  web/test_proto.py.
 */
#include <string.h>

#include "bitpack.h"
#include "proto.h"

#define BITS_PER_BYTE        (8U)
#define EVENT_KIND_BITS      (4U)
#define TELEM_FIELD_COUNT    (18U)
#define EVENT_FIELDS_MAX     (3U)   /* fields of any kind except BARCODE */
#define BARCODE_FIELD_COUNT  (4U)
#define EVENT_INPUT_COUNT    (3U)   /* code, value1, value2              */

#define CMD_TAG              (0x40U)   /* bits 3..7 of a CMD byte       */
#define CMD_TAG_MASK         (0xF8U)
#define CMD_ID_MASK          (0x07U)

#define ACK_FROM_MQTT_SHIFT  (4U)

/* Unit conversions of the generic event inputs. */
#define TENTHS_PER_UNIT      (10)      /* x0.1 cm or x0.1 mm -> cm or mm */
#define TENTHS_PER_HALF      (5)       /* x0.1 mm -> 0.5 mm steps        */
#define HHMM_SPLIT           (100)     /* HHMM -> hours, minutes         */
#define COUNTS_PER_UNIT      (4)       /* motor-test counts sent / 4     */

/** Fields of one EVENT kind (after the 4-bit kind). */
typedef struct
{
    uint8_t count;
    bitpack_field_t fields[EVENT_FIELDS_MAX];
} event_layout_t;

/** Telemetry layout: the order matches proto_telem_t. */
static bitpack_field_t const g_telem_layout[TELEM_FIELD_COUNT] =
{
    { 4U,  false },   /* state          */
    { 2U,  false },   /* motion_class   */
    { 2U,  false },   /* line_bits      */
    { 3U,  false },   /* barcode        */
    { 16U, false },   /* fault_mask     */
    { 2U,  false },   /* status         */
    { 8U,  true  },   /* speed_l_cms    */
    { 8U,  true  },   /* speed_r_cms    */
    { 8U,  true  },   /* duty_l_pct     */
    { 8U,  true  },   /* duty_r_pct     */
    { 9U,  true  },   /* pitch_half_deg */
    { 9U,  true  },   /* roll_deg       */
    { 9U,  false },   /* heading_deg    */
    { 9U,  true  },   /* turn_rate_dps  */
    { 9U,  false },   /* range_cm       */
    { 14U, true  },   /* distance_cm    */
    { 8U,  false },   /* hump_half_mm   */
    { 16U, false }    /* uptime_ticks   */
};

/** EVENT layouts, indexed by proto_event_t. The BARCODE kind is listed
 *  separately because it has more fields than the others. */
static event_layout_t const g_event_layout[EV_KIND_COUNT] =
{
    { 0U, { { 0U, false } } },                                  /* unused  */
    { 1U, { { 4U, false } } },                                  /* STATE   */
    { 0U, { { 0U, false } } },                                  /* BARCODE */
    { 1U, { { 8U, false } } },                                  /* HUMP    */
    { 3U, { { 8U, false }, { 9U, false }, { 7U, false } } },    /* OBSTACLE*/
    { 3U, { { 4U, false }, { 4U, false }, { 3U, false } } },    /* FAULT   */
    { 2U, { { 9U, false }, { 9U, false } } },                   /* XCHECK  */
    { 2U, { { 8U, false }, { 14U, true } } },                   /* SUMMARY */
    { 3U, { { 3U, false }, { 9U, true }, { 9U, true } } },      /* ENC_CAL */
    { 3U, { { 2U, false }, { 13U, true }, { 13U, true } } },    /* MOTOR   */
    { 3U, { { 5U, false }, { 6U, false }, { 6U, false } } },    /* BOOT    */
    { 1U, { { 1U, false } } },                                  /* MAG_CAL */
    { 3U, { { 1U, false }, { 7U, false }, { 8U, false } } }     /* DEADBAND*/
};

/** BARCODE: slot, valid, read (a letter was decoded), edge count. */
static bitpack_field_t const g_barcode_layout[BARCODE_FIELD_COUNT] =
{
    { 2U, false }, { 1U, false }, { 1U, false }, { 6U, false }
};

static uint32_t event_values (proto_event_t kind, uint8_t code,
                              int16_t value1, int16_t value2,
                              int32_t * p_values);
static bool     config_letters_valid (uint32_t word);

/*!
 * @brief  Pack one telemetry frame.
 * @param  p_telem  Values in wire units (clamped to each field's range).
 * @param  p_frame  Output, at least PROTO_TELEM_LEN bytes.
 * @return Number of bytes written (PROTO_TELEM_LEN).
 */
uint32_t
proto_pack_telem (proto_telem_t const * p_telem, uint8_t * p_frame)
{
    int32_t const values[TELEM_FIELD_COUNT] =
    {
        p_telem->state,        p_telem->motion_class,  p_telem->line_bits,
        p_telem->barcode,      p_telem->fault_mask,    p_telem->status,
        p_telem->speed_l_cms,  p_telem->speed_r_cms,   p_telem->duty_l_pct,
        p_telem->duty_r_pct,   p_telem->pitch_half_deg, p_telem->roll_deg,
        p_telem->heading_deg,  p_telem->turn_rate_dps, p_telem->range_cm,
        p_telem->distance_cm,  p_telem->hump_half_mm,  p_telem->uptime_ticks
    };

    (void) memset(p_frame, 0, PROTO_TELEM_LEN);
    (void) bitpack_put_list(p_frame, 0U, values, g_telem_layout,
                            TELEM_FIELD_COUNT);

    return (PROTO_TELEM_LEN);
}

/*!
 * @brief  Pack one EVENT frame: 4-bit kind, then the fields of that kind.
 * @param  kind     Event kind.
 * @param  code     Kind-specific code (see proto_event_t).
 * @param  value1   Kind-specific value 1.
 * @param  value2   Kind-specific value 2.
 * @param  p_frame  Output, at least PROTO_EVENT_MAX_LEN bytes.
 * @return Number of bytes written, or 0 for an unknown kind.
 */
uint32_t
proto_pack_event (proto_event_t kind, uint8_t code, int16_t value1,
                  int16_t value2, uint8_t * p_frame)
{
    static bitpack_field_t const kind_field = { EVENT_KIND_BITS, false };
    int32_t                      values[BARCODE_FIELD_COUNT];
    uint32_t const               count = event_values(kind, code, value1,
                                                      value2, values);
    uint32_t                     pos   = 0U;
    uint32_t                     len   = 0U;

    if (0U != count)
    {
        (void) memset(p_frame, 0, PROTO_EVENT_MAX_LEN);
        pos = bitpack_put(p_frame, pos, (int32_t) kind,   /* 1..11 */
                          &kind_field);

        if (EV_BARCODE == kind)
        {
            pos = bitpack_put_list(p_frame, pos, values, g_barcode_layout,
                                   count);
        }
        else
        {
            pos = bitpack_put_list(p_frame, pos, values,
                                   g_event_layout[kind].fields, count);
        }

        len = (pos + BITS_PER_BYTE - 1U) / BITS_PER_BYTE;
    }

    return (len);
}

/*!
 * @brief  Build the CFG_ACK byte: bits 0..3 actions that have a letter,
 *         bit 4 set when the table came over MQTT (clear = built-in
 *         defaults).
 * @param  table        Packed table (PROTO_CFG_* layout).
 * @param  b_from_mqtt  True if the table was received over MQTT.
 * @return The CFG_ACK byte.
 */
uint8_t
proto_pack_cfg_ack (uint32_t table, bool b_from_mqtt)
{
    uint32_t ack = table & PROTO_CFG_VALID_MASK;

    if (b_from_mqtt)
    {
        ack |= (1U << ACK_FROM_MQTT_SHIFT);
    }

    /* At most 5 bits are set, so the value fits in a byte. */
    return ((uint8_t) ack);
}

/*!
 * @brief  Validate and decode a CMD frame.
 * @param  p_frame  Received bytes.
 * @param  len      Number of received bytes.
 * @param  p_cmd    Output command, written only on success.
 * @return True if the frame is a valid command.
 */
bool
proto_unpack_cmd (uint8_t const * p_frame, uint32_t len, proto_cmd_t * p_cmd)
{
    bool b_ok = false;

    if ((PROTO_CMD_LEN == len) &&
        (CMD_TAG == ((uint32_t) p_frame[0] & CMD_TAG_MASK)))
    {
        uint32_t cmd_id = (uint32_t) p_frame[0] & CMD_ID_MASK;

        /* PROTO_CMD_COUNT is 7, so the cast keeps its value. */
        if (cmd_id < (uint32_t) PROTO_CMD_COUNT)
        {
            /* cmd_id is below PROTO_CMD_COUNT, so it is a valid enum value. */
            *p_cmd = (proto_cmd_t) cmd_id;
            b_ok = true;
        }
    }

    return (b_ok);
}

/*!
 * @brief  Validate and decode a CONFIG frame (letter A-Z per action).
 * @param  p_frame  Received bytes.
 * @param  len      Number of received bytes.
 * @param  p_table  Output 24-bit table, written only on success.
 * @return True if the frame is valid (right length, every configured
 *         letter in A..Z, no letter given to two actions).
 */
bool
proto_unpack_config (uint8_t const * p_frame, uint32_t len,
                     uint32_t * p_table)
{
    bool b_ok = false;

    if (PROTO_CONFIG_LEN == len)
    {
        uint32_t const word = (uint32_t) p_frame[0] |
                              ((uint32_t) p_frame[1] << BITS_PER_BYTE) |
                              ((uint32_t) p_frame[2] <<
                               (2U * BITS_PER_BYTE));

        if (config_letters_valid(word))
        {
            *p_table = word & PROTO_CFG_TABLE_MASK;
            b_ok = true;
        }
    }

    return (b_ok);
}

/*!
 * @brief  Convert the generic event inputs to the field values of a kind.
 * @param  kind      Event kind.
 * @param  code      Kind-specific code.
 * @param  value1    Kind-specific value 1.
 * @param  value2    Kind-specific value 2.
 * @param  p_values  Output, at least BARCODE_FIELD_COUNT entries.
 * @return Number of fields, or 0 for an unknown kind.
 */
static uint32_t
event_values (proto_event_t kind, uint8_t code, int16_t value1,
              int16_t value2, int32_t * p_values)
{
    uint32_t count = 0U;

    switch (kind)
    {
        case EV_BARCODE:
        {
            bool const b_valid = ((uint32_t) code < PROTO_ACTION_COUNT);

            /* code is 0..3 when valid, so it fits an int32_t. */
            p_values[0] = b_valid ? (int32_t) code : 0;
            p_values[1] = b_valid ? 1 : 0;
            p_values[2] = (PROTO_BAR_EV_UNREADABLE != code) ? 1 : 0;
            p_values[3] = value2;
            count = BARCODE_FIELD_COUNT;
        }
        break;

        case EV_HUMP:
            p_values[0] = (value1 + (TENTHS_PER_HALF / 2)) / TENTHS_PER_HALF;
            count = 1U;
        break;

        case EV_OBSTACLE:
            p_values[0] = code;
            p_values[1] = value1 / TENTHS_PER_UNIT;   /* 511+ = no echo */
            p_values[2] = value2 / TENTHS_PER_UNIT;
            count = 3U;
        break;

        case EV_SUMMARY:
            p_values[0] = (value1 + (TENTHS_PER_HALF / 2)) / TENTHS_PER_HALF;
            p_values[1] = value2;
            count = 2U;
        break;

        case EV_MOTOR_TEST:
            p_values[0] = (int32_t) code - 1;     /* phases 1..4 -> 0..3   */
            p_values[1] = value1 / COUNTS_PER_UNIT;   /* +/-16 380 counts  */
            p_values[2] = value2 / COUNTS_PER_UNIT;
            count = 3U;
        break;

        case EV_BOOT:
            p_values[0] = value1 / HHMM_SPLIT;
            p_values[1] = value1 % HHMM_SPLIT;
            p_values[2] = value2;
            count = 3U;
        break;

        case EV_STATE:
        case EV_FAULT:
        case EV_TURN_XCHECK:
        case EV_ENC_CAL:
        case EV_MAG_CAL:
        case EV_DEADBAND:
        {
            /* Code first (unless the kind has none), then value1, value2. */
            int32_t const raw[EVENT_INPUT_COUNT] = { code, value1, value2 };
            uint32_t const first = (EV_TURN_XCHECK == kind) ? 1U : 0U;
            uint32_t       idx   = 0U;

            count = g_event_layout[kind].count;
            for (idx = 0U; idx < count; idx++)
            {
                p_values[idx] = raw[first + idx];
            }
        }
        break;

        default:
            count = 0U;
        break;
    }

    return (count);
}

/*!
 * @brief  Check every configured letter is A..Z and no letter is given
 *         to two actions, so a decoded letter selects at most one action.
 * @param  word  CONFIG word (PROTO_CFG_* layout).
 * @return True if every configured action has its own valid letter.
 */
static bool
config_letters_valid (uint32_t word)
{
    uint32_t used   = 0U;                         /* bit n = letter 'A'+n  */
    uint32_t action = 0U;
    bool     b_ok   = true;

    for (action = 0U; action < PROTO_ACTION_COUNT; action++)
    {
        if (0U != (word & (1U << action)))
        {
            uint32_t const letter = (word >> (PROTO_CFG_LETTER_SHIFT +
                                              (PROTO_CFG_LETTER_BITS *
                                               action))) &
                                    PROTO_CFG_LETTER_MASK;

            if ((letter >= PROTO_LETTER_COUNT) ||
                (0U != (used & (1U << letter))))
            {
                b_ok = false;
            }
            else
            {
                used |= (1U << letter);
            }
        }
    }

    return (b_ok);
}

/*** end of file ***/

/** @file    barcode_map.c
 *  @brief   Barcode letter -> action table (see barcode_map.h).
 *
 *  Written only by the MQTT link (comms.c) and read by the Navigation FSM
 *  and the telemetry task. The table is a single aligned 16-bit word, and
 *  a halfword load or store is a single access on the Cortex-M0+, so a
 *  reader can never see half of an old table and half of a new one.
 */
#include "barcode_map.h"
#include "proto.h"

/* Bit 12 marks a table that came over MQTT. */
#define TABLE_FROM_MQTT      (1U << 12)

/* Week 6 FR2: A left, B right, C straight, D U-turn, all four valid. */
#define TABLE_DEFAULT        (PROTO_CFG_VALID_MASK |                       \
                              (((PROTO_ACT_LEFT << 0) |                    \
                                (PROTO_ACT_RIGHT << 2) |                   \
                                (PROTO_ACT_STRAIGHT << 4) |                \
                                (PROTO_ACT_UTURN << 6))                    \
                               << PROTO_CFG_ACT_SHIFT))

#define ACTION_BITS          (2U)
#define ACTION_MASK          (0x3U)

static volatile uint16_t g_table = TABLE_DEFAULT;

/*!
 * @brief  Restore the built-in default table.
 */
void
barcode_map_init (void)
{
    g_table = TABLE_DEFAULT;
}

/*!
 * @brief  Look up the action for a decoded letter.
 * @param  letter  Decoded Code 39 character.
 * @return Configured action, or BA_NONE for an unconfigured letter or a
 *         letter outside A-D.
 */
barcode_map_action_t
barcode_map_get (char letter)
{
    barcode_map_action_t action = BA_NONE;
    uint32_t const       table  = g_table;      /* one snapshot        */

    /* The protocol carries PROTO_LETTER_COUNT letters: A..D. */
    if ((letter >= 'A') && (letter <= 'D'))
    {
        /* letter is 'A'..'D' here, so index is 0..3. */
        uint32_t const index = (uint32_t) (letter - 'A');

        if (0U != (table & (1U << index)))
        {
            uint32_t const code = (table >> (PROTO_CFG_ACT_SHIFT +
                                             (ACTION_BITS * index))) &
                                  ACTION_MASK;

            /* BA_STRAIGHT..BA_UTURN follow wire codes 0..3 in order. */
            action = (barcode_map_action_t) ((uint32_t) BA_STRAIGHT + code);
        }
    }

    return (action);
}

/*!
 * @brief  Replace the table with one received over MQTT.
 * @param  table  12-bit table from proto_unpack_config().
 */
void
barcode_map_apply (uint16_t table)
{
    uint32_t const word = ((uint32_t) table & PROTO_CFG_TABLE_MASK) |
                          TABLE_FROM_MQTT;

    /* word keeps 13 bits, so it fits the single 16-bit store. */
    g_table = (uint16_t) word;
}

/*!
 * @brief  Current table.
 * @return The table in the PROTO_CFG_* layout (12 bits).
 */
uint16_t
barcode_map_table (void)
{
    uint32_t const table = g_table;               /* one snapshot          */

    /* Masked to 12 bits, so it fits a uint16_t. */
    return ((uint16_t) (table & PROTO_CFG_TABLE_MASK));
}

/*!
 * @brief  Report where the table came from.
 * @return True once a table has been received over MQTT.
 */
bool
barcode_map_is_loaded (void)
{
    uint32_t const table = g_table;               /* one snapshot          */

    return (0U != (table & TABLE_FROM_MQTT));
}

/*** end of file ***/

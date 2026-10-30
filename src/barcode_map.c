/** @file    barcode_map.c
 *  @brief   Barcode table: the letter for each action (see barcode_map.h).
 *
 *  Written only by the MQTT link (comms.c) and read by the Navigation FSM
 *  and the telemetry task. The table is a single aligned 32-bit word, and
 *  a word load or store is a single access on the Cortex-M0+, so a
 *  reader can never see half of an old table and half of a new one.
 *  proto_unpack_config() rejects a table that gives one letter to two
 *  actions, so a letter selects at most one action.
 */
#include "barcode_map.h"
#include "proto.h"

/* Bit 24 marks a table that came over MQTT. */
#define TABLE_FROM_MQTT      (1U << 24)

/* Letter codes in the table: letter - 'A'. */
#define LETTER_A             (0U)
#define LETTER_B             (1U)
#define LETTER_C             (2U)
#define LETTER_D             (3U)

/* Letter of action n, placed in its 2-bit entry. A macro, not a function
 * (Barr-C 6.3): it builds TABLE_DEFAULT, which must be a constant
 * expression for the static initialiser. */
#define ENTRY(action, letter) ((letter) << (PROTO_CFG_LETTER_SHIFT +       \
                                            (PROTO_CFG_LETTER_BITS *       \
                                             (action))))

/* Week 6 FR2: left A, right B, straight C, U-turn D, all four set. */
#define TABLE_DEFAULT        (PROTO_CFG_VALID_MASK |                       \
                              ENTRY(PROTO_ACT_STRAIGHT, LETTER_C) |        \
                              ENTRY(PROTO_ACT_LEFT, LETTER_A) |            \
                              ENTRY(PROTO_ACT_RIGHT, LETTER_B) |           \
                              ENTRY(PROTO_ACT_UTURN, LETTER_D))

static volatile uint32_t g_table = TABLE_DEFAULT;

/*!
 * @brief  Restore the built-in default table.
 */
void
barcode_map_init (void)
{
    g_table = TABLE_DEFAULT;
}

/*!
 * @brief  Look up which action a decoded letter triggers.
 * @param  letter  Decoded Code 39 character.
 * @return The action whose letter it is, or BA_NONE if no action has it
 *         (including characters outside A-Z).
 */
barcode_map_action_t
barcode_map_get (char letter)
{
    barcode_map_action_t action = BA_NONE;
    uint32_t const       table  = g_table;      /* one snapshot        */
    uint32_t             entry  = 0U;

    /* The protocol carries PROTO_LETTER_COUNT letters: A..Z. */
    if ((letter >= 'A') && (letter <= 'Z'))
    {
        /* letter is 'A'..'Z' here, so code is 0..25. */
        uint32_t const code = (uint32_t) (letter - 'A');

        for (entry = 0U; entry < PROTO_ACTION_COUNT; entry++)
        {
            uint32_t const entry_letter = (table >> (PROTO_CFG_LETTER_SHIFT +
                                                     (PROTO_CFG_LETTER_BITS *
                                                      entry))) &
                                          PROTO_CFG_LETTER_MASK;

            if ((0U != (table & (1U << entry))) && (code == entry_letter))
            {
                /* BA_STRAIGHT..BA_UTURN follow entries 0..3 in order. */
                action = (barcode_map_action_t) ((uint32_t) BA_STRAIGHT +
                                                 entry);
            }
        }
    }

    return (action);
}

/*!
 * @brief  Replace the table with one received over MQTT.
 * @param  table  24-bit table from proto_unpack_config().
 */
void
barcode_map_apply (uint32_t table)
{
    g_table = (table & PROTO_CFG_TABLE_MASK) | TABLE_FROM_MQTT;
}

/*!
 * @brief  Current table.
 * @return The table in the PROTO_CFG_* layout (24 bits).
 */
uint32_t
barcode_map_table (void)
{
    uint32_t const table = g_table;               /* one snapshot          */

    return (table & PROTO_CFG_TABLE_MASK);
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

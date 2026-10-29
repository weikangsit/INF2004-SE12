/**
 * @file    barcode_map.c
 * @brief   Letter -> action table from the binary CONFIG frame.
 *
 *  Written only by the Comms Task, read by the Navigation FSM. Each
 *  entry is one byte, so a reader never sees a torn value.
 */
#include "barcode_map.h"
#include "proto.h"

#define N_LETTERS   (26U)

static volatile uint8_t  g_map[N_LETTERS];
static volatile uint32_t g_mask;
static volatile bool     g_loaded;

void barcode_map_init(void)
{
    uint32_t i;

    for (i = 0U; i < N_LETTERS; i++)
    {
        g_map[i] = (uint8_t)BA_NONE;
    }
    g_mask = 0U;
    g_loaded = false;
}

barcode_action_t barcode_map_get(char letter)
{
    if ((letter < 'A') || (letter > 'Z'))
    {
        return BA_NONE;
    }
    return (barcode_action_t)g_map[(uint32_t)(letter - 'A')];
}

uint32_t barcode_map_mask(void)      { return g_mask; }
bool     barcode_map_is_loaded(void) { return g_loaded; }

uint32_t barcode_map_apply(const uint8_t *p_frame, uint32_t len)
{
    uint64_t bits = 0U;
    uint32_t mask;
    uint32_t i;
    uint32_t n = 0U;

    if ((len != PROTO_CONFIG_LEN) || (p_frame[0] != PROTO_CONFIG))
    {
        return 0U;
    }
    mask = (uint32_t)p_frame[1] | ((uint32_t)p_frame[2] << 8) |
           ((uint32_t)p_frame[3] << 16) | ((uint32_t)p_frame[4] << 24);
    mask &= (1UL << N_LETTERS) - 1UL;
    for (i = 0U; i < 7U; i++)
    {
        bits |= (uint64_t)p_frame[5U + i] << (8U * i);
    }

    for (i = 0U; i < N_LETTERS; i++)
    {
        if (0U != (mask & (1UL << i)))
        {
            g_map[i] = (uint8_t)(BA_STRAIGHT + (uint8_t)((bits >> (2U * i)) & 0x3U));
            n++;
        }
        else
        {
            g_map[i] = (uint8_t)BA_NONE;
        }
    }
    g_mask = mask;
    g_loaded = (n > 0U);
    return n;
}

/** @file    code39.c
 *  @brief   Code 39 decoder (see code39.h).
 *
 *  A symbol '*X*' is 3 characters of 9 elements (bar, space, ... bar)
 *  with a narrow inter-character gap between them: 29 element widths
 *  from 30 edge times. In each character the 3 widest elements are wide,
 *  which does not depend on the car's speed. A read is accepted only if
 *  (Tribunal 1, Buddy 3):
 *   - both outer characters are the '*' start/stop character,
 *   - the middle character is a letter A..Z (digits and the other
 *     Code 39 characters are rejected),
 *   - every character's narrowest wide element is at least
 *     min_ratio_x10 / 10 times its widest narrow element.
 *  A reversed scan is tried when the forward one fails; reversed '*'
 *  reads as 'P', so a backward pass can never pass the forward check.
 *
 *  Reference: ANSI/AIM BC1-1995, Uniform Symbology Specification Code 39.
 */
#include "code39.h"

#define CHAR_ELEMENTS        (9U)
#define CHAR_STRIDE          (10U)     /* 9 elements + inter-char gap    */
#define SYMBOL_ELEMENTS      (29U)
#define SYMBOL_CHARS         (3U)
#define WIDE_PER_CHAR        (3U)
#define RATIO_SCALE          (10U)
#define LEVEL_BIT            (1U)      /* bit 0 of an edge = level       */
#define PATTERN_COUNT        (27U)     /* '*' + A..Z                    */
#define PATTERN_STAR         (0U)
#define PATTERN_LETTER_FIRST (1U)
#define NOT_FOUND            (0xFFU)

/* Wide-element bit masks, bit n = element n (bar, space, bar, ...).
 * The string shows elements 0..8, 1 = wide. Each letter has 2 wide bars
 * and 1 wide space: A-J wide space 3, K-T space 4, U-Z space 1 (of the
 * 4 spaces), with the same 10 wide-bar pairs in each group. */
static uint16_t const g_patterns[PATTERN_COUNT] =
{
    0x0052U,     /* '*'  010010100 */
    0x0121U,     /* 'A'  100001001 */
    0x0124U,     /* 'B'  001001001 */
    0x0025U,     /* 'C'  101001000 */
    0x0130U,     /* 'D'  000011001 */
    0x0031U,     /* 'E'  100011000 */
    0x0034U,     /* 'F'  001011000 */
    0x0160U,     /* 'G'  000001101 */
    0x0061U,     /* 'H'  100001100 */
    0x0064U,     /* 'I'  001001100 */
    0x0070U,     /* 'J'  000011100 */
    0x0181U,     /* 'K'  100000011 */
    0x0184U,     /* 'L'  001000011 */
    0x0085U,     /* 'M'  101000010 */
    0x0190U,     /* 'N'  000010011 */
    0x0091U,     /* 'O'  100010010 */
    0x0094U,     /* 'P'  001010010 */
    0x01C0U,     /* 'Q'  000000111 */
    0x00C1U,     /* 'R'  100000110 */
    0x00C4U,     /* 'S'  001000110 */
    0x00D0U,     /* 'T'  000010110 */
    0x0103U,     /* 'U'  110000001 */
    0x0106U,     /* 'V'  011000001 */
    0x0007U,     /* 'W'  111000000 */
    0x0112U,     /* 'X'  010010001 */
    0x0013U,     /* 'Y'  110010000 */
    0x0016U      /* 'Z'  011010000 */
};

static uint8_t symbol_decode (uint32_t const * p_widths, uint32_t ratio_x10);
static uint8_t char_decode (uint32_t const * p_widths, uint32_t ratio_x10);


/*!
 * @brief  Decode one symbol from edge time stamps.
 *
 *  The capture may carry a stray leading or trailing edge (a chatter the
 *  filter did not catch, or the barcode's own approach/exit), so the
 *  '*X*' symbol is not guaranteed to start at edge 0. Every 30-edge
 *  window is tried, forward then reversed, and the first that frames a
 *  valid '*X*' wins. A window starting mid-symbol cannot pass the
 *  start/stop '*' check, so a false positive is not possible.
 *
 * @param  p_edges_us     Edge times in us; bit 0 holds the level and is
 *                        ignored for the widths.
 * @param  n_edges        Number of edges (at least CODE39_SYMBOL_EDGES).
 * @param  min_ratio_x10  Minimum wide / narrow ratio, x10.
 * @return 0..25 for A..Z, or CODE39_NONE.
 */
uint8_t
code39_decode (uint32_t const * p_edges_us, uint32_t n_edges,
               uint32_t min_ratio_x10)
{
    uint8_t  symbol = CODE39_NONE;
    uint32_t start  = 0U;

    if (n_edges >= CODE39_SYMBOL_EDGES)
    {
        uint32_t const last_start = n_edges - CODE39_SYMBOL_EDGES;

        for (start = 0U;
             (start <= last_start) && (CODE39_NONE == symbol);
             start++)
        {
            uint32_t widths[SYMBOL_ELEMENTS];
            uint32_t reversed[SYMBOL_ELEMENTS];
            uint32_t idx = 0U;

            for (idx = 0U; idx < SYMBOL_ELEMENTS; idx++)
            {
                widths[idx] = (p_edges_us[start + idx + 1U] & ~LEVEL_BIT) -
                              (p_edges_us[start + idx] & ~LEVEL_BIT);
                reversed[SYMBOL_ELEMENTS - 1U - idx] = widths[idx];
            }

            symbol = symbol_decode(widths, min_ratio_x10);
            if (CODE39_NONE == symbol)
            {
                symbol = symbol_decode(reversed, min_ratio_x10);
            }
        }
    }

    return (symbol);
}

/*!
 * @brief  Decode '*X*' from 29 element widths.
 * @param  p_widths   Element widths.
 * @param  ratio_x10  Minimum wide / narrow ratio, x10.
 * @return 0..25 for A..Z, or CODE39_NONE.
 */
static uint8_t
symbol_decode (uint32_t const * p_widths, uint32_t ratio_x10)
{
    uint8_t const first  = char_decode(&p_widths[0], ratio_x10);
    uint8_t const middle = char_decode(&p_widths[CHAR_STRIDE], ratio_x10);
    uint8_t const last   = char_decode(&p_widths[CHAR_STRIDE * 2U],
                                       ratio_x10);
    uint8_t       symbol = CODE39_NONE;

    if ((PATTERN_STAR == first) && (PATTERN_STAR == last) &&
        (NOT_FOUND != middle) && (PATTERN_STAR != middle))
    {
        /* middle is 1..26 here, so the letter index 0..25 fits. */
        symbol = (uint8_t) (middle - PATTERN_LETTER_FIRST);
    }

    return (symbol);
}

/*!
 * @brief  Decode one character of 9 element widths.
 * @param  p_widths   Element widths.
 * @param  ratio_x10  Minimum wide / narrow ratio, x10.
 * @return Index into g_patterns, or NOT_FOUND.
 */
static uint8_t
char_decode (uint32_t const * p_widths, uint32_t ratio_x10)
{
    uint32_t wide_mask  = 0U;
    uint32_t wide_min   = UINT32_MAX;
    uint32_t narrow_max = 0U;
    uint32_t pick       = 0U;
    uint32_t idx        = 0U;
    uint8_t  found      = NOT_FOUND;

    /* Mark the 3 widest elements as wide. */
    for (pick = 0U; pick < WIDE_PER_CHAR; pick++)
    {
        uint32_t best   = 0U;
        uint32_t best_i = 0U;

        for (idx = 0U; idx < CHAR_ELEMENTS; idx++)
        {
            if ((0U == (wide_mask & (1U << idx))) && (p_widths[idx] > best))
            {
                best   = p_widths[idx];
                best_i = idx;
            }
        }

        wide_mask |= 1U << best_i;
    }

    for (idx = 0U; idx < CHAR_ELEMENTS; idx++)
    {
        if (0U != (wide_mask & (1U << idx)))
        {
            wide_min = (p_widths[idx] < wide_min) ? p_widths[idx] : wide_min;
        }
        else
        {
            narrow_max = (p_widths[idx] > narrow_max) ? p_widths[idx] :
                                                        narrow_max;
        }
    }

    /* Reject an unclear read: the wide and narrow widths must be
     * clearly apart. Element widths are at most a few hundred ms in us,
     * so the products cannot overflow. */
    if ((wide_min * RATIO_SCALE) >= (narrow_max * ratio_x10))
    {
        for (idx = 0U; idx < PATTERN_COUNT; idx++)
        {
            if (wide_mask == g_patterns[idx])
            {
                /* idx < PATTERN_COUNT (27), so it fits a uint8_t. */
                found = (uint8_t) idx;
            }
        }
    }

    return (found);
}

/*** end of file ***/

/** @file    code39.c
 *  @brief   Code 39 decoder (see code39.h).
 *
 *  A symbol '*X*' is 3 characters of 9 elements (bar, space, ... bar)
 *  with a narrow inter-character gap between them: 29 element widths
 *  from 30 edges. '*' and every letter have exactly 2 wide bars (of 5)
 *  and 1 wide space (of 4), so bars and spaces are classified apart: the
 *  2 widest bars and the widest space are wide. Comparing black only
 *  with black and white only with white cancels "bar shrink": an IR
 *  sensor's spot and threshold read every black element narrower and
 *  every white one wider by about the same amount (measured: narrow bars
 *  ~4 ms, narrow spaces ~7.5 ms in one character), which made a mixed
 *  "3 widest of 9" choice misjudge or reject characters. The widths may
 *  be times or distances; neither depends on the speed within a
 *  character. A read is accepted only if (Tribunal 1, Buddy 3):
 *   - both outer characters are the '*' start/stop character,
 *   - the middle character is a letter A..Z (digits and the other
 *     Code 39 characters are rejected),
 *   - in every character, the narrowest wide bar is at least
 *     min_ratio_x10 / 10 times the widest narrow bar, and the same for
 *     the spaces.
 *  A reversed scan is tried when the forward one fails; reversed '*'
 *  reads as 'P', so a backward pass can never pass the forward check.
 *
 *  Reference: ANSI/AIM BC1-1995, Uniform Symbology Specification Code 39.
 */
#include <stdbool.h>

#include "code39.h"

#define CHAR_ELEMENTS        (9U)
#define CHAR_STRIDE          (10U)     /* 9 elements + inter-char gap    */
#define SYMBOL_ELEMENTS      (29U)
#define SYMBOL_CHARS         (3U)
#define BAR_FIRST            (0U)      /* bars: elements 0, 2, 4, 6, 8   */
#define SPACE_FIRST          (1U)      /* spaces: elements 1, 3, 5, 7    */
#define CLASS_STEP           (2U)
#define WIDE_BARS            (2U)
#define WIDE_SPACES          (1U)
#define WIDTH_MAX            (0x00FFFFFFU) /* larger: a backward step  */
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

static uint8_t  symbol_decode (uint32_t const * p_widths, uint32_t ratio_x10);
static uint8_t  char_decode (uint32_t const * p_widths, uint32_t ratio_x10);
static uint32_t widest_mask (uint32_t const * p_widths, uint32_t first,
                             uint32_t wide_count);
static bool     class_is_clear (uint32_t const * p_widths, uint32_t first,
                                uint32_t wide_mask, uint32_t ratio_x10);


/*!
 * @brief  Decode one symbol from edge time stamps.
 *
 *  The capture may carry a stray leading or trailing edge (a chatter the
 *  filter did not catch, or the barcode's own approach/exit), so the
 *  '*X*' symbol is not guaranteed to start at edge 0. Every 30-edge
 *  window that starts on a white-to-black edge (a symbol starts with a
 *  bar) is tried, forward then reversed, and the first that frames a
 *  valid '*X*' wins. A window starting mid-symbol cannot pass the
 *  start/stop '*' check.
 *
 * @param  p_edges_us     Edge times in us, or sensor travel
 *                        (bar_filter_positions()); bit 0 holds the level
 *                        after the edge (1 = black) and is not part of
 *                        the widths.
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
            if (0U != (p_edges_us[start] & LEVEL_BIT))   /* starts a bar */
            {
                uint32_t widths[SYMBOL_ELEMENTS];
                uint32_t reversed[SYMBOL_ELEMENTS];
                uint32_t idx = 0U;

                for (idx = 0U; idx < SYMBOL_ELEMENTS; idx++)
                {
                    widths[idx] = (p_edges_us[start + idx + 1U] &
                                   ~LEVEL_BIT) -
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
 * @brief  Decode one character of 9 element widths: the 2 widest bars and
 *         the widest space are wide, each colour checked on its own.
 * @param  p_widths   Element widths.
 * @param  ratio_x10  Minimum wide / narrow ratio, x10.
 * @return Index into g_patterns, or NOT_FOUND.
 */
static uint8_t
char_decode (uint32_t const * p_widths, uint32_t ratio_x10)
{
    uint32_t const wide_bars   = widest_mask(p_widths, BAR_FIRST, WIDE_BARS);
    uint32_t const wide_spaces = widest_mask(p_widths, SPACE_FIRST,
                                             WIDE_SPACES);
    uint32_t const wide_mask   = wide_bars | wide_spaces;
    uint32_t       idx         = 0U;
    uint8_t        found       = NOT_FOUND;

    if (class_is_clear(p_widths, BAR_FIRST, wide_bars, ratio_x10) &&
        class_is_clear(p_widths, SPACE_FIRST, wide_spaces, ratio_x10))
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

/*!
 * @brief  Mark the widest elements of one colour as wide.
 * @param  p_widths    Element widths of one character.
 * @param  first       BAR_FIRST or SPACE_FIRST (every second element).
 * @param  wide_count  How many of that colour are wide.
 * @return Bit mask, bit n = element n is wide (ties go to the earlier).
 */
static uint32_t
widest_mask (uint32_t const * p_widths, uint32_t first, uint32_t wide_count)
{
    uint32_t mask = 0U;
    uint32_t pick = 0U;

    for (pick = 0U; pick < wide_count; pick++)
    {
        uint32_t best   = 0U;
        uint32_t best_i = first;
        uint32_t idx    = 0U;

        for (idx = first; idx < CHAR_ELEMENTS; idx += CLASS_STEP)
        {
            if ((0U == (mask & (1U << idx))) && (p_widths[idx] > best))
            {
                best   = p_widths[idx];
                best_i = idx;
            }
        }

        mask |= 1U << best_i;
    }

    return (mask);
}

/*!
 * @brief  Check that one colour's wide and narrow elements are clearly
 *         apart: narrowest wide >= ratio x widest narrow.
 * @param  p_widths   Element widths of one character.
 * @param  first      BAR_FIRST or SPACE_FIRST.
 * @param  wide_mask  That colour's wide elements (widest_mask()).
 * @param  ratio_x10  Minimum ratio, x10.
 * @return True if the colour is clear. False also if any width is above
 *         WIDTH_MAX (a backward step of the sensor wraps around).
 */
static bool
class_is_clear (uint32_t const * p_widths, uint32_t first,
                uint32_t wide_mask, uint32_t ratio_x10)
{
    uint32_t wide_min   = UINT32_MAX;
    uint32_t narrow_max = 0U;
    uint32_t idx        = 0U;
    bool     b_sane     = true;

    for (idx = first; idx < CHAR_ELEMENTS; idx += CLASS_STEP)
    {
        if (p_widths[idx] > WIDTH_MAX)
        {
            b_sane = false;
        }
        else if (0U != (wide_mask & (1U << idx)))
        {
            wide_min = (p_widths[idx] < wide_min) ? p_widths[idx] : wide_min;
        }
        else
        {
            narrow_max = (p_widths[idx] > narrow_max) ? p_widths[idx] :
                                                        narrow_max;
        }
    }

    /* Widths are at most WIDTH_MAX (2^24) and ratio_x10 is small, so the
     * products cannot overflow. */
    return (b_sane &&
            ((wide_min * RATIO_SCALE) >= (narrow_max * ratio_x10)));
}

/*** end of file ***/

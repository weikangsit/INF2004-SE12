/** @file    fault.c
 *  @brief   Shared fault bitmask (see fault.h).
 *
 *  NOTE: The mask is stored in an event flag rather than a plain variable.
 *  The RP2040 Cortex-M0+ has no exclusive load/store, and a task on the
 *  other core can run at the same time, so a plain "mask |= bit" could lose
 *  an update. tk_set_flg() and tk_clr_flg() are atomic kernel calls that
 *  work from any task on either core and from interrupt handlers.
 */
#include <string.h>
#include <tk/tkernel.h>

#include "fault.h"

_Static_assert(FC_COUNT <= FAULT_MASK_BITS, "fault codes exceed the mask");

/** Every bit of the 16-bit fault mask. */
#define FAULT_ALL_BITS       (0xFFFFU)

static ID g_fault_flg = 0;

/*!
 * @brief  Create the fault event flag. Call once from usermain() before any
 *         task starts.
 */
void
fault_init (void)
{
    T_CFLG create;

    /* Zero first: the struct has optional fields (object name, user
     * buffer) that the kernel reads when the matching attribute is set. */
    (void) memset(&create, 0, sizeof(create));
    create.flgatr  = TA_TFIFO | TA_WMUL;
    create.iflgptn = 0U;
    g_fault_flg = tk_cre_flg(&create);
}

/*!
 * @brief  Set one fault bit.
 * @param  code  Fault to raise.
 */
void
fault_set (fault_code_t code)
{
    if ((g_fault_flg > 0) && (code < FC_COUNT))
    {
        /* NOTE: Only E_ID / E_PAR are possible, both excluded above. */
        (void) tk_set_flg(g_fault_flg, 1U << code);
    }
}

/*!
 * @brief  Clear one fault bit.
 * @param  code  Fault to clear.
 */
void
fault_clear (fault_code_t code)
{
    if ((g_fault_flg > 0) && (code < FC_COUNT))
    {
        /* tk_clr_flg() keeps the bits that are 1 in the pattern. */
        (void) tk_clr_flg(g_fault_flg, ~(1U << code));
    }
}

/*!
 * @brief  Clear every fault bit (operator reset).
 */
void
fault_clear_all (void)
{
    if (g_fault_flg > 0)
    {
        (void) tk_clr_flg(g_fault_flg, 0U);
    }
}

/*!
 * @brief  Read the whole fault mask in one atomic snapshot.
 * @return Bit n set means fault_code_t n is active.
 */
uint16_t
fault_get_mask (void)
{
    T_RFLG   status;
    uint16_t mask = 0U;

    if ((g_fault_flg > 0) && (E_OK == tk_ref_flg(g_fault_flg, &status)))
    {
        /* Only the low 16 bits are ever set (FC_COUNT <= 16). */
        mask = (uint16_t) (status.flgptn & FAULT_ALL_BITS);
    }

    return (mask);
}

/*** end of file ***/

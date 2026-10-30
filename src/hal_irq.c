/** @file    hal_irq.c
 *  @brief   Kernel side of the shared GPIO interrupt (see hal_irq.h).
 *
 *  hal.c (SDK side) does the register work and returns which signals to
 *  raise; this file registers the handler and sets them in its own event
 *  flag with tk_set_flg(), the only kernel call made in the interrupt.
 *
 *  NOTE: (void) casts mark return values that are deliberately ignored.
 */
#include <string.h>
#include <tk/tkernel.h>

#include "hal.h"
#include "hal_irq.h"

/* IO_IRQ_BANK0 is NVIC interrupt 13; level 2 matches the template's own
 * device drivers (ADC). */
#define INTNO_IO_BANK0       (13U)
#define INT_LEVEL_GPIO       (2)

static ID g_isr_flg = 0;

static void gpio_isr (UINT intno);

/*!
 * @brief  Create the signal flag, configure the pins and register the
 *         interrupt handler. Call from usermain() after motor_driver_init().
 */
void
hal_isr_init (void)
{
    T_CFLG create;
    T_DINT handler;

    (void) memset(&create, 0, sizeof(create));
    create.flgatr  = TA_TFIFO | TA_WMUL;
    create.iflgptn = 0U;
    g_isr_flg      = tk_cre_flg(&create);

    hal_gpio_init();

    (void) memset(&handler, 0, sizeof(handler));
    handler.intatr = TA_HLNG;
    handler.inthdr = (FP) gpio_isr;               /* kernel handler type  */

    if (E_OK == tk_def_int(INTNO_IO_BANK0, &handler))
    {
        ClearInt(INTNO_IO_BANK0);
        EnableInt(INTNO_IO_BANK0, INT_LEVEL_GPIO);
    }
}

/*!
 * @brief  Clear a stale echo signal. Call just before the trigger, so a
 *         late echo of an earlier ping cannot end the next wait.
 */
void
hal_echo_arm (void)
{
    (void) tk_clr_flg(g_isr_flg, ~(UINT) HAL_SIG_ECHO);
}

/*!
 * @brief  Wait for the end of an ultrasonic echo pulse (Ultrasonic task),
 *         after hal_echo_arm() and the trigger.
 * @param  timeout_ms  Longest wait.
 * @return True if the echo ended within the timeout.
 */
bool
hal_echo_wait (uint32_t timeout_ms)
{
    UINT pattern = 0U;

    /* timeout_ms is tens of ms, so it fits a TMO. */
    return (E_OK == tk_wai_flg(g_isr_flg, HAL_SIG_ECHO,
                               TWF_ORW | TWF_BITCLR, &pattern,
                               (TMO) timeout_ms));
}

/*!
 * @brief  IO_IRQ_BANK0 handler.
 * @param  intno  Interrupt number (unused).
 */
static void
gpio_isr (UINT intno)
{
    /* HAL_SIG_* values fit a UINT pattern. */
    UINT const pattern = (UINT) hal_gpio_irq_service();

    (void) intno;

    if (0U != pattern)
    {
        (void) tk_set_flg(g_isr_flg, pattern);
    }
}

/*** end of file ***/

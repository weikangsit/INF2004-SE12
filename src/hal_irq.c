/**
 * @file    hal_irq.c
 * @brief   Kernel side of the GPIO edge interrupt (IO_IRQ_BANK0).
 *
 *  hal.c (SDK side) does the register work and returns which owner tasks
 *  must be signalled; this file registers the handler with micro T-Kernel
 *  and turns those bits into event-flag sets (Section 4: the ISR only
 *  counts / time-stamps and signals with tk_set_flg).
 */
#include "app.h"
#include "hal.h"

/* IO_IRQ_BANK0 is NVIC interrupt 13; levels 1 (highest) .. 3 here, and
 * 2 matches the template's own device drivers (ADC).                   */
#define INTNO_IO_BANK0   (13U)
#define INT_LEVEL_GPIO   (2)

static void gpio_isr(UINT intno)
{
    uint32_t sig = hal_gpio_irq_service();
    UINT     ptn = 0U;

    (void)intno;
    if (0U != (sig & HAL_SIG_BARCODE))
    {
        ptn |= EVT_BARCODE_EDGE;
    }
    if (0U != (sig & HAL_SIG_ECHO))
    {
        ptn |= EVT_ECHO_DONE;
    }
    if (0U != ptn)
    {
        (void)tk_set_flg(g_evt_flg, ptn);
    }
}

void hal_isr_init(void)
{
    T_DINT dint;

    hal_gpio_init();

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)gpio_isr;
    if (E_OK == tk_def_int(INTNO_IO_BANK0, &dint))
    {
        ClearInt(INTNO_IO_BANK0);
        EnableInt(INTNO_IO_BANK0, INT_LEVEL_GPIO);
    }
}

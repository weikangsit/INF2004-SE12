/*
 * ws2812.pio.h - pre-assembled form of src/ws2812.pio (what pioasm emits).
 * Checked in because the micro T-Kernel template build has no pioasm step.
 * If you edit ws2812.pio, regenerate with:  pioasm ws2812.pio ws2812.pio.h
 *
 *   0: out    x, 1            side 0 [2]     0x6221
 *   1: jmp    !x, 3           side 1 [1]     0x1123
 *   2: jmp    0               side 1 [4]     0x1400
 *   3: nop                    side 0 [4]     0xa442
 *   T1 = 2, T2 = 5, T3 = 3  -> 10 PIO cycles per bit
 */
#ifndef WS2812_PIO_H
#define WS2812_PIO_H

#include "hardware/pio.h"
#include "hardware/clocks.h"

#define ws2812_wrap_target 0
#define ws2812_wrap 3
#define ws2812_T1 2
#define ws2812_T2 5
#define ws2812_T3 3

static const uint16_t ws2812_program_instructions[] = {
    0x6221, /* 0: out    x, 1            side 0 [2] */
    0x1123, /* 1: jmp    !x, 3           side 1 [1] */
    0x1400, /* 2: jmp    0               side 1 [4] */
    0xa442, /* 3: nop                    side 0 [4] */
};

static const struct pio_program ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length = 4,
    .origin = -1,
    .pio_version = 0,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0,
#endif
};

static inline pio_sm_config ws2812_program_get_default_config(uint offset)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + ws2812_wrap_target, offset + ws2812_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

static inline void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq)
{
    pio_sm_config c;
    float div;

    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    div = (float)clock_get_hz(clk_sys) / (freq * (float)(ws2812_T1 + ws2812_T2 + ws2812_T3));
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

#endif /* WS2812_PIO_H */

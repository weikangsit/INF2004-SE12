/** @file    hal.c
 *  @brief   Board support (team, SDK side), see hal.h.
 *
 *  One IO_IRQ_BANK0 handler serves every edge input (Tribunal 1 shared
 *  resource matrix): wheel encoders (motor_driver.c), the barcode sensor
 *  and the ultrasonic echo. It only counts, time-stamps and signals;
 *  decoding and range calculation happen in the tasks.
 */
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

#include "car_config.h"
#include "hal.h"
#include "motor_driver.h"
#include "port.h"
#include "ws2812.pio.h"

#define US_PER_MS            (1000U)
#define LEVEL_BIT            (1U)        /* bit 0 of a stored edge       */
#define TRIG_PULSE_US        (10U)
#define SERVO_TICK_HZ        (1000000.0f) /* 1 us per PWM tick           */
#define SERVO_WRAP           (19999U)     /* 20 ms = 50 Hz               */
#define SERVO_MIN_US         (500U)
#define SERVO_SPAN_US        (2000U)
#define SERVO_MAX_DEG        (180U)
#define LED_PIXELS           (2U)         /* two on-board NeoPixels      */
#define LED_GRB_SHIFT        (8U)
#define LED_BIT_HZ           (800000.0f)
#define EDGE_BOTH            (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)

/* GRB colours, dimmed (index = hal_led_colour_t). */
static uint32_t const g_led_grb[HAL_LED_COUNT] =
{
    0x000000U,   /* off   */
    0x000020U,   /* blue  */
    0x200000U,   /* green */
    0x182000U,   /* amber */
    0x002000U    /* red   */
};

/* Barcode edge ring: single producer (ISR writes head) and single
 * consumer (Line task writes tail), so no lock is needed; the barriers
 * order the data before the index on both cores. */
static volatile uint32_t g_bar_buf[BAR_EDGE_BUF_LEN];
static volatile uint32_t g_bar_head    = 0U;
static volatile uint32_t g_bar_tail    = 0U;
static volatile uint32_t g_bar_last_us = 0U;     /* ISR only             */
/* Running totals for the barcode status report; ISR is the only writer,
 * and an aligned 32-bit read is a single access. */
static volatile uint32_t g_bar_irqs    = 0U;     /* interrupts on the pin */
static volatile uint32_t g_bar_kept    = 0U;     /* edges stored          */

/* Ultrasonic echo: written by the ISR, read after the echo signal. */
static volatile uint32_t g_echo_rise_us  = 0U;
static volatile uint32_t g_echo_width_us = 0U;
static volatile bool     g_b_echo_valid  = false;

static PIO      g_led_pio  = NULL;
static uint32_t g_led_sm   = 0U;
static bool     g_b_led_ok = false;

static void     input_pin (uint32_t pin, bool b_pull_up);
static uint32_t barcode_isr (uint32_t now_us);
static uint32_t echo_isr (uint32_t now_us);

/*!
 * @brief  Configure the line, barcode and ultrasonic pins and their edge
 *         interrupts (the encoder pins are set up by motor_driver_init()).
 *
 *  NOTE: GP0 / GP1 carry the right encoder: build with CRASH_REPORT=0 so
 *  the template never routes the UART console onto them.
 */
void
hal_gpio_init (void)
{
    input_pin(PIN_IR_L, false);
    input_pin(PIN_IR_R, false);
    input_pin(PIN_IR_BC, false);
    gpio_set_irq_enabled(PIN_IR_BC, EDGE_BOTH, true);

    gpio_init(PIN_US_TRIG);
    gpio_set_dir(PIN_US_TRIG, GPIO_OUT);
    gpio_put(PIN_US_TRIG, false);
    input_pin(PIN_US_ECHO, false);
    gpio_set_irq_enabled(PIN_US_ECHO, EDGE_BOTH, true);
}

/*!
 * @brief  Body of the shared GPIO interrupt (called by hal_irq.c).
 * @return HAL_SIG_* bits for the tasks to be signalled.
 */
uint32_t
hal_gpio_irq_service (void)
{
    uint32_t const now_us = time_us_32();
    /* Same clock as systime_now_ms(), so encoder edge times do not jump
     * when the 32-bit microsecond counter wraps after 71 minutes. */
    uint32_t const now_ms = (uint32_t) (time_us_64() / US_PER_MS);

    motor_driver_encoder_isr(now_ms);

    return (barcode_isr(now_us) | echo_isr(now_us));
}

/*!
 * @brief  Running totals of the barcode pin (status report): interrupts,
 *         and edges kept after the debounce with room in the ring.
 * @param  p_irqs  Output, interrupts on the barcode pin since boot.
 * @param  p_kept  Output, edges stored since boot.
 */
void
hal_bar_stats (uint32_t * p_irqs, uint32_t * p_kept)
{
    *p_irqs = g_bar_irqs;
    *p_kept = g_bar_kept;
}

/*!
 * @brief  Copy the barcode edges captured since the last call. Each entry
 *         is a 1 us time stamp whose bit 0 is replaced by the level after
 *         the edge (1 = black).
 * @param  p_buf  Output edges.
 * @param  max    Capacity of p_buf.
 * @return Number of edges copied.
 */
uint32_t
hal_bar_take (uint32_t * p_buf, uint32_t max)
{
    uint32_t const head  = g_bar_head;
    uint32_t       count = 0U;

    __dmb();                                      /* data after head      */

    while ((g_bar_tail != head) && (count < max))
    {
        p_buf[count] = g_bar_buf[g_bar_tail];
        g_bar_tail   = (g_bar_tail + 1U) % BAR_EDGE_BUF_LEN;
        count++;
    }

    return (count);
}

/*!
 * @brief  Start an ultrasonic ping (10 us trigger pulse).
 */
void
hal_us_trigger (void)
{
    g_b_echo_valid = false;
    gpio_put(PIN_US_TRIG, true);
    port_busy_wait_us(TRIG_PULSE_US);
    gpio_put(PIN_US_TRIG, false);
}

/*!
 * @brief  Width of the last echo pulse.
 * @param  p_us  Output width, us.
 * @return True if a complete echo arrived since the trigger.
 */
bool
hal_us_echo_us (uint32_t * p_us)
{
    bool const b_valid = g_b_echo_valid;

    if (b_valid)
    {
        *p_us = g_echo_width_us;
    }

    return (b_valid);
}

/*!
 * @brief  Servo PWM: 50 Hz, 1 us per tick. Centres the servo.
 */
void
hal_servo_init (void)
{
    pwm_config cfg = pwm_get_default_config();

    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&cfg, (float) clock_get_hz(clk_sys) / SERVO_TICK_HZ);
    pwm_config_set_wrap(&cfg, SERVO_WRAP);
    pwm_init(pwm_gpio_to_slice_num(PIN_SERVO), &cfg, true);
    hal_servo_set_deg(SERVO_CENTRE_DEG);
}

/*!
 * @brief  Set the servo angle (500..2500 us pulse for 0..180 degrees).
 * @param  angle_deg  Angle; values above 180 are clamped.
 */
void
hal_servo_set_deg (uint32_t angle_deg)
{
    uint32_t const angle = (angle_deg > SERVO_MAX_DEG) ? SERVO_MAX_DEG :
                                                         angle_deg;

    /* At most 2500 ticks, so it fits a uint16_t. */
    pwm_set_gpio_level(PIN_SERVO,
                       (uint16_t) (SERVO_MIN_US +
                                   ((angle * SERVO_SPAN_US) / SERVO_MAX_DEG)));
}

/*!
 * @brief  On-board buttons GP20 / GP21 (active low, pulled up).
 */
void
hal_buttons_init (void)
{
    input_pin(PIN_BTN_START, true);
    input_pin(PIN_BTN_CAL, true);
}

/*!
 * @brief  Raw level of the start button.
 * @return True while pressed.
 */
bool
hal_btn_start_raw (void)
{
    return (!gpio_get(PIN_BTN_START));
}

/*!
 * @brief  Raw level of the calibration button.
 * @return True while pressed.
 */
bool
hal_btn_cal_raw (void)
{
    return (!gpio_get(PIN_BTN_CAL));
}

/*!
 * @brief  NeoPixel driver on a free PIO state machine; LED off.
 */
void
hal_led_init (void)
{
    uint offset        = 0U;
    uint state_machine = 0U;

    g_b_led_ok = pio_claim_free_sm_and_add_program(&ws2812_program,
                                                   &g_led_pio,
                                                   &state_machine, &offset);
    if (g_b_led_ok)
    {
        g_led_sm = state_machine;
        ws2812_program_init(g_led_pio, state_machine, offset, PIN_NEOPIXEL,
                            LED_BIT_HZ);
    }

    hal_led_set(HAL_LED_OFF);
}

/*!
 * @brief  Set both on-board pixels.
 * @param  colour  Colour.
 */
void
hal_led_set (hal_led_colour_t colour)
{
    uint32_t idx = 0U;

    if (g_b_led_ok && (colour < HAL_LED_COUNT))
    {
        for (idx = 0U; idx < LED_PIXELS; idx++)
        {
            pio_sm_put_blocking(g_led_pio, g_led_sm,
                                g_led_grb[colour] << LED_GRB_SHIFT);
        }
    }
}

/*!
 * @brief  Configure a GPIO input.
 * @param  pin        Pin.
 * @param  b_pull_up  True for a pull-up, false for no pull.
 */
static void
input_pin (uint32_t pin, bool b_pull_up)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);

    if (b_pull_up)
    {
        gpio_pull_up(pin);
    }
    else
    {
        gpio_disable_pulls(pin);
    }
}

/*!
 * @brief  Barcode part of the interrupt: debounce and store the edge.
 *         A full ring drops the edge (the read then fails cleanly).
 * @param  now_us  Interrupt time.
 * @return HAL_SIG_BARCODE if an edge was stored, else 0.
 */
static uint32_t
barcode_isr (uint32_t now_us)
{
    uint32_t const events  = gpio_get_irq_event_mask(PIN_IR_BC);
    uint32_t       sig = 0U;

    if (0U != events)
    {
        gpio_acknowledge_irq(PIN_IR_BC, events);
        g_bar_irqs++;

        if ((now_us - g_bar_last_us) >= BAR_DEBOUNCE_US)
        {
            uint32_t const level = (IR_BLACK_LEVEL ==
                                    (gpio_get(PIN_IR_BC) ? 1U : 0U)) ? 1U : 0U;
            uint32_t const next  = (g_bar_head + 1U) % BAR_EDGE_BUF_LEN;

            if (next != g_bar_tail)
            {
                g_bar_buf[g_bar_head] = (now_us & ~LEVEL_BIT) | level;
                __dmb();                          /* data before head     */
                g_bar_head = next;
                sig        = HAL_SIG_BARCODE;
                g_bar_kept++;
            }

            g_bar_last_us = now_us;
        }
    }

    return (sig);
}

/*!
 * @brief  Echo part of the interrupt: time stamp only (Tribunal 1).
 * @param  now_us  Interrupt time.
 * @return HAL_SIG_ECHO when the pulse ends, else 0.
 */
static uint32_t
echo_isr (uint32_t now_us)
{
    uint32_t const events  = gpio_get_irq_event_mask(PIN_US_ECHO);
    uint32_t       sig = 0U;

    if (0U != events)
    {
        gpio_acknowledge_irq(PIN_US_ECHO, events);

        if (0U != (events & GPIO_IRQ_EDGE_RISE))
        {
            g_echo_rise_us = now_us;
        }

        if (0U != (events & GPIO_IRQ_EDGE_FALL))
        {
            g_echo_width_us = now_us - g_echo_rise_us;
            g_b_echo_valid  = true;
            sig             = HAL_SIG_ECHO;
        }
    }

    return (sig);
}

/*** end of file ***/

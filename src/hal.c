/**
 * @file    hal.c
 * @brief   Low-level drivers for the Robo Pico (see hal.h).
 */
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/resets.h"

/* SDK side: no <tk/tkernel.h> here (see port.h). */
#include "car_config.h"
#include "port.h"
#include "hal.h"
#include "ws2812.pio.h"

/* ------------------------------------------------------------------ */
/* port.h services                                                     */
/* ------------------------------------------------------------------ */
uint32_t port_time_us_32(void) { return time_us_32(); }
bool     port_gpio_get(uint32_t pin) { return gpio_get(pin); }

void port_busy_wait_us(uint32_t us)
{
    uint32_t t0 = time_us_32();

    while ((time_us_32() - t0) < us)
    {
        /* spin */
    }
}

/* ------------------------------------------------------------------ */
/* Encoders - x4 quadrature decode                                     */
/* ------------------------------------------------------------------ */
/* index = (prev_state << 2) | new_state, state = (A << 1) | B         */
static const int8_t g_quad_table[16] =
{
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

static volatile int32_t  g_enc_cnt_l;
static volatile int32_t  g_enc_cnt_r;
static volatile uint8_t  g_enc_prev_l;
static volatile uint8_t  g_enc_prev_r;
static volatile uint32_t g_enc_edge_ms_l;
static volatile uint32_t g_enc_edge_ms_r;

/* ------------------------------------------------------------------ */
/* Barcode edge ring buffer                                            */
/* ------------------------------------------------------------------ */
static volatile uint32_t g_bar_buf[BAR_EDGE_BUF_LEN];
static volatile uint32_t g_bar_head;
static volatile uint32_t g_bar_tail;
static volatile uint32_t g_bar_last_us;

/* ------------------------------------------------------------------ */
/* Ultrasonic echo                                                     */
/* ------------------------------------------------------------------ */
static volatile uint32_t g_echo_rise_us;
static volatile uint32_t g_echo_width_us;
static volatile bool     g_echo_valid;

static inline uint8_t enc_state(uint32_t pin_a, uint32_t pin_b)
{
    return (uint8_t)((gpio_get(pin_a) << 1) | gpio_get(pin_b));
}

uint32_t hal_gpio_irq_service(void)
{
    uint32_t now_us = time_us_32();
    uint32_t sig = 0U;
    uint32_t ev;
    uint8_t  s;

    /* --- right encoder (GP0 / GP1) --- */
    ev = gpio_get_irq_event_mask(PIN_ENC_R_A) |
         gpio_get_irq_event_mask(PIN_ENC_R_B);
    if (0U != ev)
    {
        gpio_acknowledge_irq(PIN_ENC_R_A, gpio_get_irq_event_mask(PIN_ENC_R_A));
        gpio_acknowledge_irq(PIN_ENC_R_B, gpio_get_irq_event_mask(PIN_ENC_R_B));
        s = enc_state(PIN_ENC_R_A, PIN_ENC_R_B);
        g_enc_cnt_r += g_quad_table[(g_enc_prev_r << 2) | s];
        g_enc_prev_r = s;
        g_enc_edge_ms_r = now_us / 1000U;
    }

    /* --- left encoder (GP7 / GP28) --- */
    ev = gpio_get_irq_event_mask(PIN_ENC_L_A) |
         gpio_get_irq_event_mask(PIN_ENC_L_B);
    if (0U != ev)
    {
        gpio_acknowledge_irq(PIN_ENC_L_A, gpio_get_irq_event_mask(PIN_ENC_L_A));
        gpio_acknowledge_irq(PIN_ENC_L_B, gpio_get_irq_event_mask(PIN_ENC_L_B));
        s = enc_state(PIN_ENC_L_A, PIN_ENC_L_B);
        g_enc_cnt_l += g_quad_table[(g_enc_prev_l << 2) | s];
        g_enc_prev_l = s;
        g_enc_edge_ms_l = now_us / 1000U;
    }

    /* --- barcode sensor (GP6) --- */
    ev = gpio_get_irq_event_mask(PIN_IR_BC);
    if (0U != ev)
    {
        gpio_acknowledge_irq(PIN_IR_BC, ev);
        if ((now_us - g_bar_last_us) >= BAR_DEBOUNCE_US)
        {
            uint32_t level = (gpio_get(PIN_IR_BC) == IR_BLACK_LEVEL) ? 1U : 0U;
            uint32_t next  = (g_bar_head + 1U) % BAR_EDGE_BUF_LEN;

            if (next != g_bar_tail)
            {
                g_bar_buf[g_bar_head] = (now_us & ~1U) | level;
                __dmb();                   /* data visible before head */
                g_bar_head = next;
            }
            g_bar_last_us = now_us;
            sig |= HAL_SIG_BARCODE;
        }
    }

    /* --- ultrasonic echo (GP5) --- */
    ev = gpio_get_irq_event_mask(PIN_US_ECHO);
    if (0U != ev)
    {
        gpio_acknowledge_irq(PIN_US_ECHO, ev);
        if (0U != (ev & GPIO_IRQ_EDGE_RISE))
        {
            g_echo_rise_us = now_us;
        }
        if (0U != (ev & GPIO_IRQ_EDGE_FALL))
        {
            g_echo_width_us = now_us - g_echo_rise_us;
            g_echo_valid = true;
            sig |= HAL_SIG_ECHO;
        }
    }
    return sig;
}

static void input_pin(uint32_t pin, bool pull_up)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (pull_up)
    {
        gpio_pull_up(pin);
    }
    else
    {
        gpio_disable_pulls(pin);
    }
}

void hal_gpio_init(void)
{
    static const uint32_t enc_pins[4] =
        { PIN_ENC_R_A, PIN_ENC_R_B, PIN_ENC_L_A, PIN_ENC_L_B };
    const uint32_t both = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    uint32_t i;

    /* GP0/GP1 carry the right encoder: build with CRASH_REPORT=0 so the
     * template never routes the UART console onto them (gap check 8.3). */
    for (i = 0U; i < 4U; i++)
    {
        input_pin(enc_pins[i], true);
        gpio_set_irq_enabled(enc_pins[i], both, true);
    }
    g_enc_prev_r = enc_state(PIN_ENC_R_A, PIN_ENC_R_B);
    g_enc_prev_l = enc_state(PIN_ENC_L_A, PIN_ENC_L_B);

    input_pin(PIN_IR_L, false);
    input_pin(PIN_IR_R, false);
    input_pin(PIN_IR_BC, false);
    gpio_set_irq_enabled(PIN_IR_BC, both, true);

    gpio_init(PIN_US_TRIG);
    gpio_set_dir(PIN_US_TRIG, GPIO_OUT);
    gpio_put(PIN_US_TRIG, 0);
    input_pin(PIN_US_ECHO, false);
    gpio_set_irq_enabled(PIN_US_ECHO, both, true);
}

int32_t hal_enc_left(void)   { return g_enc_cnt_l; }
int32_t hal_enc_right(void)  { return g_enc_cnt_r; }
uint32_t hal_enc_last_edge_ms_left(void)  { return g_enc_edge_ms_l; }
uint32_t hal_enc_last_edge_ms_right(void) { return g_enc_edge_ms_r; }

/* Single producer (ISR) / single consumer (Line task) ring: the ISR only
 * writes head, the task only writes tail, so no lock is needed - and a
 * local interrupt disable would not help on SMP anyway, because the ISR
 * and the task can run on different cores.                           */
uint32_t hal_bar_take(uint32_t *p_buf, uint32_t max)
{
    uint32_t n = 0U;
    uint32_t head = g_bar_head;

    __dmb();                               /* read data after head */
    while ((g_bar_tail != head) && (n < max))
    {
        p_buf[n] = g_bar_buf[g_bar_tail];
        g_bar_tail = (g_bar_tail + 1U) % BAR_EDGE_BUF_LEN;
        n++;
    }
    return n;
}

void hal_bar_reset(void)
{
    g_bar_tail = g_bar_head;
}

void hal_us_trigger(void)
{
    g_echo_valid = false;
    gpio_put(PIN_US_TRIG, 1);
    port_busy_wait_us(10U);
    gpio_put(PIN_US_TRIG, 0);
}

bool hal_us_echo_us(uint32_t *p_us)
{
    if (g_echo_valid)
    {
        *p_us = g_echo_width_us;
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* Motors: Robo Pico truth table  A/B: H/L, L/H drive; L/L brake      */
/* ------------------------------------------------------------------ */
static uint16_t g_pwm_top;

static void pwm_pin(uint32_t pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
}

void hal_motor_init(void)
{
    pwm_config cfg = pwm_get_default_config();
    uint32_t sys_hz = clock_get_hz(clk_sys);

    /* The template only releases PWM from reset when USE_PTMR=1 (it is 0),
     * and nothing here runs the SDK's runtime init that normally does it.
     * Without this every PWM register write is ignored: no motor drive. */
    unreset_block_num_wait_blocking(RESET_PWM);

    g_pwm_top = (uint16_t)((sys_hz / MOTOR_PWM_HZ) - 1U);  /* 6249 @125 MHz */
    pwm_config_set_wrap(&cfg, g_pwm_top);
    pwm_config_set_clkdiv(&cfg, 1.0f);

    pwm_pin(PIN_M1A); pwm_pin(PIN_M1B);
    pwm_pin(PIN_M2A); pwm_pin(PIN_M2B);
    pwm_init(pwm_gpio_to_slice_num(PIN_M1A), &cfg, true);
    pwm_init(pwm_gpio_to_slice_num(PIN_M2A), &cfg, true);
    hal_motor_brake();
}

static void drive_channel(uint32_t pin_a, uint32_t pin_b, float duty, bool fwd_on_a)
{
    float    mag = (duty < 0.0f) ? -duty : duty;
    uint16_t lvl;
    bool     use_a;

    if (mag > 1.0f)
    {
        mag = 1.0f;
    }
    lvl   = (uint16_t)(mag * (float)g_pwm_top);
    use_a = ((duty >= 0.0f) == fwd_on_a);

    pwm_set_gpio_level(pin_a, use_a ? lvl : 0U);
    pwm_set_gpio_level(pin_b, use_a ? 0U : lvl);
}

void hal_motor_set(float duty_left, float duty_right)
{
    drive_channel(PIN_M1A, PIN_M1B, duty_left,  (0 != LEFT_FWD_ON_A));
    drive_channel(PIN_M2A, PIN_M2B, duty_right, (0 != RIGHT_FWD_ON_A));
}

void hal_motor_brake(void)
{
    pwm_set_gpio_level(PIN_M1A, 0U);
    pwm_set_gpio_level(PIN_M1B, 0U);
    pwm_set_gpio_level(PIN_M2A, 0U);
    pwm_set_gpio_level(PIN_M2B, 0U);
}

/* ------------------------------------------------------------------ */
/* Servo: 50 Hz, 1 us per tick, 500-2500 us pulse for 0-180 deg        */
/* ------------------------------------------------------------------ */
void hal_servo_init(void)
{
    pwm_config cfg = pwm_get_default_config();

    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&cfg, 19999U);
    pwm_init(pwm_gpio_to_slice_num(PIN_SERVO), &cfg, true);
    hal_servo_set_deg(SERVO_CENTRE_DEG);
}

void hal_servo_set_deg(int32_t deg)
{
    if (deg < 0)   { deg = 0;   }
    if (deg > 180) { deg = 180; }
    pwm_set_gpio_level(PIN_SERVO, (uint16_t)(500 + ((deg * 2000) / 180)));
}

/* ------------------------------------------------------------------ */
/* Buttons and NeoPixel                                                */
/* ------------------------------------------------------------------ */
void hal_buttons_init(void)
{
    input_pin(PIN_BTN_START, true);
    input_pin(PIN_BTN_CAL, true);
}

bool hal_btn_start_raw(void) { return !gpio_get(PIN_BTN_START); }
bool hal_btn_cal_raw(void)   { return !gpio_get(PIN_BTN_CAL); }

static PIO  g_led_pio;
static uint g_led_sm;
static bool g_led_ok;

void hal_led_init(void)
{
    uint offset;

    g_led_ok = pio_claim_free_sm_and_add_program(&ws2812_program,
                                                 &g_led_pio, &g_led_sm, &offset);
    if (g_led_ok)
    {
        ws2812_program_init(g_led_pio, g_led_sm, offset, PIN_NEOPIXEL, 800000.0f);
    }
    hal_led_set(LED_OFF);
}

void hal_led_set(led_colour_t colour)
{
    static const uint32_t grb[5] =
    {
        0x000000U, /* off   */
        0x000020U, /* blue  */
        0x200000U, /* green */
        0x182000U, /* amber */
        0x002000U  /* red   */
    };
    uint32_t i;

    if (!g_led_ok)
    {
        return;
    }
    /* two on-board pixels */
    for (i = 0U; i < 2U; i++)
    {
        pio_sm_put_blocking(g_led_pio, g_led_sm, grb[colour] << 8);
    }
}

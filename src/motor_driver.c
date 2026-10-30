/** @file    motor_driver.c
 *  @brief   Motor driver (Buddy 2), see motor_driver.h.
 *
 *  Robo Pico truth table per channel: A/B = H/L or L/H drives, L/L brakes.
 *  Encoders are decoded x4 in the shared GPIO interrupt (hal.c calls
 *  motor_driver_encoder_isr()); counts and edge times have a single
 *  writer (the interrupt) and are read as aligned 32-bit words.
 */
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/resets.h"
#include "hardware/sync.h"

#include "car_config.h"
#include "motor_driver.h"

#define QUAD_STATES          (16U)
#define STATE_BITS           (2U)
#define ENC_PIN_COUNT        (4U)

/* index = (previous_state << 2) | new_state, state = (A << 1) | B */
static int8_t const g_quad_table[QUAD_STATES] =
{
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

static uint16_t          g_pwm_top     = 0U;
static volatile bool     g_b_inhibit   = false;
static volatile int32_t  g_count_l     = 0;
static volatile int32_t  g_count_r     = 0;
static volatile uint32_t g_edge_ms_l   = 0U;
static volatile uint32_t g_edge_ms_r   = 0U;
static uint32_t          g_prev_l      = 0U;   /* ISR only               */
static uint32_t          g_prev_r      = 0U;   /* ISR only               */

static void     drive_channel (uint32_t pin_a, uint32_t pin_b,
                               int32_t duty_pm, bool b_fwd_on_a);
static uint32_t encoder_state (uint32_t pin_a, uint32_t pin_b);
static bool     encoder_event (uint32_t pin_a, uint32_t pin_b);

/*!
 * @brief  Set up motor PWM (brakes the motors) and the encoder inputs.
 *         Call first in usermain(), so the wheels cannot move during boot.
 */
void
motor_driver_init (void)
{
    static uint32_t const enc_pins[ENC_PIN_COUNT] =
        { PIN_ENC_R_A, PIN_ENC_R_B, PIN_ENC_L_A, PIN_ENC_L_B };
    pwm_config     cfg    = pwm_get_default_config();
    uint32_t const sys_hz = clock_get_hz(clk_sys);
    uint32_t       idx    = 0U;

    /* NOTE: The template releases PWM from reset only when USE_PTMR=1
     * (it is 0), and the SDK runtime init does not run here, so without
     * this every PWM register write would be ignored. */
    unreset_block_num_wait_blocking(RESET_PWM);

    /* 125 MHz / 20 kHz - 1 = 6249, so it fits a uint16_t. */
    g_pwm_top = (uint16_t) ((sys_hz / MOTOR_PWM_HZ) - 1U);
    pwm_config_set_wrap(&cfg, g_pwm_top);
    pwm_config_set_clkdiv(&cfg, 1.0f);

    gpio_set_function(PIN_M1A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_M1B, GPIO_FUNC_PWM);
    gpio_set_function(PIN_M2A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_M2B, GPIO_FUNC_PWM);
    pwm_init(pwm_gpio_to_slice_num(PIN_M1A), &cfg, true);
    pwm_init(pwm_gpio_to_slice_num(PIN_M2A), &cfg, true);
    motor_driver_brake();

    for (idx = 0U; idx < ENC_PIN_COUNT; idx++)
    {
        gpio_init(enc_pins[idx]);
        gpio_set_dir(enc_pins[idx], GPIO_IN);
        gpio_pull_up(enc_pins[idx]);
        gpio_set_irq_enabled(enc_pins[idx],
                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }

    g_prev_r = encoder_state(PIN_ENC_R_A, PIN_ENC_R_B);
    g_prev_l = encoder_state(PIN_ENC_L_A, PIN_ENC_L_B);
}

/*!
 * @brief  Drive both motors. While the Safety inhibit is set the motors
 *         are braked instead.
 *
 *  NOTE: Safety may set the inhibit on the other core while this runs.
 *  Safety sets the flag before it brakes, so the check after the PWM
 *  write always sees the flag if Safety's brake came first, and brakes
 *  again. The barriers keep that order.
 *
 * @param  duty_l_pm  Left duty, per mille (clamped to +/-1000).
 * @param  duty_r_pm  Right duty, per mille (clamped to +/-1000).
 */
void
motor_driver_set (int32_t duty_l_pm, int32_t duty_r_pm)
{
    if (g_b_inhibit)
    {
        motor_driver_brake();
    }
    else
    {
        drive_channel(PIN_M1A, PIN_M1B, duty_l_pm, (0 != LEFT_FWD_ON_A));
        drive_channel(PIN_M2A, PIN_M2B, duty_r_pm, (0 != RIGHT_FWD_ON_A));
        __dmb();

        if (g_b_inhibit)
        {
            motor_driver_brake();
        }
    }
}

/*!
 * @brief  Brake both motors (all four inputs LOW). Safe from any task.
 */
void
motor_driver_brake (void)
{
    pwm_set_gpio_level(PIN_M1A, 0U);
    pwm_set_gpio_level(PIN_M1B, 0U);
    pwm_set_gpio_level(PIN_M2A, 0U);
    pwm_set_gpio_level(PIN_M2B, 0U);
}

/*!
 * @brief  Set or clear the Safety motor-inhibit latch. Setting it also
 *         brakes, after the flag is visible to the other core.
 * @param  b_inhibit  True to block all motor drive.
 */
void
motor_driver_inhibit (bool b_inhibit)
{
    g_b_inhibit = b_inhibit;
    __dmb();

    if (b_inhibit)
    {
        motor_driver_brake();
    }
}

/*!
 * @brief  Report the motor-inhibit latch.
 * @return True while Safety blocks motor drive.
 */
bool
motor_driver_is_inhibited (void)
{
    return (g_b_inhibit);
}

/*!
 * @brief  Left encoder x4 counts since boot (raw direction).
 * @return Signed count.
 */
int32_t
motor_driver_count_left (void)
{
    return (g_count_l);
}

/*!
 * @brief  Right encoder x4 counts since boot (raw direction).
 * @return Signed count.
 */
int32_t
motor_driver_count_right (void)
{
    return (g_count_r);
}

/*!
 * @brief  Time of the last left encoder edge.
 * @return Milliseconds since boot (same clock as systime_now_ms()).
 */
uint32_t
motor_driver_edge_ms_left (void)
{
    return (g_edge_ms_l);
}

/*!
 * @brief  Time of the last right encoder edge.
 * @return Milliseconds since boot (same clock as systime_now_ms()).
 */
uint32_t
motor_driver_edge_ms_right (void)
{
    return (g_edge_ms_r);
}

/*!
 * @brief  Encoder part of the shared GPIO interrupt: acknowledge and
 *         decode the edges of both encoders.
 * @param  now_ms  Interrupt time, from the 64-bit clock so edge times do
 *                 not jump when the 32-bit microsecond counter wraps.
 */
void
motor_driver_encoder_isr (uint32_t now_ms)
{
    if (encoder_event(PIN_ENC_R_A, PIN_ENC_R_B))
    {
        uint32_t const state = encoder_state(PIN_ENC_R_A, PIN_ENC_R_B);

        g_count_r  += g_quad_table[(g_prev_r << STATE_BITS) | state];
        g_prev_r    = state;
        g_edge_ms_r = now_ms;
    }

    if (encoder_event(PIN_ENC_L_A, PIN_ENC_L_B))
    {
        uint32_t const state = encoder_state(PIN_ENC_L_A, PIN_ENC_L_B);

        g_count_l  += g_quad_table[(g_prev_l << STATE_BITS) | state];
        g_prev_l    = state;
        g_edge_ms_l = now_ms;
    }
}

/*!
 * @brief  Set one H-bridge channel.
 * @param  pin_a       Channel input A.
 * @param  pin_b       Channel input B.
 * @param  duty_pm     Duty, per mille; the sign selects the direction.
 * @param  b_fwd_on_a  True if PWM on input A drives the wheel forward.
 */
static void
drive_channel (uint32_t pin_a, uint32_t pin_b, int32_t duty_pm,
               bool b_fwd_on_a)
{
    int32_t const magnitude = (duty_pm < 0) ? -duty_pm : duty_pm;
    int32_t const limited   = (magnitude > MOTOR_DUTY_MAX_PM) ?
                              MOTOR_DUTY_MAX_PM : magnitude;
    /* limited is 0..1000, so the product fits and the level <= top. */
    uint16_t const level    = (uint16_t) (((uint32_t) limited * g_pwm_top) /
                                          (uint32_t) MOTOR_DUTY_MAX_PM);
    bool const     b_use_a  = ((duty_pm >= 0) == b_fwd_on_a);

    pwm_set_gpio_level(pin_a, b_use_a ? level : 0U);
    pwm_set_gpio_level(pin_b, b_use_a ? 0U : level);
}

/*!
 * @brief  Current quadrature state of one encoder.
 * @param  pin_a  Encoder channel A.
 * @param  pin_b  Encoder channel B.
 * @return (A << 1) | B.
 */
static uint32_t
encoder_state (uint32_t pin_a, uint32_t pin_b)
{
    uint32_t const level_a = gpio_get(pin_a) ? 1U : 0U;
    uint32_t const level_b = gpio_get(pin_b) ? 1U : 0U;

    return ((level_a << 1) | level_b);
}

/*!
 * @brief  Acknowledge the edge interrupts of one encoder.
 * @param  pin_a  Encoder channel A.
 * @param  pin_b  Encoder channel B.
 * @return True if either channel had an edge.
 */
static bool
encoder_event (uint32_t pin_a, uint32_t pin_b)
{
    uint32_t const ev_a = gpio_get_irq_event_mask(pin_a);
    uint32_t const ev_b = gpio_get_irq_event_mask(pin_b);

    gpio_acknowledge_irq(pin_a, ev_a);
    gpio_acknowledge_irq(pin_b, ev_b);

    return (0U != (ev_a | ev_b));
}

/*** end of file ***/

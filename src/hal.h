/**
 * @file    hal.h
 * @brief   Low-level drivers: GPIO edge ISR (encoders, barcode, echo),
 *          motor PWM, servo PWM, buttons, NeoPixel.
 */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

/* ---- GPIO edge ISR (IO_IRQ_BANK0) -------------------------------- */
/* Kernel side (hal_irq.c): configure pins and register the ISR with
 * micro T-Kernel. Call after g_evt_flg exists.                        */
void     hal_isr_init(void);

/* SDK side (hal.c), used only by hal_irq.c: */
#define HAL_SIG_BARCODE  (1U << 0)   /* a barcode edge was stored        */
#define HAL_SIG_ECHO     (1U << 1)   /* an echo pulse finished           */
void     hal_gpio_init(void);          /* pins + per-pin edge enables    */
uint32_t hal_gpio_irq_service(void);   /* ISR body; returns HAL_SIG_*    */

/* Encoders: signed x4 counts since boot (forward = positive after
 * motion.c applies the per-wheel sign). Safe to call from a task.     */
int32_t  hal_enc_left(void);
int32_t  hal_enc_right(void);
uint32_t hal_enc_last_edge_ms_left(void);
uint32_t hal_enc_last_edge_ms_right(void);

/* Barcode: copy edges captured since the last call. Each entry is a
 * 1 us timestamp; bit 0 of the timestamp is replaced by the level
 * AFTER the edge (1 = black).  Returns number copied.                 */
uint32_t hal_bar_take(uint32_t *p_buf, uint32_t max);
void     hal_bar_reset(void);

/* Ultrasonic: start a ping, then wait on EVT_ECHO_DONE and read.     */
void     hal_us_trigger(void);
bool     hal_us_echo_us(uint32_t *p_us);

/* ---- Motors (Robo Pico on-board driver) ---------------------------- */
void     hal_motor_init(void);
/* duty -1.0 .. +1.0, positive = robot forward for that wheel          */
void     hal_motor_set(float duty_left, float duty_right);
/* Both inputs LOW = brake. Callable from any task (Safety Monitor).   */
void     hal_motor_brake(void);

/* ---- Servo (SG90, 50 Hz) ------------------------------------------- */
void     hal_servo_init(void);
void     hal_servo_set_deg(int32_t deg);

/* ---- Buttons / NeoPixel ------------------------------------------- */
void     hal_buttons_init(void);
bool     hal_btn_start_raw(void);    /* true while pressed             */
bool     hal_btn_cal_raw(void);

typedef enum
{
    LED_OFF = 0,
    LED_BLUE,     /* READY / DONE  */
    LED_GREEN,    /* RUN           */
    LED_AMBER,    /* WiFi offline  */
    LED_RED       /* FAULT         */
} led_colour_t;

void     hal_led_init(void);
void     hal_led_set(led_colour_t colour);

#endif /* HAL_H */

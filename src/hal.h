/** @file    hal.h
 *  @brief   Board support (team, SDK side): the shared GPIO interrupt
 *           dispatcher, barcode edge capture, ultrasonic trigger / echo,
 *           servo PWM, buttons and the NeoPixel status LED.
 *
 *  Motor PWM and the wheel encoders are in motor_driver.h (Buddy 2).
 *  The kernel side of the interrupt is in hal_irq.h.
 */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

/* Signals returned by hal_gpio_irq_service(). */
#define HAL_SIG_BARCODE      (1U << 0)   /* a barcode edge was stored     */
#define HAL_SIG_ECHO         (1U << 1)   /* an echo pulse finished        */

/** NeoPixel status colours (Week 6). */
typedef enum
{
    HAL_LED_OFF = 0,
    HAL_LED_BLUE,     /* READY / DONE                                     */
    HAL_LED_GREEN,    /* running, WiFi up                                 */
    HAL_LED_AMBER,    /* running, WiFi offline                            */
    HAL_LED_RED,      /* FAULT                                            */
    HAL_LED_COUNT
} hal_led_colour_t;

void     hal_gpio_init (void);
uint32_t hal_gpio_irq_service (void);

uint32_t hal_bar_take (uint32_t * p_buf, int32_t * p_cl, int32_t * p_cr,
                       uint32_t max);
uint32_t hal_bar_edges (void);
uint32_t hal_bar_dropped (void);
void     hal_us_trigger (void);
bool     hal_us_echo_us (uint32_t * p_us);

void     hal_servo_init (void);
void     hal_servo_set_deg (uint32_t angle_deg);

void     hal_buttons_init (void);
bool     hal_btn_start_raw (void);
bool     hal_btn_cal_raw (void);

void     hal_led_init (void);
void     hal_led_set (hal_led_colour_t colour);

#endif /* HAL_H */

/*** end of file ***/

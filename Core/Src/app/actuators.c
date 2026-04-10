/**
 * @file    actuators.c
 * @brief   Heater/fan PWM and indicator LED output implementation.
 *
 * This module touches MX_TIM1_Init's handles directly via the tim.h
 * extern declaration. We do not duplicate timer config here — MX_TIM1_Init
 * is the canonical place for ARR, prescaler, polarity, and channel mode.
 * Our job is only to start the PWM and write CCR/GPIO at runtime.
 *
 * Output channel assignment (from main.h pin map):
 *   TIM1_CH1 = PA8  = OUT_HEATER  (gate driver)
 *   TIM1_CH2 = PA9  = OUT_FAN     (PWM-controlled fan)
 *   PB0      = OUT_LED_R1
 *   PB1      = OUT_LED_R2
 */

#include <stdint.h>
#include "main.h"
#include "tim.h"
#include "stm32f1xx_hal.h"
#include "app/actuators.h"
#include "app/sk6812.h"

/* TIM1 ARR is 1000 (see Core/Src/tim.c MX_TIM1_Init). PID output and
 * heater duty share this scale 1:1. The fan duty is in percent so we
 * multiply by 10 to land in the same CCR range. */
#define ACTUATORS_PWM_PERIOD   1000u

/* Internal helper: clamp duty to the PWM range. Avoids signed-overflow
 * shenanigans if a caller passes a too-large uint16. */
static inline uint16_t clamp_duty(uint16_t duty)
{
    return (duty > ACTUATORS_PWM_PERIOD) ? ACTUATORS_PWM_PERIOD : duty;
}

void actuators_init(void)
{
    /* Force CCR to 0 BEFORE starting PWM so the first edge is low.
     * Without this, the channel would output the previous CCR value
     * (whatever was there at boot, often 0 but not guaranteed). */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    /* Start PWM on both channels. After this point, the timer hardware
     * generates the PWM waveform autonomously and we only need to write
     * CCR to change the duty cycle. */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /* LEDs all off until the first explicit pattern command. */
    HAL_GPIO_WritePin(OUT_LED_R1_GPIO_Port, OUT_LED_R1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OUT_LED_R2_GPIO_Port, OUT_LED_R2_Pin, GPIO_PIN_RESET);

    /* SK6812MINI RGB LEDs — start dark. */
    sk6812_init();
}

void actuators_set_heater_duty(uint16_t duty_0_1000)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, clamp_duty(duty_0_1000));
}

void actuators_set_fan_duty_pct(uint8_t pct)
{
    if (pct > 100) pct = 100;
    /* Convert 0..100 percent to 0..1000 CCR. */
    uint16_t ccr = (uint16_t)pct * 10u;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
}

void actuators_set_led_pattern(led_pattern_t pat)
{
    /* GPIO indicator LEDs (PB0, PB1) — solid on/off. */
    GPIO_PinState r1 = GPIO_PIN_RESET;
    GPIO_PinState r2 = GPIO_PIN_RESET;

    /* SK6812MINI RGB colors for each pattern.
     * Both LEDs get the same color (could be individualized later). */
    sk6812_color_t rgb = {0, 0, 0};

    switch (pat) {
    case LED_OFF:
        /* all off */
        break;
    case LED_FADE_YELLOW:
        r1 = GPIO_PIN_SET;
        rgb = (sk6812_color_t){64, 32, 0};    /* dim amber */
        break;
    case LED_RAMP_YELLOW:
        r2 = GPIO_PIN_SET;
        rgb = (sk6812_color_t){255, 128, 0};   /* bright orange */
        break;
    case LED_FLASH_RED:
        r1 = GPIO_PIN_SET;
        r2 = GPIO_PIN_SET;
        rgb = (sk6812_color_t){255, 0, 0};     /* solid red */
        break;
    default:
        break;
    }

    HAL_GPIO_WritePin(OUT_LED_R1_GPIO_Port, OUT_LED_R1_Pin, r1);
    HAL_GPIO_WritePin(OUT_LED_R2_GPIO_Port, OUT_LED_R2_Pin, r2);

    /* SK6812: set both LEDs to the same color + push via DMA. */
    sk6812_set(0, rgb);
    sk6812_set(1, rgb);
    sk6812_update();
}

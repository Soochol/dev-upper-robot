/**
 * @file    actuators.h
 * @brief   Heater PWM, fan PWM, and indicator LED output API.
 *
 * Owns the physical outputs that close the PID control loop:
 *
 *   TIM1_CH1 → heater PWM (PA8, OD via gate driver)   ← PID output
 *   TIM1_CH2 → fan PWM    (PA9)                       ← FSM state LUT
 *   PB0      → indicator LED R1 (GPIO push-pull)
 *   PB1      → indicator LED R2 (GPIO push-pull)
 *
 * The TIM2_CH2 RGB LED (WS2812 on PB3) is intentionally NOT covered
 * by this module in Phase 3 — it needs DMA frame composition that
 * arrives in Phase 6 along with the rest of the polish work.
 *
 * Single-owner: T_PID is the only task that calls these functions.
 * No mutex protection — direct register writes are atomic on Cortex-M3.
 */

#ifndef APP_ACTUATORS_H
#define APP_ACTUATORS_H

#include <stdint.h>

/* ========================================================================
 * LED indicator patterns
 * ========================================================================
 * Used by both state_table.c (LUT definition) and actuators.c (GPIO
 * output mapping). Adding a new pattern requires both adding it here
 * and extending actuators_set_led_pattern() in actuators.c. */

typedef enum {
    LED_OFF               = 0,
    LED_FADE_YELLOW       = 1,   /* FORCE_DOWN: cooling, solid           */
    LED_RAMP_YELLOW       = 2,   /* FORCE_UP:   heating, solid           */
    LED_FLASH_RED         = 3,   /* FAULT:      alarm,   1 Hz blink      */
    LED_FADE_YELLOW_BLINK = 4,   /* FORCE_DOWN: cooling in progress, blink */
    LED_RAMP_YELLOW_BLINK = 5,   /* FORCE_UP:   heating in progress, blink */
} led_pattern_t;

/* ========================================================================
 * Lifecycle
 * ======================================================================== */

/**
 * @brief  Start TIM1 PWM channels and force outputs to safe defaults.
 *
 * MX_TIM1_Init in tim.c configures the timer registers but does NOT
 * call HAL_TIM_PWM_Start — the channels stay idle until something
 * starts them. We do that here, then immediately write 0 to both CCRs
 * so heater and fan are off until T_PID issues real commands.
 *
 * Must be called once at T_PID startup, before the periodic loop.
 */
void actuators_init(void);

/* ========================================================================
 * Heater (TIM1_CH1, PA8)
 * ======================================================================== */

/**
 * @brief  Set the heater PWM duty cycle.
 * @param  duty_0_1000  raw compare value, 0..1000 (TIM1 ARR = 1000)
 *                      0  → fully off
 *                      1000 → fully on (100% duty)
 *                      Values above 1000 are clamped to 1000.
 *
 * The PID compute output range matches this scale (PID_OUTPUT_MIN..MAX
 * in config.h are 0..1000) so PID results can be passed directly with
 * no unit conversion.
 */
void actuators_set_heater_duty(uint16_t duty_0_1000);

/* ========================================================================
 * Fan (PA9 inverted PWM + PA10 enable)
 * ======================================================================== */

/**
 * @brief  Set the fan speed in percent.
 * @param  pct  0..100; values above 100 are clamped.
 *              0 = fan off (PA10 LOW), 100 = full speed.
 *
 * PWM polarity is inverted in hardware: CCR=0 → full speed, CCR=1000
 * → minimum. This function handles the inversion internally.
 * PA10 serves as master enable (HIGH when pct > 0).
 */
void actuators_set_fan_duty_pct(uint8_t pct);

/* ========================================================================
 * Indicator LEDs (PB0, PB1)
 * ======================================================================== */

/**
 * @brief  Apply an LED pattern to the R1/R2 GPIO pair.
 *
 * Phase 3b: only solid on/off mappings. No blinking — that requires
 * a tick-based state and arrives in Phase 6 alongside WS2812.
 *
 * Pattern → GPIO mapping:
 *   LED_OFF          → R1=0, R2=0
 *   LED_FADE_YELLOW  → R1=1, R2=0  (cool-down indicator)
 *   LED_RAMP_YELLOW  → R1=0, R2=1  (heat-up indicator)
 *   LED_FLASH_RED    → R1=1, R2=1  (alarm; Phase 6 will turn this into blink)
 */
void actuators_set_led_pattern(led_pattern_t pat);

#endif /* APP_ACTUATORS_H */

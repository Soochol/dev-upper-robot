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
 * Single-owner during normal operation: T_PID drives all outputs.
 * Exception: actuators_emergency_off() may be called from T_WDG ONLY
 * when a canary stall is detected. Race-free because:
 *   - CCR writes and GPIO BSRR writes are atomic on Cortex-M3
 *   - emergency_off does NOT touch the LED static state
 *   - T_PID gates its own actuator writes via g_failsafe_active so it
 *     cannot overwrite the emergency-off after T_WDG sets the flag
 *   - T_WDG runs at higher priority (6 > 4), so the flag-set + write
 *     pair preempts T_PID atomically from T_PID's perspective
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
    LED_FADE_YELLOW_BLINK     = 4,   /* FORCE_DOWN: cooling in progress, blink */
    LED_RAMP_YELLOW_BLINK     = 5,   /* FORCE_UP:   heating in progress, blink */
    LED_RECORDING_BLINK       = 6,   /* DATA_COLLECT recording: magenta 1 Hz    */
    LED_RECORDING_ERROR_BLINK = 7,   /* DATA_COLLECT SD write fail: magenta fast */
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
 * Emergency stop (T_WDG only)
 * ======================================================================== */

/**
 * @brief  Force heater PWM and fan power OFF immediately. LED untouched.
 *
 * Called from T_WDG when a canary stall is detected, during the ~400 ms
 * grace window before IWDG fires. Touches only TIM1 CCRx and the PA10
 * fan-power GPIO — no static state, no mutex, no blocking.
 *
 * Must NOT be used as a normal control path. Pair with setting
 * g_failsafe_active so T_PID stops writing actuators on its next tick.
 */
void actuators_emergency_off(void);

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

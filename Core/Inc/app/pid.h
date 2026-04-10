/**
 * @file    pid.h
 * @brief   Single-input single-output PID controller.
 *
 * Algorithm: standard textbook PID with two safety enhancements:
 *
 *   1. Clamp anti-windup. When the computed output saturates against
 *      output_max or output_min, the integrator is NOT updated for that
 *      cycle. This prevents the integrator from accumulating during
 *      saturation periods (which would otherwise cause large overshoot
 *      when the system finally responds).
 *
 *   2. Bumpless setpoint transfer. The setpoint is read fresh from the
 *      caller each compute call (not stored in the struct), and the
 *      integrator is preserved across setpoint changes. A 60 °C → 32 °C
 *      step thus produces a smooth transition with no integrator reset.
 *
 * No HAL or FreeRTOS dependency — host-testable.
 */

#ifndef APP_PID_H
#define APP_PID_H

#include <stdint.h>

typedef struct {
    /* Tunable gains. Set once at init or adjusted at runtime via the
     * pid_set_gains helper. */
    float kp;
    float ki;
    float kd;

    /* Output saturation limits. The compute function clamps to this
     * range and uses the clamping decision to drive anti-windup. */
    float output_min;
    float output_max;

    /* Internal state — DO NOT poke from outside. Use pid_reset to clear. */
    float integral;
    float last_error;
    uint8_t has_prev_error;   /* false on first call, prevents derivative kick */
} pid_t;

/**
 * @brief  Initialize a PID instance with the given gains and limits.
 *         Internal state is cleared (equivalent to pid_reset).
 */
void pid_init(pid_t *p,
              float kp, float ki, float kd,
              float output_min, float output_max);

/**
 * @brief  Reset internal state (integrator, last error). Use only when
 *         the controlled system enters a state where the previous PID
 *         history is no longer relevant — e.g. FAULT recovery.
 *
 *         Do NOT call this on every FORCE_UP ↔ FORCE_DOWN transition;
 *         that would defeat bumpless transfer.
 */
void pid_reset(pid_t *p);

/**
 * @brief  Compute one PID step.
 * @param  setpoint     desired value (e.g. target temperature in °C)
 * @param  measurement  current measured value
 * @param  dt           sample period in seconds (typically PID_DT_S)
 * @return saturated control output, clamped to [output_min, output_max]
 */
float pid_compute(pid_t *p, float setpoint, float measurement, float dt);

#endif /* APP_PID_H */

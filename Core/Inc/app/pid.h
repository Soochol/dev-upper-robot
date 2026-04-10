/**
 * @file    pid.h
 * @brief   Single-input single-output PID controller.
 *
 * Algorithm: positional-form PID with three enhancements:
 *
 *   1. Clamp anti-windup. When the computed output saturates against
 *      output_max or output_min, the integrator is NOT updated for that
 *      cycle (unless the error is unwinding the saturation).
 *
 *   2. Derivative on measurement (not error). Eliminates derivative
 *      kick when the setpoint changes abruptly (e.g. 60 °C → 25 °C).
 *      Combined with a first-order low-pass filter (Tf = Td/N) to
 *      suppress sensor noise amplification.
 *
 *   3. Setpoint weighting (b parameter). P term uses
 *      Kp * (b * setpoint - measurement) instead of Kp * error.
 *      b < 1 reduces overshoot on setpoint steps while preserving
 *      full disturbance rejection.
 *
 * No HAL or FreeRTOS dependency — host-testable.
 */

#ifndef APP_PID_H
#define APP_PID_H

#include <stdint.h>

typedef struct {
    /* Tunable gains. Set once at init or adjusted at runtime via
     * pid_set_gains. */
    float kp;
    float ki;
    float kd;

    /* Setpoint weight for P term: p_term = Kp * (b * sp - meas).
     * b = 1.0 is standard PID. b = 0.5–0.7 reduces overshoot on
     * setpoint steps. Default: 1.0 (set via pid_set_tuning). */
    float b;

    /* Derivative filter coefficient N = Td / Tf. Higher N = less
     * filtering, faster D response. Typical range: 8–20.
     * Default: 10.0 (set via pid_set_tuning). */
    float n;

    /* Output saturation limits. The compute function clamps to this
     * range and uses the clamping decision to drive anti-windup. */
    float output_min;
    float output_max;

    /* Internal state — DO NOT poke from outside. Use pid_reset to
     * clear. */
    float integral;
    float last_meas;      /* previous measurement (derivative on meas) */
    float d_filt;         /* filtered derivative state */
    uint8_t has_prev;     /* false on first call, prevents derivative kick */
} pid_ctrl_t;

/**
 * @brief  Initialize a PID instance with the given gains and limits.
 *         Sets b = 1.0, n = 10.0 as defaults. Internal state is cleared.
 */
void pid_init(pid_ctrl_t *p,
              float kp, float ki, float kd,
              float output_min, float output_max);

/**
 * @brief  Reset internal state (integrator, derivative filter, last
 *         measurement). Use only when the controlled system enters a
 *         state where the previous PID history is no longer relevant
 *         — e.g. FAULT recovery.
 *
 *         Do NOT call this on every FORCE_UP ↔ FORCE_DOWN transition;
 *         that would defeat bumpless transfer.
 */
void pid_reset(pid_ctrl_t *p);

/**
 * @brief  Compute one PID step.
 * @param  setpoint     desired value (e.g. target temperature in °C)
 * @param  measurement  current measured value
 * @param  dt           sample period in seconds (typically PID_DT_S)
 * @return saturated control output, clamped to [output_min, output_max]
 */
float pid_compute(pid_ctrl_t *p, float setpoint, float measurement, float dt);

/**
 * @brief  Update gains at runtime (e.g. for gain scheduling).
 */
void pid_set_gains(pid_ctrl_t *p, float kp, float ki, float kd);

/**
 * @brief  Set setpoint weight (b) and derivative filter coefficient (N).
 */
void pid_set_tuning(pid_ctrl_t *p, float b, float n);

#endif /* APP_PID_H */

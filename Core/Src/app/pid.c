/**
 * @file    pid.c
 * @brief   Positional-form PID with derivative-on-measurement, derivative
 *          filter, setpoint weighting, and clamp anti-windup.
 *
 * Implementation note: float math on Cortex-M3 (no FPU) goes through
 * libgcc soft-float. Per-call cost for this PID is roughly 80–120 cycles
 * (~1.5 µs at 72 MHz) — negligible compared to the 50 ms PID period.
 *
 * Key differences from a textbook PID:
 *
 *   - Derivative on measurement (not error): the D term uses
 *     -Kd * d(measurement)/dt instead of Kd * d(error)/dt. This
 *     eliminates the "derivative kick" spike when the setpoint changes
 *     abruptly. The sign is negative because a rising measurement
 *     (approaching setpoint from below) should reduce the output.
 *
 *   - Derivative filter: a first-order low-pass on the D term with
 *     time constant Tf = Kd / (Kp * N). This prevents sensor noise
 *     from being amplified by the derivative. Typical N = 8–20.
 *
 *   - Setpoint weighting: the P term uses Kp * (b * sp - meas)
 *     instead of Kp * (sp - meas). With b < 1, the proportional
 *     response to setpoint changes is reduced while disturbance
 *     rejection remains at full strength.
 */

#include "app/pid.h"

void pid_init(pid_ctrl_t *p,
              float kp, float ki, float kd,
              float output_min, float output_max)
{
    p->kp         = kp;
    p->ki         = ki;
    p->kd         = kd;
    p->b          = 1.0f;
    p->n          = 10.0f;
    p->output_min = output_min;
    p->output_max = output_max;
    pid_reset(p);
}

void pid_reset(pid_ctrl_t *p)
{
    p->integral  = 0.0f;
    p->last_meas = 0.0f;
    p->d_filt    = 0.0f;
    p->has_prev  = 0u;
}

void pid_set_gains(pid_ctrl_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}

void pid_set_tuning(pid_ctrl_t *p, float b, float n)
{
    p->b = b;
    p->n = n;
}

float pid_compute(pid_ctrl_t *p, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* --- P term with setpoint weighting ---
     * p_term = Kp * (b * setpoint - measurement)
     * When b = 1.0 this equals Kp * error (standard PID). */
    float p_term = p->kp * (p->b * setpoint - measurement);

    /* --- I term: tentative integrator update ---
     * We don't commit to p->integral until we know whether the output
     * saturates (clamp anti-windup). */
    float new_integral = p->integral + error * dt;
    float i_term = p->ki * new_integral;

    /* --- D term: derivative on measurement with first-order filter ---
     * Suppress entirely on the first call (no previous measurement). */
    float d_term = 0.0f;
    if (p->has_prev && dt > 0.0f) {
        float d_meas = -(measurement - p->last_meas) / dt;

        /* First-order low-pass: Tf = Kd / (Kp * N).
         * d_filt = (Tf * d_filt_prev + Kd * d_meas * dt) / (Tf + dt)
         * Guard against Kp == 0 or N == 0: skip filter, use raw. */
        if (p->kp != 0.0f && p->n != 0.0f) {
            float tf = p->kd / (p->kp * p->n);
            p->d_filt = (tf * p->d_filt + p->kd * d_meas * dt) / (tf + dt);
            d_term = p->d_filt;
        } else {
            d_term = p->kd * d_meas;
        }
    }

    float output = p_term + i_term + d_term;

    /* --- Saturate and anti-windup ---
     * If output saturates AND the integrator is pushing in the
     * saturation direction, freeze it. Allow unwinding. */
    if (output > p->output_max) {
        output = p->output_max;
        if (error <= 0.0f)
            p->integral = new_integral;
    } else if (output < p->output_min) {
        output = p->output_min;
        if (error >= 0.0f)
            p->integral = new_integral;
    } else {
        p->integral = new_integral;
    }

    p->last_meas = measurement;
    p->has_prev  = 1u;
    return output;
}

/**
 * @file    pid.c
 * @brief   Standard PID with clamp anti-windup and bumpless transfer.
 *
 * Implementation note: float math on Cortex-M3 (no FPU) goes through
 * libgcc soft-float. Per-call cost for our 4-multiply / 5-add PID is
 * roughly 80 cycles (~1.1 µs at 72 MHz) — utterly negligible compared
 * to the 50 ms PID period. Using float keeps the gain tuning intuitive
 * (kp=2.0, ki=0.1) and avoids fixed-point Q-format bookkeeping.
 *
 * The first compute call after pid_init or pid_reset has no previous
 * error value, so the derivative term is forced to 0 to prevent a
 * "derivative kick" — a one-time large output spike caused by computing
 * (current_error - 0) / dt. The has_prev_error flag tracks this.
 */

#include "app/pid.h"

void pid_init(pid_t *p,
              float kp, float ki, float kd,
              float output_min, float output_max)
{
    p->kp         = kp;
    p->ki         = ki;
    p->kd         = kd;
    p->output_min = output_min;
    p->output_max = output_max;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    p->integral       = 0.0f;
    p->last_error     = 0.0f;
    p->has_prev_error = 0u;
}

float pid_compute(pid_t *p, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* Proportional term — straightforward, no state. */
    float p_term = p->kp * error;

    /* Tentative integrator update. We don't commit this to p->integral
     * until we know whether the output saturates (clamp anti-windup). */
    float new_integral = p->integral + error * dt;
    float i_term = p->ki * new_integral;

    /* Derivative term. Suppress on the first call so an arbitrary
     * (current_error - 0) does not produce a spike. */
    float d_term = 0.0f;
    if (p->has_prev_error && dt > 0.0f) {
        d_term = p->kd * (error - p->last_error) / dt;
    }

    float output = p_term + i_term + d_term;

    /* Saturate and decide whether to commit the integrator update.
     * If the output would saturate against either limit AND the
     * integrator is pushing in the saturation direction, freeze it. */
    if (output > p->output_max) {
        output = p->output_max;
        if (error <= 0.0f) {
            /* Error is negative (below setpoint) — integrator is
             * unwinding, allow it to update so we can recover. */
            p->integral = new_integral;
        }
        /* else: error positive, integrator pushing up against the
         * upper limit — freeze to prevent windup. */
    } else if (output < p->output_min) {
        output = p->output_min;
        if (error >= 0.0f) {
            p->integral = new_integral;
        }
    } else {
        /* Output in linear range — always update. */
        p->integral = new_integral;
    }

    p->last_error     = error;
    p->has_prev_error = 1u;
    return output;
}

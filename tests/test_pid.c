/**
 * @file    test_pid.c
 * @brief   Host-side unit tests for pid_ctrl_t (no HAL, no FreeRTOS).
 *
 * Build:  gcc -o test_pid tests/test_pid.c Core/Src/app/pid.c \
 *             -ICore/Inc -lm -Wall -Wextra
 * Run:    ./test_pid
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "app/pid.h"

static int tests_run    = 0;
static int tests_passed = 0;

#define ASSERT(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
    } else { \
        tests_passed++; \
    } \
} while (0)

#define ASSERT_NEAR(a, b, tol, msg) \
    ASSERT(fabsf((a) - (b)) < (tol), msg)

/* ------------------------------------------------------------------ */

static void test_step_response(void)
{
    printf("[test_step_response]\n");

    pid_ctrl_t p;
    pid_init(&p, 2.0f, 0.1f, 0.5f, 0.0f, 1000.0f);
    pid_set_tuning(&p, 1.0f, 10.0f);  /* standard PID, b=1 */

    float meas = 25.0f;
    float sp   = 60.0f;
    float dt   = 0.05f;

    /* Run for 500 steps (25 s simulated) with a simple first-order
     * plant model. The derivative-on-measurement correctly reduces
     * output as meas rises, so output is NOT monotonic — that's
     * desired behavior, not a bug. We only check convergence. */
    for (int i = 0; i < 500; i++) {
        float out = pid_compute(&p, sp, meas, dt);
        meas += out * 0.002f;  /* plant gain */
    }

    ASSERT(meas >= sp - 10.0f && meas <= sp + 10.0f,
           "measurement should converge near setpoint (within 10C)");
}

/* ------------------------------------------------------------------ */

static void test_output_clamping(void)
{
    printf("[test_output_clamping]\n");

    pid_ctrl_t p;
    pid_init(&p, 100.0f, 10.0f, 0.0f, 0.0f, 1000.0f);

    /* Large error should saturate at output_max */
    float out = pid_compute(&p, 100.0f, 0.0f, 0.05f);
    ASSERT(out <= 1000.0f, "output must not exceed output_max");
    ASSERT(out >= 0.0f,    "output must not go below output_min");

    /* Negative error: output should clamp at 0 */
    pid_reset(&p);
    out = pid_compute(&p, 0.0f, 100.0f, 0.05f);
    ASSERT(out >= 0.0f,    "output must not go below output_min (neg error)");
    ASSERT(out <= 1000.0f, "output must not exceed output_max (neg error)");
}

/* ------------------------------------------------------------------ */

static void test_anti_windup(void)
{
    printf("[test_anti_windup]\n");

    pid_ctrl_t p;
    pid_init(&p, 2.0f, 0.5f, 0.0f, 0.0f, 1000.0f);

    /* Drive into saturation for many cycles */
    for (int i = 0; i < 200; i++)
        pid_compute(&p, 100.0f, 0.0f, 0.05f);

    /* Now flip: measurement above setpoint. Output should drop
     * quickly if anti-windup is working (no massive integrator). */
    float out1 = pid_compute(&p, 50.0f, 80.0f, 0.05f);
    float out2 = pid_compute(&p, 50.0f, 80.0f, 0.05f);
    float out3 = pid_compute(&p, 50.0f, 80.0f, 0.05f);

    ASSERT(out3 < 500.0f,
           "after saturation, output should recover quickly (anti-windup)");
    (void)out1; (void)out2;
}

/* ------------------------------------------------------------------ */

static void test_derivative_on_measurement(void)
{
    printf("[test_derivative_on_measurement]\n");

    /* With derivative on measurement, a setpoint step should NOT
     * cause a derivative spike. Only measurement changes affect D. */
    pid_ctrl_t p;
    pid_init(&p, 0.0f, 0.0f, 10.0f, -10000.0f, 10000.0f);
    pid_set_tuning(&p, 1.0f, 1000.0f);  /* N=1000: minimal filter */

    /* Prime with one call at steady state */
    pid_compute(&p, 25.0f, 25.0f, 0.05f);

    /* Step setpoint from 25 to 60 — measurement unchanged */
    float out = pid_compute(&p, 60.0f, 25.0f, 0.05f);

    /* D term should be ~0 because measurement did not change.
     * P and I are 0 (Kp=0, Ki=0), so output is purely D. */
    ASSERT_NEAR(out, 0.0f, 1.0f,
                "setpoint step should not cause D spike");

    /* Now step measurement — D term should respond */
    float out_after = pid_compute(&p, 60.0f, 30.0f, 0.05f);

    /* measurement went up by 5 → D = -Kd * (30-25)/0.05 = -1000
     * (negative because measurement rising = reduce output) */
    ASSERT(out_after < -100.0f,
           "measurement step should produce negative D response");
}

/* ------------------------------------------------------------------ */

static void test_derivative_filter(void)
{
    printf("[test_derivative_filter]\n");

    /* Compare filtered vs unfiltered D response to a measurement step.
     * Filtered should be smaller in magnitude on the first cycle. */
    pid_ctrl_t p_filt, p_raw;
    pid_init(&p_filt, 1.0f, 0.0f, 5.0f, -10000.0f, 10000.0f);
    pid_init(&p_raw,  1.0f, 0.0f, 5.0f, -10000.0f, 10000.0f);

    pid_set_tuning(&p_filt, 1.0f, 5.0f);     /* N=5: heavy filter */
    pid_set_tuning(&p_raw,  1.0f, 1000.0f);   /* N=1000: minimal filter */

    /* Prime both at steady state */
    pid_compute(&p_filt, 50.0f, 50.0f, 0.05f);
    pid_compute(&p_raw,  50.0f, 50.0f, 0.05f);

    /* Step measurement by 10 degrees */
    float out_filt = pid_compute(&p_filt, 50.0f, 60.0f, 0.05f);
    float out_raw  = pid_compute(&p_raw,  50.0f, 60.0f, 0.05f);

    /* Both should be negative (measurement rose above setpoint).
     * Filtered D should be smaller in magnitude. */
    ASSERT(fabsf(out_filt) < fabsf(out_raw),
           "filtered D should be smaller than unfiltered on step");
}

/* ------------------------------------------------------------------ */

static void test_setpoint_weighting(void)
{
    printf("[test_setpoint_weighting]\n");

    /* With b=0.5, P term = Kp * (0.5*sp - meas).
     * With b=1.0, P term = Kp * (sp - meas).
     * For sp=60, meas=25: b=1 → Kp*35, b=0.5 → Kp*(30-25)=Kp*5 */
    pid_ctrl_t p_full, p_half;
    pid_init(&p_full, 2.0f, 0.0f, 0.0f, -10000.0f, 10000.0f);
    pid_init(&p_half, 2.0f, 0.0f, 0.0f, -10000.0f, 10000.0f);

    pid_set_tuning(&p_full, 1.0f, 10.0f);  /* b = 1.0 */
    pid_set_tuning(&p_half, 0.5f, 10.0f);  /* b = 0.5 */

    float out_full = pid_compute(&p_full, 60.0f, 25.0f, 0.05f);
    float out_half = pid_compute(&p_half, 60.0f, 25.0f, 0.05f);

    /* b=1: P = 2*(1*60 - 25) = 70
     * b=0.5: P = 2*(0.5*60 - 25) = 2*(30-25) = 10 */
    ASSERT_NEAR(out_full, 70.0f, 1.0f,  "b=1.0: P = Kp*(sp-meas)");
    ASSERT_NEAR(out_half, 10.0f, 1.0f,  "b=0.5: P = Kp*(b*sp-meas)");
    ASSERT(out_half < out_full,
           "b=0.5 output should be less than b=1.0");
}

/* ------------------------------------------------------------------ */

static void test_bumpless_transfer(void)
{
    printf("[test_bumpless_transfer]\n");

    /* Simulate: run PID at setpoint=60 for a while, then switch to
     * setpoint=25. The output should transition smoothly (no large
     * jump beyond what the error change dictates). */
    pid_ctrl_t p;
    pid_init(&p, 2.0f, 0.1f, 0.5f, 0.0f, 1000.0f);
    pid_set_tuning(&p, 0.7f, 10.0f);

    float meas = 55.0f;  /* near the first setpoint */

    /* Stabilize near sp=60 */
    float out_before = 0.0f;
    for (int i = 0; i < 100; i++)
        out_before = pid_compute(&p, 60.0f, meas, 0.05f);

    /* Switch setpoint to 25 — large step down */
    float out_after = pid_compute(&p, 25.0f, meas, 0.05f);

    /* With b=0.7 and derivative on measurement, the jump should be
     * bounded. The key check: pid_reset was NOT called, so the
     * integrator contribution is preserved. */
    float jump = fabsf(out_after - out_before);

    ASSERT(jump < 500.0f,
           "setpoint switch should not cause extreme output jump");
}

/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== PID Controller Unit Tests ===\n\n");

    test_step_response();
    test_output_clamping();
    test_anti_windup();
    test_derivative_on_measurement();
    test_derivative_filter();
    test_setpoint_weighting();
    test_bumpless_transfer();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}

#include "unity.h"
#include "app/pid.h"

static pid_ctrl_t pid;

void test_pid_zero_error_zero_output(void)
{
    pid_init(&pid, 2.0f, 0.1f, 0.5f, 0.0f, 1000.0f);
    float out = pid_compute(&pid, 60.0f, 60.0f, 0.05f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, out);
}

void test_pid_positive_error_positive_output(void)
{
    pid_init(&pid, 2.0f, 0.0f, 0.0f, 0.0f, 1000.0f);
    float out = pid_compute(&pid, 60.0f, 20.0f, 0.05f);
    TEST_ASSERT_GREATER_THAN(0.0f, out);
}

void test_pid_output_clamped_to_max(void)
{
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 0.0f, 1000.0f);
    float out = pid_compute(&pid, 1000.0f, 0.0f, 0.05f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1000.0f, out);
}

void test_pid_output_clamped_to_min(void)
{
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 0.0f, 1000.0f);
    float out = pid_compute(&pid, 0.0f, 1000.0f, 0.05f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out);
}

void test_pid_reset_clears_integrator(void)
{
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 0.0f, 1000.0f);
    pid_compute(&pid, 60.0f, 20.0f, 0.05f);
    pid_compute(&pid, 60.0f, 20.0f, 0.05f);
    pid_reset(&pid);
    float out = pid_compute(&pid, 60.0f, 60.0f, 0.05f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, out);
}

void test_pid_integral_accumulates(void)
{
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 0.0f, 1000.0f);
    float out1 = pid_compute(&pid, 60.0f, 20.0f, 0.05f);
    float out2 = pid_compute(&pid, 60.0f, 20.0f, 0.05f);
    TEST_ASSERT_GREATER_THAN(out1, out2);
}

void test_pid_no_derivative_kick_on_first_call(void)
{
    pid_init(&pid, 0.0f, 0.0f, 10.0f, 0.0f, 1000.0f);
    float out = pid_compute(&pid, 60.0f, 20.0f, 0.05f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out);
}

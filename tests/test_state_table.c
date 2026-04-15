#include "unity.h"
#include "app/fsm.h"
#include "app/config.h"

void test_force_down_output(void)
{
    const fsm_output_t *out = fsm_output(FSM_FORCE_DOWN);
    TEST_ASSERT_NOT_NULL(out);
    TEST_ASSERT_EQUAL(TEMP_COOL_C, out->setpoint_c);
    TEST_ASSERT_TRUE(out->pid_enabled);
    TEST_ASSERT_EQUAL(0, out->safety_max_ms);  /* no timeout in FORCE_DOWN */
}

void test_force_up_output(void)
{
    const fsm_output_t *out = fsm_output(FSM_FORCE_UP);
    TEST_ASSERT_NOT_NULL(out);
    TEST_ASSERT_EQUAL(TEMP_ACTIVE_C, out->setpoint_c);
    TEST_ASSERT_TRUE(out->pid_enabled);
    TEST_ASSERT_EQUAL(SAFETY_MAX_FORCE_UP_MS, out->safety_max_ms);
}

void test_fault_output(void)
{
    const fsm_output_t *out = fsm_output(FSM_FAULT);
    TEST_ASSERT_NOT_NULL(out);
    TEST_ASSERT_FALSE(out->pid_enabled);
    TEST_ASSERT_EQUAL(0, out->setpoint_c);
    TEST_ASSERT_EQUAL(0, out->safety_max_ms);  /* terminal */
}

void test_invalid_state_returns_force_down(void)
{
    const fsm_output_t *out = fsm_output(FSM_STATE_COUNT);
    TEST_ASSERT_NOT_NULL(out);
    /* Should defensively return FORCE_DOWN (safe default) */
    TEST_ASSERT_EQUAL(fsm_output(FSM_FORCE_DOWN)->setpoint_c, out->setpoint_c);
}

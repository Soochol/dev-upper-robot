#include "unity.h"
#include "app/fsm.h"

void test_initial_state_is_force_down(void)
{
    /* Boot-time initial state per design D8. */
    TEST_ASSERT_EQUAL(0, FSM_FORCE_DOWN);
}

void test_force_down_trig_up_goes_force_up(void)
{
    TEST_ASSERT_EQUAL(FSM_FORCE_UP,
                      fsm_next(FSM_FORCE_DOWN, FSM_EVT_TRIGGER_FORCE_UP));
}

void test_force_up_trig_down_goes_force_down(void)
{
    TEST_ASSERT_EQUAL(FSM_FORCE_DOWN,
                      fsm_next(FSM_FORCE_UP, FSM_EVT_TRIGGER_FORCE_DOWN));
}

void test_force_up_safety_timeout_goes_force_down(void)
{
    TEST_ASSERT_EQUAL(FSM_FORCE_DOWN,
                      fsm_next(FSM_FORCE_UP, FSM_EVT_SAFETY_TIMEOUT));
}

void test_force_down_fault_req_goes_fault(void)
{
    TEST_ASSERT_EQUAL(FSM_FAULT,
                      fsm_next(FSM_FORCE_DOWN, FSM_EVT_FAULT_REQUESTED));
}

void test_force_up_fault_req_goes_fault(void)
{
    TEST_ASSERT_EQUAL(FSM_FAULT,
                      fsm_next(FSM_FORCE_UP, FSM_EVT_FAULT_REQUESTED));
}

void test_fault_is_terminal_for_all_events(void)
{
    TEST_ASSERT_EQUAL(FSM_FAULT, fsm_next(FSM_FAULT, FSM_EVT_NONE));
    TEST_ASSERT_EQUAL(FSM_FAULT, fsm_next(FSM_FAULT, FSM_EVT_TRIGGER_FORCE_UP));
    TEST_ASSERT_EQUAL(FSM_FAULT, fsm_next(FSM_FAULT, FSM_EVT_TRIGGER_FORCE_DOWN));
    TEST_ASSERT_EQUAL(FSM_FAULT, fsm_next(FSM_FAULT, FSM_EVT_FAULT_REQUESTED));
    TEST_ASSERT_EQUAL(FSM_FAULT, fsm_next(FSM_FAULT, FSM_EVT_SAFETY_TIMEOUT));
}

void test_none_event_keeps_state(void)
{
    TEST_ASSERT_EQUAL(FSM_FORCE_DOWN,
                      fsm_next(FSM_FORCE_DOWN, FSM_EVT_NONE));
    TEST_ASSERT_EQUAL(FSM_FORCE_UP,
                      fsm_next(FSM_FORCE_UP, FSM_EVT_NONE));
}

void test_force_down_ignores_trig_down(void)
{
    TEST_ASSERT_EQUAL(FSM_FORCE_DOWN,
                      fsm_next(FSM_FORCE_DOWN, FSM_EVT_TRIGGER_FORCE_DOWN));
}

void test_force_up_ignores_trig_up(void)
{
    TEST_ASSERT_EQUAL(FSM_FORCE_UP,
                      fsm_next(FSM_FORCE_UP, FSM_EVT_TRIGGER_FORCE_UP));
}

void test_state_names_not_null(void)
{
    TEST_ASSERT_NOT_NULL(fsm_state_name(FSM_FORCE_DOWN));
    TEST_ASSERT_NOT_NULL(fsm_state_name(FSM_FORCE_UP));
    TEST_ASSERT_NOT_NULL(fsm_state_name(FSM_FAULT));
}

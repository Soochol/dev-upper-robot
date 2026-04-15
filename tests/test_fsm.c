/**
 * @file    test_fsm.c
 * @brief   Host-side unit tests for fsm.c + state_table.c.
 *
 * Build:  gcc -o test_fsm test_fsm.c ../Core/Src/app/fsm.c \
 *             ../Core/Src/app/state_table.c -I../Core/Inc -Istubs -lm
 * Run:    ./test_fsm
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "app/fsm.h"
#include "app/config.h"

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

#define ASSERT_EQ(a, b, msg) ASSERT((a) == (b), msg)

/* ------------------------------------------------------------------ */

static void test_fsm_transitions(void)
{
    printf("[test_fsm_transitions]\n");

    /* FORCE_DOWN + TRIGGER_FORCE_UP → FORCE_UP */
    ASSERT_EQ(fsm_next(FSM_FORCE_DOWN, FSM_EVT_TRIGGER_FORCE_UP),
              FSM_FORCE_UP,
              "DOWN + trig_up -> UP");

    /* FORCE_UP + TRIGGER_FORCE_DOWN → FORCE_DOWN */
    ASSERT_EQ(fsm_next(FSM_FORCE_UP, FSM_EVT_TRIGGER_FORCE_DOWN),
              FSM_FORCE_DOWN,
              "UP + trig_down -> DOWN");

    /* FORCE_UP + SAFETY_TIMEOUT → FORCE_DOWN */
    ASSERT_EQ(fsm_next(FSM_FORCE_UP, FSM_EVT_SAFETY_TIMEOUT),
              FSM_FORCE_DOWN,
              "UP + safety_timeout -> DOWN");

    /* FORCE_DOWN + FAULT_REQUESTED → FAULT */
    ASSERT_EQ(fsm_next(FSM_FORCE_DOWN, FSM_EVT_FAULT_REQUESTED),
              FSM_FAULT,
              "DOWN + fault_req -> FAULT");

    /* FORCE_UP + FAULT_REQUESTED → FAULT */
    ASSERT_EQ(fsm_next(FSM_FORCE_UP, FSM_EVT_FAULT_REQUESTED),
              FSM_FAULT,
              "UP + fault_req -> FAULT");
}

static void test_fault_terminal(void)
{
    printf("[test_fault_terminal]\n");

    /* FAULT is terminal — no event can escape it */
    ASSERT_EQ(fsm_next(FSM_FAULT, FSM_EVT_TRIGGER_FORCE_UP),
              FSM_FAULT,
              "FAULT + trig_up -> FAULT (terminal)");

    ASSERT_EQ(fsm_next(FSM_FAULT, FSM_EVT_TRIGGER_FORCE_DOWN),
              FSM_FAULT,
              "FAULT + trig_down -> FAULT (terminal)");

    ASSERT_EQ(fsm_next(FSM_FAULT, FSM_EVT_FAULT_REQUESTED),
              FSM_FAULT,
              "FAULT + fault_req -> FAULT (terminal)");

    ASSERT_EQ(fsm_next(FSM_FAULT, FSM_EVT_SAFETY_TIMEOUT),
              FSM_FAULT,
              "FAULT + safety_to -> FAULT (terminal)");

    ASSERT_EQ(fsm_next(FSM_FAULT, FSM_EVT_NONE),
              FSM_FAULT,
              "FAULT + none -> FAULT (terminal)");
}

static void test_no_change_on_none(void)
{
    printf("[test_no_change_on_none]\n");

    ASSERT_EQ(fsm_next(FSM_FORCE_DOWN, FSM_EVT_NONE),
              FSM_FORCE_DOWN,
              "DOWN + none -> DOWN (no change)");

    ASSERT_EQ(fsm_next(FSM_FORCE_UP, FSM_EVT_NONE),
              FSM_FORCE_UP,
              "UP + none -> UP (no change)");
}

static void test_state_name(void)
{
    printf("[test_state_name]\n");

    const char *name;

    name = fsm_state_name(FSM_FORCE_DOWN);
    ASSERT(name != NULL, "state_name(DOWN) non-NULL");
    ASSERT(strcmp(name, "FORCE_DOWN") == 0, "state_name(DOWN) == FORCE_DOWN");

    name = fsm_state_name(FSM_FORCE_UP);
    ASSERT(name != NULL, "state_name(UP) non-NULL");
    ASSERT(strcmp(name, "FORCE_UP") == 0, "state_name(UP) == FORCE_UP");

    name = fsm_state_name(FSM_FAULT);
    ASSERT(name != NULL, "state_name(FAULT) non-NULL");
    ASSERT(strcmp(name, "FAULT") == 0, "state_name(FAULT) == FAULT");
}

static void test_event_name(void)
{
    printf("[test_event_name]\n");

    const char *name;

    name = fsm_event_name(FSM_EVT_NONE);
    ASSERT(name != NULL, "event_name(NONE) non-NULL");
    ASSERT(strcmp(name, "NONE") == 0, "event_name(NONE) == NONE");

    name = fsm_event_name(FSM_EVT_TRIGGER_FORCE_UP);
    ASSERT(name != NULL, "event_name(TRIG_UP) non-NULL");

    name = fsm_event_name(FSM_EVT_FAULT_REQUESTED);
    ASSERT(name != NULL, "event_name(FAULT_REQ) non-NULL");
}

static void test_fsm_output(void)
{
    printf("[test_fsm_output]\n");

    const fsm_output_t *out;

    /* FORCE_UP output: setpoint should be TEMP_ACTIVE_C (60) */
    out = fsm_output(FSM_FORCE_UP);
    ASSERT(out != NULL, "output(FORCE_UP) non-NULL");
    ASSERT_EQ(out->setpoint_c, TEMP_ACTIVE_C,
              "FORCE_UP setpoint == TEMP_ACTIVE_C (60)");
    ASSERT(out->pid_enabled == true,
           "FORCE_UP pid_enabled == true");
    ASSERT(out->safety_max_ms == SAFETY_MAX_FORCE_UP_MS,
           "FORCE_UP safety_max_ms == 60000");

    /* FORCE_DOWN output: setpoint should be TEMP_COOL_C (25) */
    out = fsm_output(FSM_FORCE_DOWN);
    ASSERT(out != NULL, "output(FORCE_DOWN) non-NULL");
    ASSERT_EQ(out->setpoint_c, TEMP_COOL_C,
              "FORCE_DOWN setpoint == TEMP_COOL_C (25)");
    ASSERT(out->pid_enabled == true,
           "FORCE_DOWN pid_enabled == true");
    ASSERT(out->safety_max_ms == 0,
           "FORCE_DOWN safety_max_ms == 0 (no timeout)");

    /* FAULT output: pid disabled, setpoint 0 */
    out = fsm_output(FSM_FAULT);
    ASSERT(out != NULL, "output(FAULT) non-NULL");
    ASSERT_EQ(out->setpoint_c, 0,
              "FAULT setpoint == 0");
    ASSERT(out->pid_enabled == false,
           "FAULT pid_enabled == false");
}

/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== FSM Unit Tests ===\n\n");

    test_fsm_transitions();
    test_fault_terminal();
    test_no_change_on_none();
    test_state_name();
    test_event_name();
    test_fsm_output();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}

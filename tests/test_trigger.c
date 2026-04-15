/**
 * @file    test_trigger.c
 * @brief   Host-side unit tests for trigger_rule.c (rule-based trigger).
 *
 * FreeRTOS stubs are provided below so trigger_rule.c compiles on the host.
 * The mock tick counter lets us test debounce logic.
 *
 * Build:  gcc -o test_trigger test_trigger.c ../Core/Src/app/trigger_rule.c \
 *             -I../Core/Inc -Istubs -lm
 * Run:    ./test_trigger
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

/* ================================================================
 * FreeRTOS stubs — must be defined BEFORE trigger_rule.c includes
 * FreeRTOS.h and task.h.  We provide minimal stub headers.
 * ================================================================ */

/* Mock tick counter for debounce testing. */
uint32_t g_mock_tick = 0;

/* FreeRTOS stub: returns the mock tick value controlled by the test. */
uint32_t xTaskGetTickCount(void) { return g_mock_tick; }

/* These stub headers are created in stubs/ alongside stm32f1xx_hal.h */

#include "app/trigger.h"
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

/* Helper: build a snapshot with given FSR and tilt values. */
static sensor_snapshot_t make_snap(int16_t fsr, float tilt_x)
{
    sensor_snapshot_t snap;
    memset(&snap, 0, sizeof(snap));
    snap.fsr_raw = fsr;
    snap.imu_tilt_x_deg = tilt_x;
    snap.imu_tilt_y_deg = 0.0f;
    snap.ml_feat = NULL;
    return snap;
}

/* ------------------------------------------------------------------ */

static void test_force_up_fsr(void)
{
    printf("[test_force_up_fsr]\n");

    /* Reset debounce state by advancing tick far ahead */
    g_mock_tick = 100000;

    sensor_snapshot_t snap = make_snap(
        FSR_THRESHOLD_UP_RAW + 100,  /* above UP threshold */
        0.0f                         /* tilt below threshold */
    );

    uint8_t evt = trig_rule.eval(&snap, FSM_FORCE_DOWN);
    ASSERT_EQ(evt, TRIG_EVENT_FORCE_UP,
              "FORCE_DOWN + FSR > UP_THRESH -> FORCE_UP");
}

static void test_force_up_tilt(void)
{
    printf("[test_force_up_tilt]\n");

    /* Advance tick past debounce */
    g_mock_tick += TRIGGER_DEBOUNCE_MS + 100;

    sensor_snapshot_t snap = make_snap(
        0,                           /* FSR below threshold */
        (float)IMU_TILT_UP_DEG + 5.0f  /* tilt above threshold */
    );

    uint8_t evt = trig_rule.eval(&snap, FSM_FORCE_DOWN);
    ASSERT_EQ(evt, TRIG_EVENT_FORCE_UP,
              "FORCE_DOWN + tilt > UP_DEG -> FORCE_UP");
}

static void test_force_down(void)
{
    printf("[test_force_down]\n");

    /* Advance tick past debounce */
    g_mock_tick += TRIGGER_DEBOUNCE_MS + 100;

    /* FORCE_UP → FORCE_DOWN requires BOTH FSR < DOWN_THRESH AND tilt < DOWN_DEG */
    sensor_snapshot_t snap = make_snap(
        FSR_THRESHOLD_DOWN_RAW - 100,         /* below DOWN threshold */
        (float)IMU_TILT_DOWN_DEG - 2.0f       /* below DOWN tilt */
    );

    uint8_t evt = trig_rule.eval(&snap, FSM_FORCE_UP);
    ASSERT_EQ(evt, TRIG_EVENT_FORCE_DOWN,
              "FORCE_UP + FSR < DOWN & tilt < DOWN -> FORCE_DOWN");
}

static void test_hysteresis(void)
{
    printf("[test_hysteresis]\n");

    /* Advance tick past debounce */
    g_mock_tick += TRIGGER_DEBOUNCE_MS + 100;

    /* FSR in the hysteresis band (between DOWN and UP thresholds).
     * In FORCE_DOWN state: no trigger because FSR < UP_THRESH.
     * Tilt also in dead zone. */
    sensor_snapshot_t snap = make_snap(
        (FSR_THRESHOLD_DOWN_RAW + FSR_THRESHOLD_UP_RAW) / 2,  /* mid-band */
        (float)(IMU_TILT_DOWN_DEG + IMU_TILT_UP_DEG) / 2.0f   /* mid-band */
    );

    uint8_t evt = trig_rule.eval(&snap, FSM_FORCE_DOWN);
    ASSERT_EQ(evt, TRIG_EVENT_NONE,
              "FORCE_DOWN + FSR in hysteresis band -> NONE");

    /* In FORCE_UP state: no trigger because FSR > DOWN_THRESH */
    evt = trig_rule.eval(&snap, FSM_FORCE_UP);
    ASSERT_EQ(evt, TRIG_EVENT_NONE,
              "FORCE_UP + FSR in hysteresis band -> NONE");
}

static void test_debounce(void)
{
    printf("[test_debounce]\n");

    /* First, advance tick far ahead to clear any prior debounce */
    g_mock_tick = 500000;

    /* Trigger a FORCE_UP event */
    sensor_snapshot_t snap_up = make_snap(
        FSR_THRESHOLD_UP_RAW + 500,
        0.0f
    );
    uint8_t evt = trig_rule.eval(&snap_up, FSM_FORCE_DOWN);
    ASSERT_EQ(evt, TRIG_EVENT_FORCE_UP,
              "debounce: initial trigger fires");

    /* Advance tick by less than TRIGGER_DEBOUNCE_MS */
    g_mock_tick += TRIGGER_DEBOUNCE_MS / 2;

    /* Try to trigger again — should be suppressed by debounce */
    sensor_snapshot_t snap_down = make_snap(
        FSR_THRESHOLD_DOWN_RAW - 100,
        0.0f
    );
    evt = trig_rule.eval(&snap_down, FSM_FORCE_UP);
    ASSERT_EQ(evt, TRIG_EVENT_NONE,
              "debounce: event suppressed within debounce window");

    /* Advance tick past the debounce window */
    g_mock_tick += TRIGGER_DEBOUNCE_MS;

    /* Now a trigger should fire again */
    evt = trig_rule.eval(&snap_down, FSM_FORCE_UP);
    ASSERT_EQ(evt, TRIG_EVENT_FORCE_DOWN,
              "debounce: event fires after debounce window expires");
}

static void test_no_trigger_in_fault(void)
{
    printf("[test_no_trigger_in_fault]\n");

    /* Advance tick past debounce */
    g_mock_tick += TRIGGER_DEBOUNCE_MS + 100;

    /* In FAULT state, rule_eval checks cur == FSM_FORCE_DOWN or
     * cur == FSM_FORCE_UP. FAULT matches neither, so no event. */
    sensor_snapshot_t snap = make_snap(
        FSR_THRESHOLD_UP_RAW + 500,
        (float)IMU_TILT_UP_DEG + 10.0f
    );

    uint8_t evt = trig_rule.eval(&snap, FSM_FAULT);
    ASSERT_EQ(evt, TRIG_EVENT_NONE,
              "FAULT state -> no trigger events");
}

static void test_force_down_needs_both(void)
{
    printf("[test_force_down_needs_both]\n");

    /* Advance tick past debounce */
    g_mock_tick += TRIGGER_DEBOUNCE_MS + 100;

    /* FORCE_DOWN requires BOTH conditions (AND logic).
     * FSR below threshold but tilt above threshold → no trigger. */
    sensor_snapshot_t snap = make_snap(
        FSR_THRESHOLD_DOWN_RAW - 100,
        (float)IMU_TILT_DOWN_DEG + 5.0f   /* tilt too high */
    );

    uint8_t evt = trig_rule.eval(&snap, FSM_FORCE_UP);
    ASSERT_EQ(evt, TRIG_EVENT_NONE,
              "FORCE_UP + FSR low but tilt high -> NONE (AND logic)");

    /* Advance tick past debounce */
    g_mock_tick += TRIGGER_DEBOUNCE_MS + 100;

    /* Tilt below threshold but FSR above threshold → no trigger. */
    snap = make_snap(
        FSR_THRESHOLD_DOWN_RAW + 100,     /* FSR too high */
        (float)IMU_TILT_DOWN_DEG - 2.0f   /* tilt low enough */
    );

    evt = trig_rule.eval(&snap, FSM_FORCE_UP);
    ASSERT_EQ(evt, TRIG_EVENT_NONE,
              "FORCE_UP + tilt low but FSR high -> NONE (AND logic)");
}

/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== Trigger Rule Unit Tests ===\n\n");

    test_force_up_fsr();
    test_force_up_tilt();
    test_force_down();
    test_hysteresis();
    test_debounce();
    test_no_trigger_in_fault();
    test_force_down_needs_both();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}

/**
 * @file    test_trigger_ml.c
 * @brief   Host-side unit tests for trigger_ml.c (ML-based trigger).
 *
 * Mocks score() so each test injects controlled P_UP/P_DOWN values.
 * The FSR sanity gate (FSR_THRESHOLD_DOWN_RAW) is the focus — scenario
 * #1 is the regression test for the 2026-05-06 false-positive bug
 * where FSR=0 + IMU tilt fired FORCE_UP.
 *
 * Build:  see tests/Makefile
 * Run:    ./test_trigger_ml
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

/* Mock tick counter for debounce testing. */
uint32_t g_mock_tick = 0;
uint32_t xTaskGetTickCount(void) { return g_mock_tick; }

/* rtt_log_hb_s stub — trigger_ml.c logs every 20th call. No-op. */
void rtt_log_hb_s(const char *tag,
                  const char *k1, int32_t v1,
                  const char *k2, int32_t v2,
                  const char *k3, int32_t v3,
                  const char *k4, int32_t v4)
{
    (void)tag; (void)k1; (void)v1; (void)k2; (void)v2;
    (void)k3; (void)v3; (void)k4; (void)v4;
}

/* Mock score() — controlled by globals so each test injects P_UP/P_DOWN. */
static float g_mock_p_down = 0.5f;
static float g_mock_p_up   = 0.5f;
void score(float *input, float *output)
{
    (void)input;
    output[0] = g_mock_p_down;
    output[1] = g_mock_p_up;
}

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

/* Build a snapshot. gyro_dps sets gyro_x raw so |gyro| equals that DPS. */
static sensor_snapshot_t make_snap(int16_t fsr, float gyro_dps)
{
    static ml_features_t feat;  /* must be non-NULL for ml_eval to evaluate */
    sensor_snapshot_t snap;
    memset(&snap, 0, sizeof(snap));
    memset(&feat, 0, sizeof(feat));
    snap.fsr_raw = fsr;
    /* gyro_now = sqrt(gx² + gy² + gz²) * IMU_GYRO_SCALE_DPS, with gy=gz=0
     * → gx_raw = gyro_dps / IMU_GYRO_SCALE_DPS */
    snap.imu.gyro_x = (int16_t)(gyro_dps / IMU_GYRO_SCALE_DPS);
    snap.imu.gyro_y = 0;
    snap.imu.gyro_z = 0;
    snap.ml_feat = &feat;
    return snap;
}

/* Reset all internal static state in trig_ml by advancing tick far past
 * debounce and exhausting any active streaks with neutral inputs. */
static void reset_state(void)
{
    g_mock_tick += TRIGGER_DEBOUNCE_MS * 4;
    g_mock_p_up = 0.0f; g_mock_p_down = 0.0f;
    sensor_snapshot_t neutral = make_snap(0, 0.0f);
    for (int i = 0; i < 10; i++) {
        (void)trig_ml.eval(&neutral, FSM_FORCE_DOWN);
    }
    g_mock_tick += TRIGGER_DEBOUNCE_MS * 4;
}

/* ------------------------------------------------------------------ */

/* SCENARIO 1 [REGRESSION]: FSR=0 + high P_UP + high gyro
 * → no fire. This is the 2026-05-06 bug: IMU held in air with FSR
 * unpressed previously fired FORCE_UP. The FSR sanity gate must block. */
static void test_regression_fsr_zero_blocks_up(void)
{
    printf("[test_regression_fsr_zero_blocks_up]\n");
    reset_state();

    g_mock_p_up = 0.7f; g_mock_p_down = 0.3f;
    sensor_snapshot_t snap = make_snap(0, 200.0f);  /* FSR=0, gyro=200dps */

    /* Run 10 ticks — even with all other gates passing, FSR=0 blocks. */
    int fired = 0;
    for (int i = 0; i < 10; i++) {
        if (trig_ml.eval(&snap, FSM_FORCE_DOWN) != TRIG_EVENT_NONE) {
            fired = 1;
        }
    }
    ASSERT_EQ(fired, 0, "FSR=0 + P_UP=0.7 + gyro=200 → no UP fire (regression)");
}

/* SCENARIO 2: FSR pressed + high P_UP + high gyro → fires after 5 ticks. */
static void test_true_positive_fires(void)
{
    printf("[test_true_positive_fires]\n");
    reset_state();

    g_mock_p_up = 0.7f; g_mock_p_down = 0.3f;
    sensor_snapshot_t snap = make_snap(2000, 100.0f);

    /* Ticks 1–4: streak accumulates, no fire yet. */
    for (int i = 0; i < 4; i++) {
        ASSERT_EQ(trig_ml.eval(&snap, FSM_FORCE_DOWN), TRIG_EVENT_NONE,
                  "streak < 5 → no fire");
    }
    /* Tick 5: streak hits ML_CONFIRM_TICKS, fires. */
    ASSERT_EQ(trig_ml.eval(&snap, FSM_FORCE_DOWN), TRIG_EVENT_FORCE_UP,
              "streak == 5 → FORCE_UP fires");
}

/* SCENARIO 3: P_UP below threshold → no fire even with FSR pressed. */
static void test_low_prob_blocks(void)
{
    printf("[test_low_prob_blocks]\n");
    reset_state();

    g_mock_p_up = 0.5f; g_mock_p_down = 0.5f;  /* below 0.6 */
    sensor_snapshot_t snap = make_snap(2000, 100.0f);

    int fired = 0;
    for (int i = 0; i < 10; i++) {
        if (trig_ml.eval(&snap, FSM_FORCE_DOWN) != TRIG_EVENT_NONE) fired = 1;
    }
    ASSERT_EQ(fired, 0, "P_UP=0.5 (< threshold) → no fire");
}

/* SCENARIO 4: gyro below threshold → no fire. */
static void test_low_gyro_blocks(void)
{
    printf("[test_low_gyro_blocks]\n");
    reset_state();

    g_mock_p_up = 0.7f; g_mock_p_down = 0.3f;
    sensor_snapshot_t snap = make_snap(2000, 10.0f);  /* gyro=10dps < 30 */

    int fired = 0;
    for (int i = 0; i < 10; i++) {
        if (trig_ml.eval(&snap, FSM_FORCE_DOWN) != TRIG_EVENT_NONE) fired = 1;
    }
    ASSERT_EQ(fired, 0, "gyro=10dps < 30 → no UP fire");
}

/* SCENARIO 5: directional gate — already in FORCE_UP, UP cannot re-fire. */
static void test_direction_gate(void)
{
    printf("[test_direction_gate]\n");
    reset_state();

    g_mock_p_up = 0.7f; g_mock_p_down = 0.3f;
    sensor_snapshot_t snap = make_snap(2000, 100.0f);

    int fired = 0;
    for (int i = 0; i < 10; i++) {
        if (trig_ml.eval(&snap, FSM_FORCE_UP) == TRIG_EVENT_FORCE_UP) fired = 1;
    }
    ASSERT_EQ(fired, 0, "cur=FORCE_UP + high P_UP → no UP re-fire");
}

/* SCENARIO 6: debounce — after firing, identical input is suppressed
 * within TRIGGER_DEBOUNCE_MS. */
static void test_debounce(void)
{
    printf("[test_debounce]\n");
    reset_state();

    g_mock_p_up = 0.7f; g_mock_p_down = 0.3f;
    sensor_snapshot_t snap = make_snap(2000, 100.0f);

    /* Fire once. */
    int fired = 0;
    for (int i = 0; i < 5; i++) {
        if (trig_ml.eval(&snap, FSM_FORCE_DOWN) == TRIG_EVENT_FORCE_UP) fired = 1;
    }
    ASSERT_EQ(fired, 1, "debounce: initial fire");

    /* Within debounce window, no fire. */
    g_mock_tick += TRIGGER_DEBOUNCE_MS / 2;
    int fired_again = 0;
    for (int i = 0; i < 10; i++) {
        if (trig_ml.eval(&snap, FSM_FORCE_DOWN) != TRIG_EVENT_NONE) fired_again = 1;
    }
    ASSERT_EQ(fired_again, 0, "debounce: suppressed within window");
}

/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== Trigger ML Unit Tests ===\n\n");

    test_regression_fsr_zero_blocks_up();
    test_true_positive_fires();
    test_low_prob_blocks();
    test_low_gyro_blocks();
    test_direction_gate();
    test_debounce();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}

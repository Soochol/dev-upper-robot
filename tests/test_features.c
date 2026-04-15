/**
 * @file    test_features.c
 * @brief   Host-side unit tests for features.c (tilt + ML feature extraction).
 *
 * Build:  gcc -o test_features test_features.c ../Core/Src/app/features.c \
 *             -I../Core/Inc -Istubs -lm
 * Run:    ./test_features
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* stubs/ provides stm32f1xx_hal.h so sensors_i2c.h can compile */
#include "app/features.h"
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

#define ASSERT_NEAR(a, b, tol, msg) \
    ASSERT(fabsf((a) - (b)) < (tol), msg)

/* ------------------------------------------------------------------ */

static void test_tilt_state_init(void)
{
    printf("[test_tilt_state_init]\n");

    tilt_state_t s;
    tilt_state_init(&s);

    ASSERT_NEAR(s.filtered_tilt_x, 90.0f, 0.01f,
                "init: filtered_tilt_x == 90");
    ASSERT_NEAR(s.filtered_tilt_y, 90.0f, 0.01f,
                "init: filtered_tilt_y == 90");
    ASSERT_NEAR(s.ref_tilt_x, 90.0f, 0.01f,
                "init: ref_tilt_x == 90");
    ASSERT_NEAR(s.ref_tilt_y, 90.0f, 0.01f,
                "init: ref_tilt_y == 90");
    ASSERT(s.sample_count == 0,
           "init: sample_count == 0");
    ASSERT(s.anchored == 0,
           "init: anchored == 0");
}

static void test_tilt_upright(void)
{
    printf("[test_tilt_upright]\n");

    /* Accel vector (0, 0, 16384) = 1g along Z axis.
     * ax/amag = 0/1 → acos(0) = 90 degrees for X.
     * ay/amag = 0/1 → acos(0) = 90 degrees for Y.
     * Since init is at 90, after anchoring, output should be ~0. */

    tilt_state_t s;
    tilt_state_init(&s);

    imu_raw_t raw = {0};
    raw.accel_x = 0;
    raw.accel_y = 0;
    raw.accel_z = 16384;  /* 1g along Z = ±2g FSR → 16384 LSB/g */

    float tx, ty;

    /* Feed 40 samples (> 30 warm-up) to trigger anchoring. With
     * constant input, after anchoring the output is relative to the
     * anchored reference. Since accel is constant at (0,0,1g), the
     * LPF converges to 90 deg and the anchor captures 90 deg, so
     * the relative tilt output should be near 0. */
    for (int i = 0; i < 40; i++) {
        tilt_update(&s, &raw, &tx, &ty);
    }

    ASSERT(s.anchored == 1, "anchored after 40 samples");
    ASSERT_NEAR(tx, 0.0f, 2.0f, "tilt_x ~0 for upright Z-axis");
    ASSERT_NEAR(ty, 0.0f, 2.0f, "tilt_y ~0 for upright Z-axis");
}

static void test_tilt_freefall(void)
{
    printf("[test_tilt_freefall]\n");

    /* Free-fall: accel = (0, 0, 0), amag ≈ 0.
     * Should return last filtered values without updating. */

    tilt_state_t s;
    tilt_state_init(&s);

    imu_raw_t normal = {0};
    normal.accel_z = 16384;

    float tx, ty;

    /* Prime with normal readings to get through anchoring */
    for (int i = 0; i < 40; i++) {
        tilt_update(&s, &normal, &tx, &ty);
    }
    float last_tx = tx;
    float last_ty = ty;

    /* Now feed zero accel (free-fall) */
    imu_raw_t freefall = {0};
    tilt_update(&s, &freefall, &tx, &ty);

    ASSERT_NEAR(tx, last_tx, 0.01f,
                "free-fall returns last filtered tilt_x");
    ASSERT_NEAR(ty, last_ty, 0.01f,
                "free-fall returns last filtered tilt_y");
}

/* ------------------------------------------------------------------ */

static void test_ml_window_init(void)
{
    printf("[test_ml_window_init]\n");

    ml_window_t w;
    ml_window_init(&w);

    ASSERT(w.head == 0,  "ml_window_init: head == 0");
    ASSERT(w.count == 0, "ml_window_init: count == 0");
}

static void test_ml_features_constant(void)
{
    printf("[test_ml_features_constant]\n");

    /* Feed a constant FSR signal — slope should be ~0. */
    ml_window_t w;
    memset(&w, 0, sizeof(w));
    ml_window_init(&w);

    imu_raw_t imu = {0};
    imu.accel_z = 16384;  /* 1g Z */

    ml_features_t feat;

    /* Fill the window with constant fsr_raw = 1000 */
    for (int i = 0; i < ML_WINDOW_SIZE + 5; i++) {
        ml_features_update(&w, 1000, 45.0f, 10.0f, &imu, &feat);
    }

    /* fsr_slope (v[5]) should be ~0 for constant signal */
    ASSERT_NEAR(feat.v[5], 0.0f, 0.1f,
                "constant FSR -> fsr_slope ~0");

    /* fsr_mean (v[4]) should be ~1000 */
    ASSERT_NEAR(feat.v[4], 1000.0f, 0.1f,
                "constant FSR -> fsr_mean ~1000");

    /* fsr_raw (v[0]) should be 1000 */
    ASSERT_NEAR(feat.v[0], 1000.0f, 0.01f,
                "fsr_raw == 1000");
}

static void test_ml_features_ramp(void)
{
    printf("[test_ml_features_ramp]\n");

    /* Feed a linearly increasing FSR signal — slope should be positive. */
    ml_window_t w;
    memset(&w, 0, sizeof(w));
    ml_window_init(&w);

    imu_raw_t imu = {0};
    imu.accel_z = 16384;

    ml_features_t feat;

    /* Fill the window with ramp: 0, 100, 200, ... */
    for (int i = 0; i < ML_WINDOW_SIZE + 5; i++) {
        int16_t fsr = (int16_t)(i * 100);
        ml_features_update(&w, fsr, 0.0f, 0.0f, &imu, &feat);
    }

    /* fsr_slope (v[5]) should be > 0 for increasing signal */
    ASSERT(feat.v[5] > 0.0f,
           "ramp FSR -> fsr_slope > 0");
}

static void test_ml_features_valid_underfilled(void)
{
    printf("[test_ml_features_valid_underfilled]\n");

    /* Window not full → features invalid */
    ml_window_t w;
    memset(&w, 0, sizeof(w));
    ml_window_init(&w);

    imu_raw_t imu = {0};
    imu.accel_z = 16384;
    ml_features_t feat;

    /* Push only 5 samples (< ML_WINDOW_SIZE = 20) */
    for (int i = 0; i < 5; i++) {
        ml_features_update(&w, 1000, 0.0f, 0.0f, &imu, &feat);
    }

    ASSERT(ml_features_valid(&w, &feat) == false,
           "underfilled window -> features invalid");
}

static void test_ml_features_valid_full(void)
{
    printf("[test_ml_features_valid_full]\n");

    /* Full window with normal accel → features valid */
    ml_window_t w;
    memset(&w, 0, sizeof(w));
    ml_window_init(&w);

    imu_raw_t imu = {0};
    imu.accel_z = 16384;  /* 1g Z → ax=0, ay=0 in range */
    ml_features_t feat;

    for (int i = 0; i < ML_WINDOW_SIZE; i++) {
        ml_features_update(&w, 1000, 0.0f, 0.0f, &imu, &feat);
    }

    ASSERT(ml_features_valid(&w, &feat) == true,
           "full window + normal accel -> features valid");
}

static void test_ml_features_valid_outofrange(void)
{
    printf("[test_ml_features_valid_outofrange]\n");

    /* Full window but accel out of ±2g range → invalid */
    ml_window_t w;
    memset(&w, 0, sizeof(w));
    ml_window_init(&w);

    /* accel_x = 32767 → ax_g = 32767 * 2/32768 ≈ 2.0 (borderline)
     * accel_x = 32767+500 would overflow int16 so let's use a value
     * that maps to > 2.1g: 2.1 / (2.0/32768) = 34406 → clamp to 32767
     * Actually for int16, max is 32767 → 32767*2/32768 = 1.999...
     * So we need to craft the feat manually or use a high raw accel_x.
     * With ±2g FSR, raw 32767 → 1.9999g which is in range.
     * To get > 2.1g we need a larger FSR... but the sensor caps at 32767.
     *
     * The validity check tests feat->v[3] (ax_g) and feat->v[7] (ay_g).
     * We can craft a feature vector where ax > 2.1 by directly testing. */

    imu_raw_t imu = {0};
    imu.accel_z = 16384;
    ml_features_t feat;

    for (int i = 0; i < ML_WINDOW_SIZE; i++) {
        ml_features_update(&w, 1000, 0.0f, 0.0f, &imu, &feat);
    }

    /* Directly overwrite the accel feature to simulate out-of-range */
    feat.v[3] = 2.5f;  /* ax_g out of range */
    ASSERT(ml_features_valid(&w, &feat) == false,
           "ax > 2.1g -> features invalid");

    /* Reset and test ay out of range */
    feat.v[3] = 0.0f;
    feat.v[7] = -2.5f;  /* ay_g out of range */
    ASSERT(ml_features_valid(&w, &feat) == false,
           "ay < -2.1g -> features invalid");
}

/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== Features Unit Tests ===\n\n");

    test_tilt_state_init();
    test_tilt_upright();
    test_tilt_freefall();
    test_ml_window_init();
    test_ml_features_constant();
    test_ml_features_ramp();
    test_ml_features_valid_underfilled();
    test_ml_features_valid_full();
    test_ml_features_valid_outofrange();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}

/**
 * @file    features.c
 * @brief   IMU tilt calculation with adaptive LPF and boot-time anchoring.
 *
 * Algorithm from the IMU hardware reference guide:
 *   1. Raw → g conversion (±2g: raw * 2.0 / 32768.0)
 *   2. Vector magnitude |a| = sqrt(ax² + ay² + az²)
 *   3. Normalize: ax_n = ax / |a|, clamp to [-1, 1]
 *   4. Tilt = acos(ax_n) * (180/π)  → 0°~180° per axis
 *   5. Adaptive LPF: alpha depends on how close |a| is to 1g
 *   6. Anchoring: after 30 warm-up samples, capture reference
 *   7. Return: filtered_tilt - reference → degrees since boot
 *
 * Uses <math.h> for sqrtf, acosf. On Cortex-M3 (no FPU) these are
 * soft-float (~100 cycles each), called once per T_ML tick (20 Hz)
 * = ~5 µs. Negligible.
 */

#include <math.h>
#include "app/features.h"
#include "app/config.h"

/* Adaptive LPF thresholds (deviation of |accel| from 1g, in g units). */
#define AMAG_STATIC_THRESH   0.050f  /* < 50mg  → static */
#define AMAG_MILD_THRESH     0.150f  /* < 150mg → mild dynamic */

/* LPF alpha values. Higher = faster tracking. */
#define ALPHA_STATIC         0.15f   /* ~67ms  time constant @ 100Hz */
#define ALPHA_MILD           0.02f   /* ~500ms */
#define ALPHA_DYNAMIC        0.001f  /* ~10s   (nearly frozen) */

/* Samples to skip before anchoring (LPF warm-up). At 20 Hz read rate
 * this is 30 / 20 = 1.5 seconds. The reference guide says 30 at 100Hz
 * (0.3s) but at our 20Hz rate we use 30 samples = 1.5s for stability. */
#define ANCHOR_WARMUP_SAMPLES  30u

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void tilt_state_init(tilt_state_t *s)
{
    s->filtered_tilt_x = 90.0f;  /* upright default */
    s->filtered_tilt_y = 90.0f;
    s->ref_tilt_x      = 90.0f;
    s->ref_tilt_y      = 90.0f;
    s->sample_count    = 0;
    s->anchored        = 0;
}

void tilt_update(tilt_state_t *s,
                 const imu_raw_t *raw,
                 float *out_tilt_x_deg,
                 float *out_tilt_y_deg)
{
    /* 1. Raw → g */
    float ax = (float)raw->accel_x * IMU_ACCEL_SCALE_G;
    float ay = (float)raw->accel_y * IMU_ACCEL_SCALE_G;
    float az = (float)raw->accel_z * IMU_ACCEL_SCALE_G;

    /* 2. Vector magnitude */
    float amag = sqrtf(ax * ax + ay * ay + az * az);
    if (amag < 0.001f) {
        /* Sensor error or free-fall — return last filtered values. */
        *out_tilt_x_deg = s->filtered_tilt_x - s->ref_tilt_x;
        *out_tilt_y_deg = s->filtered_tilt_y - s->ref_tilt_y;
        return;
    }

    /* 3. Normalize and clamp to [-1, 1] for acos domain. */
    float ax_n = clampf(ax / amag, -1.0f, 1.0f);
    float ay_n = clampf(ay / amag, -1.0f, 1.0f);

    /* 4. Tilt angle (0°=axis aligned with gravity, 90°=perpendicular). */
    float tilt_x = acosf(ax_n) * (180.0f / M_PI);
    float tilt_y = acosf(ay_n) * (180.0f / M_PI);

    /* 5. Adaptive LPF alpha selection. */
    float deviation = fabsf(amag - 1.0f);
    float alpha;
    if (deviation < AMAG_STATIC_THRESH) {
        alpha = ALPHA_STATIC;
    } else if (deviation < AMAG_MILD_THRESH) {
        alpha = ALPHA_MILD;
    } else {
        alpha = ALPHA_DYNAMIC;
    }

    /* Apply LPF. */
    s->filtered_tilt_x = s->filtered_tilt_x * (1.0f - alpha) + tilt_x * alpha;
    s->filtered_tilt_y = s->filtered_tilt_y * (1.0f - alpha) + tilt_y * alpha;

    /* 6. Anchoring. */
    s->sample_count++;
    if (!s->anchored && s->sample_count >= ANCHOR_WARMUP_SAMPLES) {
        s->ref_tilt_x = s->filtered_tilt_x;
        s->ref_tilt_y = s->filtered_tilt_y;
        s->anchored = 1;
    }

    /* 7. Return: relative tilt since boot. */
    *out_tilt_x_deg = s->filtered_tilt_x - s->ref_tilt_x;
    *out_tilt_y_deg = s->filtered_tilt_y - s->ref_tilt_y;
}

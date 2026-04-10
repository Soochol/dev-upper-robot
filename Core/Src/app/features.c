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

/* ========================================================================
 * ML feature extraction — sliding window statistics
 * ======================================================================== */

void ml_window_init(ml_window_t *w)
{
    w->head  = 0;
    w->count = 0;
    /* Buffers are zero-initialized by BSS for static allocation. */
}

/* Compute mean of a circular buffer. n = min(count, ML_WINDOW_SIZE). */
static float win_mean(const float *buf, uint16_t n)
{
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        sum += buf[i];
    }
    return sum / (float)n;
}

/* Compute least-squares slope over the window.
 *
 *   slope = (N * sum(i*y) - sum(i)*sum(y)) / (N * sum(i²) - sum(i)²)
 *
 * For a fixed window of size N, sum(i) and sum(i²) are constants:
 *   sum(i)  = N*(N-1)/2
 *   sum(i²) = N*(N-1)*(2N-1)/6
 *
 * The buffer is circular: the oldest sample is at head (wraps around).
 * We iterate from oldest to newest so i=0 is the oldest sample.
 */
static float win_slope(const float *buf, uint16_t head, uint16_t n)
{
    float sum_y  = 0.0f;
    float sum_iy = 0.0f;

    for (uint16_t k = 0; k < n; k++) {
        uint16_t idx = (head + k) % ML_WINDOW_SIZE;
        float y = buf[idx];
        sum_y  += y;
        sum_iy += (float)k * y;
    }

    float fn = (float)n;
    float sum_i  = fn * (fn - 1.0f) / 2.0f;
    float sum_i2 = fn * (fn - 1.0f) * (2.0f * fn - 1.0f) / 6.0f;
    float denom  = fn * sum_i2 - sum_i * sum_i;

    if (fabsf(denom) < 1e-6f) return 0.0f;

    return (fn * sum_iy - sum_i * sum_y) / denom;
}

void ml_features_update(ml_window_t *w,
                        int16_t fsr_raw,
                        float tilt_x, float tilt_y,
                        const imu_raw_t *imu,
                        ml_features_t *out)
{
    /* Compute instantaneous derived values. */
    float ax = (float)imu->accel_x * IMU_ACCEL_SCALE_G;
    float ay = (float)imu->accel_y * IMU_ACCEL_SCALE_G;

    float gx = (float)imu->gyro_x * IMU_GYRO_SCALE_DPS;
    float gy = (float)imu->gyro_y * IMU_GYRO_SCALE_DPS;
    float gz = (float)imu->gyro_z * IMU_GYRO_SCALE_DPS;
    float gyro_mag = sqrtf(gx * gx + gy * gy + gz * gz);

    /* Push into circular buffers. */
    w->buf_fsr[w->head]       = (float)fsr_raw;
    w->buf_tilt_x[w->head]    = tilt_x;
    w->buf_gyro_mag[w->head]  = gyro_mag;

    w->head = (w->head + 1) % ML_WINDOW_SIZE;
    if (w->count < ML_WINDOW_SIZE) {
        w->count++;
    }

    /* Number of valid samples in the window. */
    uint16_t n = w->count;

    /* The oldest sample index (start of the window). When the buffer is
     * full, this is w->head (the slot we just overwrote was the oldest).
     * When not full, it's always 0. */
    uint16_t oldest = (n < ML_WINDOW_SIZE) ? 0 : w->head;

    /* Feature vector. */
    out->v[0] = (float)fsr_raw;            /* fsr_raw */
    out->v[1] = tilt_x;                    /* tilt_x_deg */
    out->v[2] = tilt_y;                    /* tilt_y_deg */
    out->v[3] = ax;                        /* ax_g (raw accel X) */
    out->v[4] = win_mean(w->buf_fsr, n);   /* fsr_mean */
    out->v[5] = win_slope(w->buf_fsr, oldest, n);      /* fsr_slope */
    out->v[6] = win_slope(w->buf_tilt_x, oldest, n);   /* tilt_x_slope */
    out->v[7] = ay;                        /* ay_g (raw accel Y) */
    out->v[8] = win_mean(w->buf_gyro_mag, n);           /* gyro_mag_mean */
}

bool ml_features_valid(const ml_window_t *w, const ml_features_t *feat)
{
    /* Window must be full before we trust temporal features. */
    if (w->count < ML_WINDOW_SIZE) return false;

    /* Accel axis sanity: ±2g FSR means valid range is [-2.1, 2.1].
     * Outside this range means sensor error or saturation. */
    float ax_val = feat->v[3];
    float ay_val = feat->v[7];
    if (ax_val < -2.1f || ax_val > 2.1f) return false;
    if (ay_val < -2.1f || ay_val > 2.1f) return false;

    return true;
}

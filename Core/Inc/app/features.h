/**
 * @file    features.h
 * @brief   Sensor feature extraction — IMU tilt angle with adaptive LPF.
 *
 * Converts raw ICM42670P accelerometer data into a filtered tilt angle
 * relative to the boot-time pose (anchoring). The adaptive low-pass filter
 * varies its time constant based on how close |accel| is to 1g:
 *
 *   Static  (|a| - 1g < 50mg)  → alpha 0.15, fast tracking (~67ms)
 *   Mild    (|a| - 1g < 150mg) → alpha 0.02, smooth     (~500ms)
 *   Dynamic (|a| - 1g ≥ 150mg) → alpha 0.001, freeze     (~10s)
 *
 * This prevents vibration or shock from producing false tilt readings
 * while still tracking slow, genuine inclination changes quickly.
 *
 * Anchoring: the first 30 samples (~0.3s at 100Hz / 20Hz read rate)
 * are used to warm up the LPF. After that, the current filtered tilt
 * is captured as the reference. Subsequent returns are relative to
 * that reference — so the output is "degrees of tilt change since boot."
 *
 * No HAL or FreeRTOS dependency — host-testable.
 */

#ifndef APP_FEATURES_H
#define APP_FEATURES_H

#include <stdint.h>
#include <stdbool.h>
#include "app/sensors_i2c.h"   /* imu_raw_t */
#include "app/config.h"        /* ML_WINDOW_SIZE */

/**
 * @brief  Persistent state for the tilt computation.
 *         One instance per sensor (or one shared if only Right is used).
 *         Call tilt_state_init once, then tilt_update each sample.
 */
typedef struct {
    float filtered_tilt_x;  /* LPF output (degrees) */
    float filtered_tilt_y;
    float ref_tilt_x;       /* anchored reference */
    float ref_tilt_y;
    uint32_t sample_count;  /* total samples fed so far */
    uint8_t  anchored;      /* 1 after anchoring is done */
} tilt_state_t;

/**
 * @brief  Initialize the tilt state. Call once before the first update.
 */
void tilt_state_init(tilt_state_t *s);

/**
 * @brief  Feed one raw IMU sample and return the filtered, anchored tilt.
 *
 * @param  s     persistent state
 * @param  raw   raw accelerometer+gyro from icm42670p_read()
 * @param  out_tilt_x_deg  output: tilt around X axis (degrees, relative to boot)
 * @param  out_tilt_y_deg  output: tilt around Y axis
 */
void tilt_update(tilt_state_t *s,
                 const imu_raw_t *raw,
                 float *out_tilt_x_deg,
                 float *out_tilt_y_deg);

/* ========================================================================
 * ML feature extraction — sliding window statistics
 * ========================================================================
 * Maintains a circular buffer of the last ML_WINDOW_SIZE samples and
 * computes temporal features (mean, slope, variance) each tick.
 *
 * Feature vector (9 elements):
 *   [0] fsr_raw          instantaneous FSR reading
 *   [1] tilt_x_deg       instantaneous X-axis tilt
 *   [2] tilt_y_deg       instantaneous Y-axis tilt
 *   [3] accel_mag        instantaneous accel magnitude (g)
 *   [4] fsr_mean         windowed FSR mean
 *   [5] fsr_slope        windowed FSR slope (least-squares)
 *   [6] tilt_x_slope     windowed tilt X slope
 *   [7] accel_var        windowed accel magnitude variance
 *   [8] gyro_mag_mean    windowed gyro magnitude mean
 */

#define ML_FEAT_COUNT  9

/**
 * @brief  Output feature vector for ML inference.
 */
typedef struct {
    float v[ML_FEAT_COUNT];
} ml_features_t;

/**
 * @brief  Persistent state for the sliding window feature extractor.
 *         Allocated as a static variable in T_ML (not on the stack).
 */
typedef struct {
    float buf_fsr[ML_WINDOW_SIZE];
    float buf_tilt_x[ML_WINDOW_SIZE];
    float buf_accel_mag[ML_WINDOW_SIZE];
    float buf_gyro_mag[ML_WINDOW_SIZE];
    uint16_t head;          /* next write position (circular) */
    uint16_t count;         /* samples inserted so far (max ML_WINDOW_SIZE) */
} ml_window_t;

/**
 * @brief  Initialize the sliding window state. Call once at task start.
 */
void ml_window_init(ml_window_t *w);

/**
 * @brief  Push one sample and compute the 9-element feature vector.
 *
 * @param  w         persistent window state
 * @param  fsr_raw   current FSR reading (raw int16)
 * @param  tilt_x    current tilt X (degrees, from tilt_update)
 * @param  tilt_y    current tilt Y (degrees)
 * @param  imu       raw IMU data (accel + gyro)
 * @param  out       output feature vector
 */
void ml_features_update(ml_window_t *w,
                        int16_t fsr_raw,
                        float tilt_x, float tilt_y,
                        const imu_raw_t *imu,
                        ml_features_t *out);

/**
 * @brief  Sanity check: return true if the feature vector is usable.
 *         Returns false if accel magnitude is abnormal (< 0.3g or > 3g)
 *         or if the window hasn't filled yet.
 */
bool ml_features_valid(const ml_window_t *w, const ml_features_t *feat);

#endif /* APP_FEATURES_H */

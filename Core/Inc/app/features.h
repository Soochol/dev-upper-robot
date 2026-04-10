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
#include "app/sensors_i2c.h"   /* imu_raw_t */

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

#endif /* APP_FEATURES_H */

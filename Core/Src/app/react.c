/**
 * @file    react.c
 * @brief   Implementation of the reaction-time tracker.
 *
 * Pure compute — no globals, no logging, no FreeRTOS calls beyond using
 * TickType_t arithmetic. Side effects flow only through the caller-owned
 * react_tracker_t struct and the return value.
 *
 * Onset criterion: gyro magnitude (in dps) crossing REACT_GYRO_ONSET_DPS
 * OR fsr_raw crossing FSR_THRESHOLD_DOWN_RAW. The first-to-cross wins and
 * sets source = GYRO or FSR. Re-arming only happens after the previous
 * measurement completes (or times out, or sees an FU→FD edge).
 *
 * Why gyro AND fsr (OR-gate): boot-pressed FSR would block FSR-only
 * detection forever; idle-arm jitter on gyro alone would miss
 * slow-press scenarios. Either signal is enough.
 */

#include <math.h>
#include "app/react.h"
#include "app/config.h"
#include "app/sensors_i2c.h"   /* IMU_GYRO_SCALE_DPS */

void react_tracker_init(react_tracker_t *t)
{
    t->onset_tick    = 0;
    t->prev_state    = FSM_FORCE_DOWN;
    t->source        = REACT_SRC_NONE;
    t->last_react_ms = 0;
}

static inline float gyro_magnitude_dps(const imu_raw_t *imu)
{
    float gx = (float)imu->gyro_x * IMU_GYRO_SCALE_DPS;
    float gy = (float)imu->gyro_y * IMU_GYRO_SCALE_DPS;
    float gz = (float)imu->gyro_z * IMU_GYRO_SCALE_DPS;
    return sqrtf(gx * gx + gy * gy + gz * gz);
}

uint32_t react_update(react_tracker_t *t,
                      fsm_state_t cur_state,
                      const sensor_snapshot_t *snap,
                      TickType_t tick,
                      bool fu_fired)
{
    /* Arm: FORCE_DOWN + sensor onset, only if not already armed. */
    if (cur_state == FSM_FORCE_DOWN && t->onset_tick == 0) {
        float gmag = gyro_magnitude_dps(&snap->imu);
        if (gmag > REACT_GYRO_ONSET_DPS) {
            t->onset_tick = tick;
            t->source     = REACT_SRC_GYRO;
        } else if (snap->fsr_raw > FSR_THRESHOLD_DOWN_RAW) {
            t->onset_tick = tick;
            t->source     = REACT_SRC_FSR;
        }
    }

    /* Stale-onset timeout — cancel if no FU within window.
     * Absorbs false-onset (user wiggled then stopped) and debounced
     * trigger suppression. */
    if (t->onset_tick != 0) {
        uint32_t age_ms = (uint32_t)((tick - t->onset_tick)
                                      * portTICK_PERIOD_MS);
        if (age_ms > REACT_ONSET_TIMEOUT_MS) {
            t->onset_tick = 0;
            t->source     = REACT_SRC_NONE;
        }
    }

    /* FU→FD edge: clear stale arm so next FD cycle re-arms cleanly. */
    if (t->prev_state == FSM_FORCE_UP && cur_state == FSM_FORCE_DOWN) {
        t->onset_tick = 0;
        t->source     = REACT_SRC_NONE;
    }
    t->prev_state = cur_state;

    /* Measure on FU dispatch, only if armed. */
    if (fu_fired && t->onset_tick != 0) {
        uint32_t react_ms = (uint32_t)((tick - t->onset_tick)
                                        * portTICK_PERIOD_MS);
        t->last_react_ms = react_ms;
        t->onset_tick    = 0;
        /* source preserved so caller can read which channel armed it. */
        return react_ms;
    }

    return 0;
}

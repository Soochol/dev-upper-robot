/**
 * @file    trigger_ml.c
 * @brief   ML-based trigger provider — Random Forest inference via m2cgen.
 *
 * Calls the auto-generated score() function from model_rf.c (produced by
 * train.py + m2cgen). The score function takes 9 features and outputs
 * class probabilities [P(FORCE_DOWN), P(FORCE_UP)].
 *
 * Directional filtering: only fires transitions that make sense from
 * the current FSM state (same semantics as trigger_rule.c).
 */

#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "app/trigger.h"
#include "app/features.h"
#include "app/config.h"
#include "app/rtt_log.h"

/* m2cgen-generated model — lives in Core/Src/app/model_rf.c.
 * Replaced each training iteration by train.py --output. */
extern void score(float *input, float *output);

/* Probability threshold. Higher = fewer false positives but slower
 * response. 0.6 is a conservative starting point. */
#define ML_PROB_THRESHOLD  0.6f

/* Consecutive ticks above threshold before firing. At 20 Hz, 5 ticks
 * = 250 ms. Filters out transient FSR spikes that briefly exceed the
 * threshold. Real arm lifts sustain above threshold for seconds. */
#define ML_CONFIRM_TICKS   5

static uint8_t ml_eval(const sensor_snapshot_t *snap, fsm_state_t cur)
{
    if (!snap->ml_feat) return TRIG_EVENT_NONE;

    /* Debounce: suppress events for TRIGGER_DEBOUNCE_MS after the last
     * trigger. Same mechanism as trigger_rule.c. */
    static TickType_t last_trigger_tick = 0;
    static uint8_t    ever_triggered    = 0;

    if (ever_triggered) {
        TickType_t elapsed = (xTaskGetTickCount() - last_trigger_tick)
                             * portTICK_PERIOD_MS;
        if (elapsed < TRIGGER_DEBOUNCE_MS) {
            return TRIG_EVENT_NONE;
        }
    }

    float output[2];  /* [P(FORCE_DOWN), P(FORCE_UP)] */
    score((float *)snap->ml_feat->v, output);

    /* Instantaneous gyro magnitude from raw IMU. Computed before the
     * debug log so both the log and the gate share the same value.
     * Drops to ~0 immediately when arm stops — prevents "arm already
     * up + FSR press" false FORCE_UP trigger. */
    float gx = (float)snap->imu.gyro_x * IMU_GYRO_SCALE_DPS;
    float gy_raw = (float)snap->imu.gyro_y * IMU_GYRO_SCALE_DPS;
    float gz = (float)snap->imu.gyro_z * IMU_GYRO_SCALE_DPS;
    float gyro_now = sqrtf(gx * gx + gy_raw * gy_raw + gz * gz);

    /* Debug: log model output every ~1s (every 20th call). */
    static uint32_t ml_call_count = 0;
    ml_call_count++;
    if ((ml_call_count % 20) == 0) {
        rtt_log_hb_s("[ml:p]",
                     " u=", (int32_t)(output[1] * 100.0f),
                     " gy=", (int32_t)(gyro_now * 10.0f),
                     " fs=", (int32_t)snap->ml_feat->v[5],
                     " ax=", (int32_t)(snap->ml_feat->v[3] * 1000.0f));
    }

    /* Directional filtering with consecutive-tick confirmation.
     * Must exceed threshold for ML_CONFIRM_TICKS in a row before firing.
     * Resets to 0 if the signal drops below threshold. */
    static uint8_t up_streak   = 0;
    static uint8_t down_streak = 0;

    if (cur == FSM_FORCE_DOWN && output[1] > ML_PROB_THRESHOLD
                              && gyro_now > 30.0f) {
        up_streak++;
        down_streak = 0;
    } else if (cur == FSM_FORCE_UP && output[0] > ML_PROB_THRESHOLD) {
        down_streak++;
        up_streak = 0;
    } else {
        up_streak   = 0;
        down_streak = 0;
    }

    uint8_t event = TRIG_EVENT_NONE;

    if (up_streak >= ML_CONFIRM_TICKS) {
        event = TRIG_EVENT_FORCE_UP;
        up_streak = 0;
    } else if (down_streak >= ML_CONFIRM_TICKS) {
        event = TRIG_EVENT_FORCE_DOWN;
        down_streak = 0;
    }

    if (event != TRIG_EVENT_NONE) {
        last_trigger_tick = xTaskGetTickCount();
        ever_triggered    = 1;
    }

    return event;
}

const trigger_provider_t trig_ml = {
    .name = "ml",
    .eval = ml_eval,
};

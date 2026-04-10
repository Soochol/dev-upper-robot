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
extern void score(double *input, double *output);

/* Probability threshold. Higher = fewer false positives but slower
 * response. 0.6 is a conservative starting point. */
#define ML_PROB_THRESHOLD  0.6

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

    /* Convert float[9] → double[9] for m2cgen score(). */
    double input[ML_FEAT_COUNT];
    for (int i = 0; i < ML_FEAT_COUNT; i++) {
        input[i] = (double)snap->ml_feat->v[i];
    }

    double output[2];  /* [P(FORCE_DOWN), P(FORCE_UP)] */
    score(input, output);

    /* Debug: log model output every ~1s (every 20th call). */
    static uint32_t ml_call_count = 0;
    ml_call_count++;
    if ((ml_call_count % 20) == 0) {
        rtt_log_hb_s("[ml:p]",
                     " u=", (int32_t)(output[1] * 100.0),
                     " gy=", (int32_t)(sqrtf(
                         (float)snap->imu.gyro_x * (2000.0f/32768.0f) * (float)snap->imu.gyro_x * (2000.0f/32768.0f) +
                         (float)snap->imu.gyro_y * (2000.0f/32768.0f) * (float)snap->imu.gyro_y * (2000.0f/32768.0f) +
                         (float)snap->imu.gyro_z * (2000.0f/32768.0f) * (float)snap->imu.gyro_z * (2000.0f/32768.0f)) * 10.0f),
                     " fs=", (int32_t)snap->ml_feat->v[5],
                     " ax=", (int32_t)(snap->ml_feat->v[3] * 1000.0f));
    }

    /* Directional filtering with consecutive-tick confirmation.
     * Must exceed threshold for ML_CONFIRM_TICKS in a row before firing.
     * Resets to 0 if the signal drops below threshold. */
    static uint8_t up_streak   = 0;
    static uint8_t down_streak = 0;

    /* FORCE_UP requires gyro activity RIGHT NOW (not windowed mean).
     * Instantaneous gyro from raw IMU — drops to 0 immediately when
     * arm stops. Prevents "arm already up + FSR press" false trigger. */
    float gx = (float)snap->imu.gyro_x * (2000.0f / 32768.0f);
    float gy_raw = (float)snap->imu.gyro_y * (2000.0f / 32768.0f);
    float gz = (float)snap->imu.gyro_z * (2000.0f / 32768.0f);
    float gyro_now = sqrtf(gx * gx + gy_raw * gy_raw + gz * gz);

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

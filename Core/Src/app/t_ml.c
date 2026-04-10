/**
 * @file    t_ml.c
 * @brief   T_ML task — sensor acquisition + trigger provider evaluation.
 *
 * 20 Hz periodic loop with 25 ms offset from T_PID (minimizes I2C1
 * mutex contention, see plan A1 analysis).
 *
 * Per tick:
 *   1. Take mtx_i2c1, read ICM42670P (accel + gyro) and ADS1115 (FSR).
 *   2. Pack a sensor_snapshot_t from the raw readings.
 *   3. Call trigger_eval() which dispatches to the compile-time selected
 *      provider (rule or ml). The rule provider checks FSR > threshold
 *      with hysteresis.
 *   4. If the event is not NONE, publish to q_trigger_to_state.
 *   5. Log sensor values + trigger event once per second.
 *
 * One-shot init:
 *   - ICM42670P: WHO_AM_I check + enable accel/gyro low-noise mode.
 *   - ADS1115: write config register for continuous AIN0.
 *   Both inits run under mtx_i2c1 after the 25 ms offset delay.
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "i2c.h"
#include "app/config.h"
#include "app/ipc.h"
#include "app/fsm.h"
#include "app/sensors_i2c.h"
#include "app/trigger.h"
#include "app/rtt_log.h"

void t_ml_run(void *arg)
{
    (void)arg;

    /* Establish the 25 ms offset relative to T_PID so the two tasks
     * that share the I2C1 bus are naturally interleaved. */
    vTaskDelay(pdMS_TO_TICKS(OFFSET_T_ML_MS));

    rtt_log_str("[t_ml] start (Phase 4: sensor + trigger)");
    rtt_log_kv("[t_ml] provider=", 0);  /* 0 = name follows next line */
    rtt_log_str(trigger_provider_name());

    /* One-shot sensor init under the bus mutex. ICM42670P needs a ~10 ms
     * ramp-up after enabling; ADS1115 starts converting immediately. */
    if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(500)) == pdTRUE) {
        (void)icm42670p_init(&hi2c1);
        vTaskDelay(pdMS_TO_TICKS(ICM42670P_STARTUP_DELAY_MS));
        (void)ads1115_init(&hi2c1);
        xSemaphoreGive(mtx_i2c1);
    } else {
        rtt_log_str("[t_ml] init: mtx_i2c1 timeout");
    }

    TickType_t next_wake = xTaskGetTickCount();
    uint32_t   tick = 0;

    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PERIOD_T_ML_MS));
        tick++;

        /* ---- 1. Read sensors ---- */
        imu_raw_t imu = {0};
        int16_t   fsr_raw = 0;
        HAL_StatusTypeDef imu_st = HAL_ERROR, fsr_st = HAL_ERROR;

        if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(10)) == pdTRUE) {
            imu_st = icm42670p_read(&hi2c1, &imu);
            fsr_st = ads1115_read(&hi2c1, &fsr_raw);
            xSemaphoreGive(mtx_i2c1);
        }

        /* ---- 2. Pack snapshot ---- */
        sensor_snapshot_t snap = {
            .fsr_raw      = fsr_raw,
            .imu          = imu,
            .imu_tilt_deg = 0.0f,  /* Phase 4b: atan2 from accel */
        };

        /* ---- 3. Evaluate trigger ---- */
        fsm_state_t cur_state = (fsm_state_t)g_fsm_state;
        uint8_t event = trigger_eval(&snap, cur_state);

        /* ---- 4. Publish trigger event if not NONE ---- */
        if (event != TRIG_EVENT_NONE) {
            trig_msg_t msg = { .event = event, .pad = {0, 0, 0} };
            (void)xQueueSendToBack(q_trigger_to_state, &msg, 0);
        }

        /* ---- 5. Heartbeat log (1 Hz) ---- */
        if ((tick % 20) == 0) {
            rtt_log_hb("[t_ml]",
                       " fsr=", (uint32_t)(uint16_t)fsr_raw,
                       " imu_z=", (uint32_t)(uint16_t)imu.accel_z,
                       " evt=", (uint32_t)event,
                       " st=", (uint32_t)((imu_st == HAL_OK && fsr_st == HAL_OK) ? 1 : 0));
        }
    }
}

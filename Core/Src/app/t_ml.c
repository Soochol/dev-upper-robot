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
#include "app/features.h"
#include "app/trigger.h"
#include "app/rtt_log.h"
#if DATA_COLLECT_MODE
#include "app/sd_logger.h"
#include "main.h"             /* IN_FUNC_SW1_Pin/Port, SW2 */
#endif

void t_ml_run(void *arg)
{
    (void)arg;

    /* Establish the 25 ms offset relative to T_PID so the two tasks
     * that share the I2C1 bus are naturally interleaved. */
    vTaskDelay(pdMS_TO_TICKS(OFFSET_T_ML_MS));

    rtt_log_str("[t_ml] start (Phase 4: sensor + trigger)");
    rtt_log_kv("[t_ml] provider=", 0);  /* 0 = name follows next line */
    rtt_log_str(trigger_provider_name());

    /* One-shot sensor init. Split into 3 mutex-bounded phases with
     * delays OUTSIDE the mutex. This prevents T_PID from hitting
     * IR_READ_FAIL_LIMIT during our 150 ms of combined sensor
     * start-up delays. Each phase holds the mutex for <10 ms of
     * I2C traffic only. */

    /* Phase 1: ICM42670P soft reset (I2C writes only, <5 ms). */
    if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(500)) == pdTRUE) {
        (void)icm42670p_reset(&hi2c1);
        xSemaphoreGive(mtx_i2c1);
    } else {
        rtt_log_str("[t_ml] init: mtx timeout (reset)");
    }
    /* Soft reset settling — mutex FREE, T_PID can read IR here. */
    vTaskDelay(pdMS_TO_TICKS(ICM42670P_RESET_DELAY_MS));

    /* Phase 2: ICM42670P config (WHO_AM_I + accel/gyro/PWR, <10 ms). */
    if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(500)) == pdTRUE) {
        (void)icm42670p_configure(&hi2c1);
        xSemaphoreGive(mtx_i2c1);
    } else {
        rtt_log_str("[t_ml] init: mtx timeout (configure)");
    }
    /* PWR_MGMT0 stabilization — mutex FREE. */
    vTaskDelay(pdMS_TO_TICKS(ICM42670P_STARTUP_DELAY_MS));

    /* Phase 3: ADS1115 config write (<2 ms). */
    if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(500)) == pdTRUE) {
        (void)ads1115_init(&hi2c1);
        xSemaphoreGive(mtx_i2c1);
    } else {
        rtt_log_str("[t_ml] init: mtx timeout (ads1115)");
    }

    /* Tilt computation state — persistent across ticks. */
    tilt_state_t tilt;
    tilt_state_init(&tilt);

    /* ML feature extraction — sliding window state (static to keep it
     * off the task stack; ~360 bytes for ML_WINDOW_SIZE=20). */
    static ml_window_t ml_win;
    ml_window_init(&ml_win);

#if DATA_COLLECT_MODE
    /* SD card init — mount and scan for next file number.
     * Recording starts when user presses FUNC_SW1. */
    bool sd_ok = sd_logger_init();
    if (sd_ok) {
        rtt_log_str("[t_ml] SD mounted -- press SW1 to record");
    } else {
        rtt_log_str("[t_ml] SD mount FAILED");
    }
    uint32_t btn_debounce = 0;
#endif

    TickType_t next_wake = xTaskGetTickCount();
    uint32_t   tick = 0;

    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PERIOD_T_ML_MS));
        tick++;

        /* ---- 1. Read sensors ---- */
        imu_raw_t imu = {0};
        int16_t   fsr_raw = 0;
        HAL_StatusTypeDef imu_st = HAL_ERROR;

        if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(10)) == pdTRUE) {
            imu_st = icm42670p_read(&hi2c1, &imu);
            (void)ads1115_read(&hi2c1, &fsr_raw);
            xSemaphoreGive(mtx_i2c1);
        }

        /* ---- 2. Feature extraction: tilt from accel ---- */
        float tilt_x = 0.0f, tilt_y = 0.0f;
        if (imu_st == HAL_OK) {
            tilt_update(&tilt, &imu, &tilt_x, &tilt_y);
        }

        /* ---- 3. ML feature extraction (sliding window) ---- */
        ml_features_t ml_feat;
        ml_features_update(&ml_win, fsr_raw, tilt_x, tilt_y,
                           &imu, &ml_feat);

        /* ---- 3a. Pack snapshot ---- */
        sensor_snapshot_t snap = {
            .fsr_raw        = fsr_raw,
            .imu            = imu,
            .imu_tilt_x_deg = tilt_x,
            .imu_tilt_y_deg = tilt_y,
            .ml_feat        = ml_features_valid(&ml_win, &ml_feat)
                              ? &ml_feat : NULL,
        };

        /* Prove T_ML is alive — readable via GDB. */
        { extern volatile uint32_t sd_dbg_tick; sd_dbg_tick = tick; }

#if DATA_COLLECT_MODE
        /* ---- 3b. Button-controlled SD recording ---- */
        if (sd_ok) {
            bool sw1 = (HAL_GPIO_ReadPin(IN_FUNC_SW1_GPIO_Port,
                                         IN_FUNC_SW1_Pin) == GPIO_PIN_RESET);
            bool sw2 = (HAL_GPIO_ReadPin(IN_FUNC_SW2_GPIO_Port,
                                         IN_FUNC_SW2_Pin) == GPIO_PIN_RESET);

            if (sw1 && !sd_logger_is_recording()
                    && (tick - btn_debounce) > 4) {
                sd_logger_start();
                btn_debounce = tick;
            }
            if (sw2 && sd_logger_is_recording()
                    && (tick - btn_debounce) > 4) {
                sd_logger_stop();
                sd_logger_dump_rtt();
                btn_debounce = tick;
            }

            /* Write sensor row if recording. */
            if (sd_logger_is_recording()) {
                uint32_t ts_ms = (uint32_t)(xTaskGetTickCount()
                                 * portTICK_PERIOD_MS);
                sd_logger_write_row(ts_ms, fsr_raw,
                                    imu.accel_x, imu.accel_y, imu.accel_z,
                                    imu.gyro_x, imu.gyro_y, imu.gyro_z,
                                    tilt_x, tilt_y);
            }
        }
#endif

        /* ---- 3c. Evaluate trigger ---- */
        fsm_state_t cur_state = (fsm_state_t)g_fsm_state;
        uint8_t event = trigger_eval(&snap, cur_state);

        /* ---- 4. Publish trigger event if not NONE ---- */
        if (event != TRIG_EVENT_NONE) {
            trig_msg_t msg = { .event = event, .pad = {0, 0, 0} };
            (void)xQueueSendToBack(q_trigger_to_state, &msg, 0);
        }

        /* ---- 5. Heartbeat log (1 Hz) — 3 lines per second ---- */
        if ((tick % 20) == 0) {
            rtt_log_hb_s("[ml:a]",
                         " x=", (int32_t)imu.accel_x,
                         " y=", (int32_t)imu.accel_y,
                         " z=", (int32_t)imu.accel_z,
                         " fsr=", (int32_t)fsr_raw);

            rtt_log_hb_s("[ml:g]",
                         " x=", (int32_t)imu.gyro_x,
                         " y=", (int32_t)imu.gyro_y,
                         " z=", (int32_t)imu.gyro_z,
                         " evt=", (int32_t)event);

            rtt_log_hb_s("[ml:t]",
                         " tx=", (int32_t)(tilt_x * 100.0f),
                         " ty=", (int32_t)(tilt_y * 100.0f),
                         NULL, 0,
                         NULL, 0);
        }
    }
}

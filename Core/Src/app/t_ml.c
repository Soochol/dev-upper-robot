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

/* Phase markers written to g_phase_ml. T_WDG dumps the last phase on
 * STALL so we know which code section a frozen T_ML was executing. */
enum {
    PHASE_ML_IDLE       = 0,   /* between iterations (vTaskDelayUntil) */
    PHASE_ML_MUTEX_TAKE = 1,   /* waiting on mtx_i2c1 */
    PHASE_ML_IMU_READ   = 2,   /* HAL_I2C_Mem_Read for ICM42670P (busy-wait) */
    PHASE_ML_FSR_READ   = 3,   /* HAL_I2C_Mem_Write+Read for ADS1115 */
    PHASE_ML_FAIL_CHECK = 4,   /* failure tracking + recovery decision */
    PHASE_ML_RECOVERY   = 5,   /* I2C bus recovery loop */
    PHASE_ML_TILT       = 6,   /* tilt_update (sqrtf, acosf) */
    PHASE_ML_FEATURES   = 7,   /* ml_features_update (sliding window) */
    PHASE_ML_TRIGGER    = 8,   /* trigger_eval + queue publish */
    PHASE_ML_SD_LOG     = 9,   /* SD card row write (DATA_COLLECT_MODE) */
    PHASE_ML_HEARTBEAT  = 10,  /* RTT 1Hz heartbeat */
};

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
        g_phase_ml = PHASE_ML_IDLE;
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PERIOD_T_ML_MS));
        TickType_t cycle_start = xTaskGetTickCount();
        tick++;

        /* ---- 1. Read sensors ---- */
        imu_raw_t imu = {0};
        int16_t   fsr_raw = 0;
        HAL_StatusTypeDef imu_st = HAL_ERROR;
        HAL_StatusTypeDef fsr_st = HAL_ERROR;

        g_phase_ml = PHASE_ML_MUTEX_TAKE;
        if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_phase_ml = PHASE_ML_IMU_READ;
            imu_st = icm42670p_read(&hi2c1, &imu);
            g_phase_ml = PHASE_ML_FSR_READ;
            fsr_st = ads1115_read(&hi2c1, &fsr_raw);
            xSemaphoreGive(mtx_i2c1);
        }

        /* ---- 1a. Sensor failure tracking ----
         * Keep last valid values on failure (prevents false triggers).
         * Escalate to bus recovery + FAULT after ML_SENSOR_FAIL_LIMIT. */
        g_phase_ml = PHASE_ML_FAIL_CHECK;
        {
            static int16_t last_valid_fsr = 0;
            static imu_raw_t last_valid_imu = {0};
            static uint32_t fsr_fail_count = 0;
            static uint32_t imu_fail_count = 0;

            if (fsr_st == HAL_OK) {
                last_valid_fsr = fsr_raw;
                fsr_fail_count = 0;
            } else {
                fsr_raw = last_valid_fsr;
                fsr_fail_count++;
            }

            /* IMU sentinel detection: HAL_OK with all-0xFFFF payload means
             * the I2C transaction protocol-completed but the chip returned
             * idle bus (EMI / brownout / clock-stretch glitch). Treat as
             * failure — restore last valid sample so feature/SD/trigger
             * pipelines see clean data instead of -1 garbage. */
            bool imu_corrupt = (imu_st == HAL_OK) && (
                (imu.gyro_x  == -1 && imu.gyro_y  == -1 && imu.gyro_z  == -1) ||
                (imu.accel_x == -1 && imu.accel_y == -1 && imu.accel_z == -1)
            );

            if (imu_st == HAL_OK && !imu_corrupt) {
                last_valid_imu = imu;
                imu_fail_count = 0;
            } else {
                imu = last_valid_imu;
                imu_fail_count++;
            }

            if (fsr_fail_count >= ML_SENSOR_FAIL_LIMIT ||
                imu_fail_count >= ML_SENSOR_FAIL_LIMIT) {
                g_phase_ml = PHASE_ML_RECOVERY;
                /* Bus recovery under mutex [C2]. */
                if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(100)) == pdTRUE) {
                    bool recovered = false;
                    for (uint8_t att = 0; att < I2C_MAX_RECOVERY; att++) {
                        g_canary_ml++;  /* [C2] keep canary alive during recovery */
                        i2c1_bus_recover(&hi2c1);
                        imu_raw_t test_imu = {0};
                        int16_t   test_fsr = 0;
                        HAL_StatusTypeDef t1 = icm42670p_read(&hi2c1, &test_imu);
                        HAL_StatusTypeDef t2 = ads1115_read(&hi2c1, &test_fsr);
                        if (t1 == HAL_OK && t2 == HAL_OK) {
                            recovered = true;
                            fsr_fail_count = 0;
                            imu_fail_count = 0;
                            break;
                        }
                    }
                    xSemaphoreGive(mtx_i2c1);
                    if (!recovered) {
                        fault_req_t freq = {
                            .reason = (fsr_fail_count >= ML_SENSOR_FAIL_LIMIT)
                                      ? FAULT_REASON_FSR_TIMEOUT
                                      : FAULT_REASON_IMU_TIMEOUT,
                            .pad = {0,0,0}
                        };
                        (void)xQueueSendToBack(q_fault_req, &freq, 0);
                        fsr_fail_count = 0;
                        imu_fail_count = 0;
                    }
                } else {
                    /* [W2] Mutex timeout — recovery impossible. Send FAULT. */
                    fault_req_t freq = {
                        .reason = (fsr_fail_count >= ML_SENSOR_FAIL_LIMIT)
                                  ? FAULT_REASON_FSR_TIMEOUT
                                  : FAULT_REASON_IMU_TIMEOUT,
                    };
                    (void)xQueueSendToBack(q_fault_req, &freq, 0);
                    fsr_fail_count = 0;
                    imu_fail_count = 0;
                }
            }
        }

        /* ---- 2. Feature extraction: tilt from accel ---- */
        g_phase_ml = PHASE_ML_TILT;
        float tilt_x = 0.0f, tilt_y = 0.0f;
        if (imu_st == HAL_OK) {
            tilt_update(&tilt, &imu, &tilt_x, &tilt_y);
        }

        /* ---- 3. ML feature extraction (sliding window) ---- */
        g_phase_ml = PHASE_ML_FEATURES;
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
        g_phase_ml = PHASE_ML_SD_LOG;
        /* ---- 3b. SW1 single-button toggle for SD recording ----
         * Falling-edge detect (released → pressed) prevents auto-toggle
         * when the user holds the button. 4-tick (200 ms) debounce
         * absorbs mechanical chatter. Data is retrieved by physically
         * removing the SD card — no in-band dump path. */
        if (sd_ok) {
            static bool sw1_prev = false;

            bool sw1 = (HAL_GPIO_ReadPin(IN_FUNC_SW1_GPIO_Port,
                                         IN_FUNC_SW1_Pin) == GPIO_PIN_RESET);

            if (sw1 && !sw1_prev && (tick - btn_debounce) > 4) {
                if (sd_logger_is_recording()) {
                    sd_logger_stop();
                } else {
                    sd_logger_start();
                }
                btn_debounce = tick;
            }
            sw1_prev = sw1;

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
        g_phase_ml = PHASE_ML_TRIGGER;
        fsm_state_t cur_state = (fsm_state_t)g_fsm_state;
        uint8_t event = trigger_eval(&snap, cur_state);

        /* ---- 4. Publish trigger event if not NONE ---- */
        if (event != TRIG_EVENT_NONE) {
            trig_msg_t msg = { .event = event, .pad = {0, 0, 0} };
            (void)xQueueSendToBack(q_trigger_to_state, &msg, 0);
        }

        /* ---- 5. Heartbeat log (1 Hz) — 3 lines per second ---- */
        g_phase_ml = PHASE_ML_HEARTBEAT;
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

        /* Cycle duration measurement (ms). Lets T_WDG / 1Hz log catch
         * gradual slowdowns before they become full stalls. */
        g_cycle_ms_ml = (uint32_t)((xTaskGetTickCount() - cycle_start)
                                   * portTICK_PERIOD_MS);

        /* Canary: prove T_ML completed a full loop iteration. */
        g_canary_ml++;
    }
}

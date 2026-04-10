/**
 * @file    t_pid.c
 * @brief   T_PID task — full control loop (Phase 3b).
 *
 * 20 Hz periodic loop:
 *
 *   1. Read both IR sensors via the I2C1 mutex.
 *   2. Drain q_ctrl_to_pid (coalesce any backlog) → current_cmd.
 *   3. Pick PID input = max(IR1, IR2) for safety: always track the
 *      hotter sensor so over-temperature is detected fastest.
 *   4. Over-temp check: if either IR > OVERTEMP_HARD_C, send a fault
 *      request to T_STATE and force heater_duty = 0 for this cycle.
 *   5. FSM state check: if g_fsm_state == FAULT, force heater_duty = 0
 *      and fan_duty = 0 regardless of any other input. T_STATE shuts
 *      down PC5 right after entering FAULT, so this last cycle must
 *      leave actuators in their safe state before VDD drops.
 *   6. PID compute (only if pid_enabled in current_cmd).
 *   7. Write heater PWM, fan PWM, LED pattern via actuators.
 *   8. Heartbeat log every second.
 *
 * The IR read can fail (HAL_ERROR / TIMEOUT) without crashing the loop.
 * On failure we keep the previous valid measurement and increment a
 * fault counter — three consecutive failures → q_fault_req with
 * FAULT_REASON_SENSOR_TIMEOUT.
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
#include "app/actuators.h"
#include "app/pid.h"
#include "app/rtt_log.h"

/* Three consecutive read failures on either sensor escalates to FAULT. */
#define IR_READ_FAIL_LIMIT   3u

/* Encode a Celsius value as signed centi-Celsius for the integer-only
 * RTT logger. 25.43 °C → 2543, -1.20 °C → -120. */
static uint32_t celsius_to_centi_uint(float c)
{
    int32_t centi = (int32_t)(c * 100.0f + (c >= 0 ? 0.5f : -0.5f));
    return (uint32_t)centi;
}

void t_pid_run(void *arg)
{
    (void)arg;

    rtt_log_str("[t_pid] start (Phase 3b: full control loop)");

    /* Bring up actuators FIRST so the timer is generating zero-duty PWM
     * before any code path can ask for a non-zero output. */
    actuators_init();

    /* PID instance — gains from config.h, output range 0..1000 to match
     * TIM1 ARR. PLACEHOLDER gains; tune in Phase 6. */
    pid_t heater_pid;
    pid_init(&heater_pid, PID_KP, PID_KI, PID_KD,
             (float)PID_OUTPUT_MIN, (float)PID_OUTPUT_MAX);

    /* Sensor subsystem bring-up — 200 ms wait + bus scan + IR ping. */
    if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(500)) == pdTRUE) {
        (void)sensors_i2c_init(&hi2c1);
        xSemaphoreGive(mtx_i2c1);
    } else {
        rtt_log_str("[t_pid] init: mtx_i2c1 timeout, continuing anyway");
    }

    /* Default command matches the boot FSM state (FORCE_DOWN). T_STATE
     * will publish a fresh ctrl_cmd_t shortly after we start, but having
     * a sensible default avoids a brief window where PWM = 0 because
     * pid_enabled defaulted to false. */
    ctrl_cmd_t current_cmd = {
        .setpoint_c   = TEMP_COOL_C,
        .fan_duty_pct = FAN_DUTY_FORCE_DOWN,
        .led_pattern  = LED_FADE_YELLOW,
        .pid_enabled  = 1u,
        .pad          = {0, 0, 0},
    };

    float    last_valid_meas = (float)TEMP_COOL_C;
    uint32_t ir_fail_count   = 0u;

    TickType_t next_wake = xTaskGetTickCount();
    uint32_t   tick = 0;

    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PERIOD_T_PID_MS));
        tick++;

        /* ---- 1. Read both IR sensors ---- */
        float ir1_c = 0.0f, ir2_c = 0.0f;
        HAL_StatusTypeDef st1 = HAL_ERROR, st2 = HAL_ERROR;

        if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(10)) == pdTRUE) {
            st1 = tbp_h70_read_target_c(&hi2c1, IR_SENSOR_1_I2C_ADDR_7B, &ir1_c);
            st2 = tbp_h70_read_target_c(&hi2c1, IR_SENSOR_2_I2C_ADDR_7B, &ir2_c);
            xSemaphoreGive(mtx_i2c1);
        }

        /* ---- 2. Drain ctrl_cmd queue (keep last) ---- */
        ctrl_cmd_t cmd_in;
        while (xQueueReceive(q_ctrl_to_pid, &cmd_in, 0) == pdTRUE) {
            current_cmd = cmd_in;
        }

        /* ---- 3. Pick PID input = max(IR1, IR2), with fault tracking ---- */
        float measurement;
        bool  ir_ok = false;
        if (st1 == HAL_OK && st2 == HAL_OK) {
            measurement = (ir1_c > ir2_c) ? ir1_c : ir2_c;
            ir_ok = true;
        } else if (st1 == HAL_OK) {
            measurement = ir1_c;
            ir_ok = true;
        } else if (st2 == HAL_OK) {
            measurement = ir2_c;
            ir_ok = true;
        } else {
            measurement = last_valid_meas;  /* keep stale value, count failure */
        }

        if (ir_ok) {
            last_valid_meas = measurement;
            ir_fail_count = 0u;
        } else {
            ir_fail_count++;
            if (ir_fail_count >= IR_READ_FAIL_LIMIT) {
                fault_req_t freq = { .reason = FAULT_REASON_SENSOR_TIMEOUT, .pad = {0,0,0} };
                (void)xQueueSendToBack(q_fault_req, &freq, 0);
                ir_fail_count = 0u;  /* avoid retriggering every cycle */
            }
        }

        /* ---- 4. Over-temperature: raw value check, immediate fault ---- */
        bool overtemp = false;
        if (st1 == HAL_OK && ir1_c > (float)OVERTEMP_HARD_C) overtemp = true;
        if (st2 == HAL_OK && ir2_c > (float)OVERTEMP_HARD_C) overtemp = true;
        if (overtemp) {
            fault_req_t freq = { .reason = FAULT_REASON_OVERTEMP, .pad = {0,0,0} };
            (void)xQueueSendToBack(q_fault_req, &freq, 0);
        }

        /* ---- 5. PID compute ---- */
        uint16_t heater_duty = 0;
        if (current_cmd.pid_enabled && ir_ok && !overtemp) {
            float output = pid_compute(&heater_pid,
                                       (float)current_cmd.setpoint_c,
                                       measurement,
                                       PID_DT_S);
            if (output < 0.0f) output = 0.0f;
            heater_duty = (uint16_t)output;
        }

        /* ---- 6. FSM FAULT override (last word, always wins) ---- */
        uint8_t fan_pct = current_cmd.fan_duty_pct;
        led_pattern_t led = (led_pattern_t)current_cmd.led_pattern;
        if (g_fsm_state == (uint32_t)FSM_FAULT) {
            heater_duty = 0;
            fan_pct = 0;
            led = LED_FLASH_RED;
            pid_reset(&heater_pid);  /* clean restart if FAULT recovers (it doesn't, but be tidy) */
        }

        /* ---- 7. Write actuators ---- */
        actuators_set_heater_duty(heater_duty);
        actuators_set_fan_duty_pct(fan_pct);
        actuators_set_led_pattern(led);

        /* ---- 8. Heartbeat log (1 Hz) ---- */
        if ((tick % 20) == 0) {
            rtt_log_hb("[t_pid]",
                       " ir_cC=",  celsius_to_centi_uint(measurement),
                       " sp_cC=",  (uint32_t)((int32_t)current_cmd.setpoint_c * 100),
                       " heat=",   (uint32_t)heater_duty,
                       " fan=",    (uint32_t)fan_pct);

            /* Structured log to q_log for T_LOGGER (Phase 5). Non-blocking;
             * drop silently if the queue is full — the direct RTT heartbeat
             * above is the primary bring-up output, q_log is the future
             * SD/UART path. */
            log_msg_t lm = {
                .timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                .level        = LOG_LVL_INFO,
                .source       = LOG_SRC_PID,
                .code         = 0,  /* 0 = periodic heartbeat */
                .value        = (int32_t)celsius_to_centi_uint(measurement),
                .extra        = (int32_t)heater_duty,
            };
            (void)xQueueSendToBack(q_log, &lm, 0);
        }
    }
}

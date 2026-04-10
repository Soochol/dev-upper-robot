/**
 * @file    t_pid.c
 * @brief   T_PID task — IR read + (Phase 3a stub) PID + actuators.
 *
 * Phase 3a (this commit) wires up the IR sensor read path only:
 *   - Bring up sensors_i2c (200 ms wait + bus scan + IR ping).
 *   - Each tick, take mtx_i2c1, read both IR target temperatures, log them.
 *   - PID computation, heater PWM, fan PWM, LED GPIO are still stubbed
 *     out. They arrive in Phase 3b once the sensor read path is verified
 *     on hardware and the live IR addresses are confirmed.
 *
 * Splitting Phase 3 into 3a (sensors only) and 3b (PID + actuators)
 * keeps the bring-up debug loop short — if the IR read does not work,
 * the issue is isolated to addressing or HAL_I2C_Mem_Read parameters,
 * not buried under PID and PWM bugs.
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "i2c.h"             /* hi2c1 */
#include "app/config.h"
#include "app/ipc.h"
#include "app/sensors_i2c.h"
#include "app/rtt_log.h"

/* Pack a Celsius value into a stable integer for the printf-free RTT
 * logger: signed centi-Celsius (e.g. 25.43 °C → 2543). The logger only
 * has uint32 formatters so we cast through a temporary signed int and
 * then to uint32 for transport — readers reverse the conversion. */
static uint32_t celsius_to_centi_uint(float c)
{
    int32_t centi = (int32_t)(c * 100.0f + (c >= 0 ? 0.5f : -0.5f));
    return (uint32_t)centi;
}

void t_pid_run(void *arg)
{
    (void)arg;

    rtt_log_str("[t_pid] start (Phase 3a: IR read only)");

    /* One-shot sensor subsystem bring-up. The 200 ms internal delay
     * blocks this task only — the rest of the system continues to run
     * (T_STATE keeps the FSM ticking, defaultTask keeps emitting
     * heartbeats and auto-triggers). */
    if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(500)) == pdTRUE) {
        (void)sensors_i2c_init(&hi2c1);
        xSemaphoreGive(mtx_i2c1);
    } else {
        rtt_log_str("[t_pid] init: mtx_i2c1 timeout, continuing anyway");
    }

    TickType_t next_wake = xTaskGetTickCount();
    uint32_t   tick = 0;

    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PERIOD_T_PID_MS));
        tick++;

        float ir1_c = 0.0f, ir2_c = 0.0f;
        HAL_StatusTypeDef st1 = HAL_ERROR, st2 = HAL_ERROR;

        if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(10)) == pdTRUE) {
            st1 = tbp_h70_read_target_c(&hi2c1,
                                        IR_SENSOR_1_I2C_ADDR_7B,
                                        &ir1_c);
            st2 = tbp_h70_read_target_c(&hi2c1,
                                        IR_SENSOR_2_I2C_ADDR_7B,
                                        &ir2_c);
            xSemaphoreGive(mtx_i2c1);
        }

        /* Phase 3a heartbeat: every second, log both IR readings as
         * centi-Celsius integers along with the per-sensor HAL status.
         * status code: 0 = HAL_OK, non-zero = HAL_ERROR/BUSY/TIMEOUT. */
        if ((tick % 20) == 0) {
            rtt_log_hb("[t_pid]",
                       " ir1_cC=", celsius_to_centi_uint(ir1_c),
                       " st1=",    (uint32_t)st1,
                       " ir2_cC=", celsius_to_centi_uint(ir2_c),
                       " st2=",    (uint32_t)st2);
        }
    }
}

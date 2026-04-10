/**
 * @file    t_pid.c
 * @brief   T_PID task — IR read, PID compute, heater/fan/LED actuation.
 *
 * Phase 1 stub: same shape as the eventual real loop (mtx_i2c1 try, work,
 * release, command poll, log) but with no I/O. This validates that the PI
 * mutex creation in freertos.c works and that this task can be scheduled
 * at PRIO_T_PID without starving lower priority work.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "app/config.h"
#include "app/ipc.h"
#include "app/rtt_log.h"

void t_pid_run(void *arg)
{
    (void)arg;
    TickType_t next_wake = xTaskGetTickCount();
    uint32_t   tick = 0;

    rtt_log_str("[t_pid] start");

    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PERIOD_T_PID_MS));
        tick++;

        /* Phase 1: prove the mutex is takeable. Real I2C work in Phase 3. */
        if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(10)) == pdTRUE) {
            /* would do IR#1, IR#2 read here in Phase 3 */
            xSemaphoreGive(mtx_i2c1);
        }

        if ((tick % 20) == 0) {
            rtt_log_kv("[t_pid] tick=", tick);
        }
    }
}

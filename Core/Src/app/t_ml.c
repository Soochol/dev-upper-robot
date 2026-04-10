/**
 * @file    t_ml.c
 * @brief   T_ML task — sensor acquisition + trigger provider evaluation.
 *
 * Phase 1 stub: 25 ms offset relative to T_PID, then mutex try. The offset
 * is the entire reason the I2C1 PI mutex almost never contests in normal
 * operation; verifying the offset works is part of the Phase 1 acceptance.
 *
 * Real ICM42670P + ADS1115 acquisition and trigger_eval() arrive in Phase 4.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "app/config.h"
#include "app/ipc.h"
#include "app/rtt_log.h"

void t_ml_run(void *arg)
{
    (void)arg;

    /* Establish the 25 ms offset relative to T_PID. Both tasks start at the
     * same tick when the scheduler launches, so we sleep half a period here
     * once before entering the periodic loop. */
    vTaskDelay(pdMS_TO_TICKS(OFFSET_T_ML_MS));

    TickType_t next_wake = xTaskGetTickCount();
    uint32_t   tick = 0;

    rtt_log_str("[t_ml] start (offset 25ms)");

    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PERIOD_T_ML_MS));
        tick++;

        if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(10)) == pdTRUE) {
            /* would do ICM42670P FIFO + ADS1115 read here in Phase 4 */
            xSemaphoreGive(mtx_i2c1);
        }

        if ((tick % 20) == 0) {
            rtt_log_kv("[t_ml] tick=", tick);
        }
    }
}

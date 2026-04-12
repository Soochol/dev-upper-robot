/**
 * @file    t_wdg.c
 * @brief   T_WDG task — dedicated hardware watchdog manager.
 *
 * 10 Hz periodic task. Per tick:
 *   1. Read canary counters from T_STATE, T_PID, T_ML.
 *   2. If a canary has not changed for CANARY_MAX_MISS consecutive
 *      checks, stop kicking the IWDG → MCU resets in ~400 ms.
 *   3. If all canaries are alive, call HAL_IWDG_Refresh().
 *
 * Boot grace: for the first WDG_BOOT_GRACE_TICKS iterations, kick
 * unconditionally. T_ML needs ~200 ms for IMU init (soft reset +
 * stabilization delays), during which its canary does not increment.
 *
 * IWDG is initialized inside this task (not main.c) so the countdown
 * never starts before the kicker is running [C1].
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "iwdg.h"
#include "app/config.h"
#include "app/ipc.h"
#include "app/rtt_log.h"

void t_wdg_run(void *arg)
{
    (void)arg;

    /* Start IWDG HERE, not in main.c. The countdown begins now, and
     * we are already running — no timing gap [C1]. */
    MX_IWDG_Init();
    HAL_IWDG_Refresh(&hiwdg);

    rtt_log_str("[t_wdg] start, IWDG active (~400ms)");

    uint32_t last_state = 0, last_pid = 0, last_ml = 0;
    uint8_t  miss_state = 0, miss_pid = 0, miss_ml = 0;

    TickType_t next_wake = xTaskGetTickCount();
    uint32_t   tick = 0;

    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PERIOD_T_WDG_MS));
        tick++;

        /* Boot grace: unconditional kick while tasks are initializing. */
        if (tick <= WDG_BOOT_GRACE_TICKS) {
            HAL_IWDG_Refresh(&hiwdg);
            continue;
        }

        /* Sample canaries. */
        uint32_t cur_state = g_canary_state;
        uint32_t cur_pid   = g_canary_pid;
        uint32_t cur_ml    = g_canary_ml;

        /* [W6] Clamp at CANARY_MAX_MISS to prevent uint8_t wrap-around. */
        miss_state = (cur_state == last_state)
            ? ((miss_state < CANARY_MAX_MISS) ? (miss_state + 1) : CANARY_MAX_MISS)
            : 0;
        miss_pid = (cur_pid == last_pid)
            ? ((miss_pid < CANARY_MAX_MISS) ? (miss_pid + 1) : CANARY_MAX_MISS)
            : 0;
        miss_ml = (cur_ml == last_ml)
            ? ((miss_ml < CANARY_MAX_MISS) ? (miss_ml + 1) : CANARY_MAX_MISS)
            : 0;

        last_state = cur_state;
        last_pid   = cur_pid;
        last_ml    = cur_ml;

        /* All alive → kick. Any stalled → stop kicking → IWDG resets MCU. */
        if (miss_state < CANARY_MAX_MISS &&
            miss_pid   < CANARY_MAX_MISS &&
            miss_ml    < CANARY_MAX_MISS) {
            HAL_IWDG_Refresh(&hiwdg);
        } else {
            /* Log which task stalled — visible in RTT right before reset. */
            rtt_log_hb("[t_wdg] STALL",
                       " st=",  (uint32_t)miss_state,
                       " pid=", (uint32_t)miss_pid,
                       " ml=",  (uint32_t)miss_ml,
                       (const char *)0, 0);
        }

        /* 1 Hz heartbeat for debugging. */
        if ((tick % 10) == 0) {
            rtt_log_hb("[t_wdg]",
                       " tick=", tick,
                       " st=",   (uint32_t)miss_state,
                       " pid=",  (uint32_t)miss_pid,
                       " ml=",   (uint32_t)miss_ml);
        }
    }
}

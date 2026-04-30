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
#include "main.h"           /* OUT_LED_R1/R2_Pin / GPIO_Port for stall LED */
#include "stm32f1xx_hal.h"
#include "app/config.h"
#include "app/ipc.h"
#include "app/actuators.h"
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
            /* Stall failsafe: latch the flag FIRST so T_PID's next tick
             * (50 ms cadence) sees it and skips its own actuator writes.
             * Without this gate, T_PID would overwrite our emergency-off
             * within ε of T_WDG yielding back. __DSB() pins the store
             * order so the flag is visible before any subsequent write
             * lands on the bus. */
            g_failsafe_active = 1;
            __DSB();

            /* Force heater PWM and fan power off. Repeats every 100 ms
             * until IWDG fires (~3-4 writes total during the grace). */
            actuators_emergency_off();

            /* Best-effort visual signal during the ~400 ms window before
             * IWDG fires. Direct GPIO writes bypass actuators_set_led_pattern
             * to avoid touching its static state (prev_pat, blink_tick) —
             * functional ODR write is atomic on Cortex-M3. The post-reset
             * boot indicator (main.c) catches the case where this brief
             * window is missed. */
            HAL_GPIO_WritePin(OUT_LED_R1_GPIO_Port, OUT_LED_R1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OUT_LED_R2_GPIO_Port, OUT_LED_R2_Pin, GPIO_PIN_SET);

            /* Log which task stalled — visible in RTT right before reset. */
            rtt_log_hb("[t_wdg] STALL",
                       " st=",  (uint32_t)miss_state,
                       " pid=", (uint32_t)miss_pid,
                       " ml=",  (uint32_t)miss_ml,
                       (const char *)0, 0);

            /* Phase + last-cycle-ms diagnostics. Tells us *which line of
             * code* a frozen task was executing and whether cycle times
             * had been creeping up before the stall. */
            rtt_log_hb("[t_wdg] PHASE",
                       " st=",  (uint32_t)g_phase_state,
                       " pid=", (uint32_t)g_phase_pid,
                       " ml=",  (uint32_t)g_phase_ml,
                       (const char *)0, 0);
            rtt_log_hb("[t_wdg] CYCMS",
                       " st=",  g_cycle_ms_state,
                       " pid=", g_cycle_ms_pid,
                       " ml=",  g_cycle_ms_ml,
                       (const char *)0, 0);
        }

        /* 1 Hz heartbeat for debugging. */
        if ((tick % 10) == 0) {
            rtt_log_hb("[t_wdg]",
                       " tick=", tick,
                       " st=",   (uint32_t)miss_state,
                       " pid=",  (uint32_t)miss_pid,
                       " ml=",   (uint32_t)miss_ml);
            /* Cycle-time tracker — if any task starts taking >50ms per
             * cycle regularly, a stall is brewing. Steady-state values
             * are normally ~1-5 ms. */
            rtt_log_hb("[t_wdg] CYC",
                       " st=",  g_cycle_ms_state,
                       " pid=", g_cycle_ms_pid,
                       " ml=",  g_cycle_ms_ml,
                       (const char *)0, 0);

            /* T_PID per-phase wall-clock max for the last 1 s window.
             * Phase indices: 0=IDLE 1=MUTEX 2=IRR 3=DRAIN 4=FCHK
             *                5=RECOV 6=OTEMP 7=PID 8=ACT 9=HB
             * Read+reset the array so each 1 s window is independent. */
            rtt_log_hb("[t_wdg] PIDMS_A",
                       " p1=", g_pid_phase_ms[1],
                       " p2=", g_pid_phase_ms[2],
                       " p3=", g_pid_phase_ms[3],
                       " p4=", g_pid_phase_ms[4]);
            rtt_log_hb("[t_wdg] PIDMS_B",
                       " p5=", g_pid_phase_ms[5],
                       " p6=", g_pid_phase_ms[6],
                       " p7=", g_pid_phase_ms[7],
                       " p8=", g_pid_phase_ms[8]);
            for (uint8_t i = 0; i < PID_PHASE_COUNT; i++) {
                g_pid_phase_ms[i] = 0;
            }
        }
    }
}

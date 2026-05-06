/**
 * @file    trigger_timer.c
 * @brief   Time-based test cycle trigger — FU/FD infinite burn-in loop.
 *
 * Ignores sensor input, alternates FORCE_UP/FORCE_DOWN by elapsed time:
 *
 *   boot ─► FD (TEST_FD_DURATION_MS) ─► FU (TEST_FU_DURATION_MS) ─► FD ─► ...
 *
 * If FSM transitions externally (safety_timeout, FAULT), the timer
 * re-anchors so the next cycle begins from that state's entry tick.
 *
 * RTT diagnostics:
 *   - boot banner: provider name + FU/FD durations (one shot)
 *   - per-transition: cycle count for FU and FD, FAULT marker on entry
 *
 * Cycle counters (s_fu_count, s_fd_count) are static globals so GDB
 * watch can read them live during burn-in.
 */

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app/config.h"
#include "app/fsm.h"
#include "app/rtt_log.h"
#include "app/trigger.h"
#include "app/trigger_timer.h"

static TickType_t  s_state_entered_tick = 0;
static fsm_state_t s_last_state         = FSM_FORCE_DOWN;
static bool        s_initialized        = false;
static uint32_t    s_fu_count           = 0;  /* completed FU entries */
static uint32_t    s_fd_count           = 0;  /* completed FD entries */

static uint8_t timer_eval(const sensor_snapshot_t *snap, fsm_state_t cur)
{
    (void)snap;  /* sensor input ignored — pure time-based trigger */

    const TickType_t now = xTaskGetTickCount();

    /* First call: anchor + boot banner. Defends against `now - 0` blow-up
     * if the static initializer was non-zero on some toolchain. */
    if (!s_initialized) {
        s_initialized        = true;
        s_last_state         = cur;
        s_state_entered_tick = now;
        rtt_log_hb("[timer] *** TEST CYCLE BUILD ***",
                   "FU_ms", (uint32_t)TEST_FU_DURATION_MS,
                   "FD_ms", (uint32_t)TEST_FD_DURATION_MS,
                   NULL, 0,
                   NULL, 0);
        return TRIG_EVENT_NONE;
    }

    /* External state transition (FSM moved without our event): re-anchor
     * timer + log cycle entry for burn-in trace. */
    if (cur != s_last_state) {
        s_last_state         = cur;
        s_state_entered_tick = now;
        if (cur == FSM_FORCE_UP) {
            rtt_log_kv("[timer] FU# begin n=", ++s_fu_count);
        } else if (cur == FSM_FORCE_DOWN) {
            rtt_log_kv("[timer] FD# begin n=", ++s_fd_count);
        } else if (cur == FSM_FAULT) {
            rtt_log_str("[timer] FAULT — cycle stopped");
        }
        return TRIG_EVENT_NONE;
    }

    /* Wrap-safe: TickType_t is unsigned, subtraction handles 32-bit
     * wraparound correctly (~49.7 days at 1 ms tick). */
    const uint32_t elapsed_ms =
        (uint32_t)((now - s_state_entered_tick) * portTICK_PERIOD_MS);

    if (cur == FSM_FORCE_DOWN && elapsed_ms >= TEST_FD_DURATION_MS) {
        return TRIG_EVENT_FORCE_UP;
    }
    if (cur == FSM_FORCE_UP && elapsed_ms >= TEST_FU_DURATION_MS) {
        return TRIG_EVENT_FORCE_DOWN;
    }
    return TRIG_EVENT_NONE;
}

const trigger_provider_t trig_timer = {
    .name = "timer",
    .eval = timer_eval,
};

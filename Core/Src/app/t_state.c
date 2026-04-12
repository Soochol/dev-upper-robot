/**
 * @file    t_state.c
 * @brief   T_STATE task — FSM tick + safety timeout + ctrl publication.
 *
 * 20 Hz periodic task. Per tick:
 *   1. Drain q_trigger_to_state and q_fault_req (non-blocking).
 *   2. Check FORCE_UP safety timeout (60 s elapsed → SAFETY_TIMEOUT event).
 *   3. Apply each event via fsm_next() in priority order:
 *        FAULT_REQUESTED > SAFETY_TIMEOUT > TRIGGER
 *      so a sensor fault during a trigger event still routes to FAULT.
 *   4. On state change: log transition, publish ctrl_cmd_t to T_PID with
 *      the new setpoint/fan/LED values from state_table.
 *   5. On FAULT entry: log the event. T_PID forces heater=0/fan=0.
 *
 * Buttons are not used (D13). The boot initial state is FORCE_DOWN (D8).
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"           /* OUT_PWR_HOLD_Pin / GPIO_Port */
#include "stm32f1xx_hal.h"
#include "app/config.h"
#include "app/ipc.h"
#include "app/fsm.h"
#include "app/rtt_log.h"

/* ------------------------------------------------------------------------ */
/* Local helpers                                                            */
/* ------------------------------------------------------------------------ */

/* Publish the per-state outputs to T_PID. Non-blocking — if the queue is
 * full we drop and rely on the next tick. q_ctrl_to_pid depth is 4 and
 * T_PID consumes once per tick at the same 20 Hz, so the queue should
 * never have more than one outstanding item. */
static void publish_ctrl(fsm_state_t state)
{
    const fsm_output_t *out = fsm_output(state);
    ctrl_cmd_t cmd = {
        .setpoint_c   = out->setpoint_c,
        .fan_duty_pct = out->fan_duty_pct,
        .led_pattern  = out->led_pattern,
        .pid_enabled  = out->pid_enabled ? 1u : 0u,
        .pad          = {0, 0, 0},
    };
    (void)xQueueSendToBack(q_ctrl_to_pid, &cmd, 0);
}

/* ------------------------------------------------------------------------ */
/* Task body                                                                */
/* ------------------------------------------------------------------------ */

void t_state_run(void *arg)
{
    (void)arg;

    /* Boot-time initial state per D8: FORCE_DOWN (fail-safe).
     * g_fsm_state is the public copy other tasks read for safety checks. */
    fsm_state_t state = FSM_FORCE_DOWN;
    g_fsm_state = (uint32_t)state;
    publish_ctrl(state);
    rtt_log_str("[t_state] start, init=FORCE_DOWN");

    /* When did we enter the current state? Used by safety timeout. */
    TickType_t state_entered_tick = xTaskGetTickCount();
    TickType_t next_wake          = xTaskGetTickCount();
    /* Bring-up heartbeat counter — idle log every 20 ticks (= 1 s at
     * 20 Hz). Without this, the task is silent between transitions and
     * the user has no way to distinguish "alive but idle" from "stuck".
     * Removable in Phase 6 once the system is trusted. */
    uint32_t tick = 0;

    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PERIOD_T_STATE_MS));
        tick++;

        fsm_event_t event = FSM_EVT_NONE;

        /* Priority 1: fault requests from T_PID and/or T_ML. Drain the
         * entire queue so all fault reasons are logged [C3]. Only the
         * first triggers the FSM transition; subsequent ones in FAULT
         * state are no-ops via fsm_next(). */
        {
            fault_req_t freq;
            while (xQueueReceive(q_fault_req, &freq, 0) == pdTRUE) {
                if (event == FSM_EVT_NONE) {
                    event = FSM_EVT_FAULT_REQUESTED;
                }
                rtt_log_kv("[t_state] fault_req reason=", (uint32_t)freq.reason);
            }
        }

        /* Priority 2: safety timeout (FORCE_UP only). Skip if a higher
         * priority event already won this tick. */
        if (event == FSM_EVT_NONE && state == FSM_FORCE_UP) {
            const fsm_output_t *out = fsm_output(state);
            uint32_t elapsed_ms = (xTaskGetTickCount() - state_entered_tick)
                                  * portTICK_PERIOD_MS;
            if (out->safety_max_ms > 0 && elapsed_ms >= out->safety_max_ms) {
                event = FSM_EVT_SAFETY_TIMEOUT;
                rtt_log_kv("[t_state] safety_to elapsed_ms=", elapsed_ms);
            }
        }

        /* Priority 3: trigger event from T_ML (via Phase 2 emulator for
         * now). Drain only one per tick to keep the FSM serializable. */
        if (event == FSM_EVT_NONE) {
            trig_msg_t tmsg;
            if (xQueueReceive(q_trigger_to_state, &tmsg, 0) == pdTRUE) {
                event = (fsm_event_t)tmsg.event;
            }
        }

        /* Apply the event. fsm_next is pure — no side effects. */
        fsm_state_t next = fsm_next(state, event);
        if (next != state) {
            rtt_log_hb("[t_state] transition",
                       " from=", (uint32_t)state,
                       " evt=",  (uint32_t)event,
                       " to=",   (uint32_t)next,
                       (const char *)0, 0);
            state              = next;
            g_fsm_state        = (uint32_t)state;
            state_entered_tick = xTaskGetTickCount();
            publish_ctrl(state);

            if (state == FSM_FAULT) {
                rtt_log_str("[t_state] FAULT: all outputs off");
            }
        }

        /* Idle heartbeat: prove the task is alive between transitions.
         * Once per second at 20 Hz. */
        if ((tick % 20) == 0) {
            rtt_log_hb("[t_state]",
                       " tick=", tick,
                       " fsm=",  (uint32_t)state,
                       (const char *)0, 0,
                       (const char *)0, 0);
        }

        /* Canary: prove T_STATE completed a full loop iteration. */
        g_canary_state++;
    }
}

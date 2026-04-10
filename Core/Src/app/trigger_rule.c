/**
 * @file    trigger_rule.c
 * @brief   Rule-based trigger provider — FSR threshold with hysteresis + debounce.
 *
 * Hysteresis (Schmitt-trigger):
 *   fsr > FSR_THRESHOLD_UP_RAW   AND state == FORCE_DOWN → TRIG_UP
 *   fsr < FSR_THRESHOLD_DOWN_RAW AND state == FORCE_UP   → TRIG_DOWN
 *   Values between the two thresholds → no event (dead zone).
 *
 * Debounce (minimum hold time):
 *   After firing a trigger event, subsequent events are suppressed for
 *   TRIGGER_DEBOUNCE_MS. This prevents FSR jitter near the threshold
 *   from causing rapid UP/DOWN oscillation that was observed in Phase 4
 *   testing (t=17~22s in the RTT log).
 *
 * Phase 4 v1: FSR only. IMU tilt is available in the snapshot but not
 * consulted — Phase 4b will add tilt-based triggers.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "app/trigger.h"
#include "app/config.h"

static uint8_t rule_eval(const sensor_snapshot_t *snap, fsm_state_t cur)
{
    /* Debounce: suppress events for TRIGGER_DEBOUNCE_MS after the last
     * trigger. Uses a static tick counter so state persists across calls.
     * This is safe because trigger_eval (and therefore rule_eval) is only
     * ever called from one task (T_ML), so the static is single-writer. */
    static TickType_t last_trigger_tick = 0;
    static uint8_t    ever_triggered    = 0;

    if (ever_triggered) {
        TickType_t elapsed = (xTaskGetTickCount() - last_trigger_tick)
                             * portTICK_PERIOD_MS;
        if (elapsed < TRIGGER_DEBOUNCE_MS) {
            return TRIG_EVENT_NONE;
        }
    }

    int16_t fsr = snap->fsr_raw;
    uint8_t event = TRIG_EVENT_NONE;

    if (cur == FSM_FORCE_DOWN && fsr > (int16_t)FSR_THRESHOLD_UP_RAW) {
        event = TRIG_EVENT_FORCE_UP;
    } else if (cur == FSM_FORCE_UP && fsr < (int16_t)FSR_THRESHOLD_DOWN_RAW) {
        event = TRIG_EVENT_FORCE_DOWN;
    }

    if (event != TRIG_EVENT_NONE) {
        last_trigger_tick = xTaskGetTickCount();
        ever_triggered    = 1;
    }

    return event;
}

const trigger_provider_t trig_rule = {
    .name = "rule",
    .eval = rule_eval,
};

/**
 * @file    trigger_rule.c
 * @brief   Rule-based trigger provider — FSR threshold with hysteresis.
 *
 * Phase 4 v1: uses FSR only. IMU tilt is available in the snapshot but
 * not consulted — adding it to the logic is a single if-statement away
 * (Phase 4b).
 *
 * Hysteresis model:
 *
 *   FSR raw value
 *   ─────────────────────────────────────────
 *   ▲                        ╔═══════════╗
 *   │  FSR_THRESHOLD_UP_RAW  ║ dead zone ║
 *   │  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ║ (no event)║
 *   │  FSR_THRESHOLD_DOWN_RAW╚═══════════╝
 *   │
 *   0 ──────────────────────────────────────→ time
 *
 *   fsr > FSR_THRESHOLD_UP_RAW   AND state == FORCE_DOWN → TRIGGER_FORCE_UP
 *   fsr < FSR_THRESHOLD_DOWN_RAW AND state == FORCE_UP   → TRIGGER_FORCE_DOWN
 *
 * The state guard prevents re-triggering: once in FORCE_UP, only a DROP
 * below the lower threshold can bring the system back down. Values between
 * the two thresholds are ignored (dead zone). This is the standard
 * Schmitt-trigger pattern used in every debounced digital input.
 */

#include "app/trigger.h"
#include "app/config.h"

static uint8_t rule_eval(const sensor_snapshot_t *snap, fsm_state_t cur)
{
    int16_t fsr = snap->fsr_raw;

    /* In FORCE_DOWN, wait for pressure above the upper threshold to trigger UP. */
    if (cur == FSM_FORCE_DOWN && fsr > (int16_t)FSR_THRESHOLD_UP_RAW) {
        return TRIG_EVENT_FORCE_UP;
    }

    /* In FORCE_UP, wait for pressure below the lower threshold to trigger DOWN. */
    if (cur == FSM_FORCE_UP && fsr < (int16_t)FSR_THRESHOLD_DOWN_RAW) {
        return TRIG_EVENT_FORCE_DOWN;
    }

    /* Dead zone or FAULT state — no event. */
    return TRIG_EVENT_NONE;
}

const trigger_provider_t trig_rule = {
    .name = "rule",
    .eval = rule_eval,
};

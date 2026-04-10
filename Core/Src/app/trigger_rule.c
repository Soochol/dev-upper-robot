/**
 * @file    trigger_rule.c
 * @brief   Rule-based trigger provider — FSR + IMU tilt with hysteresis + debounce.
 *
 * Trigger logic (OR combination):
 *   FORCE_UP  when: FSR > FSR_THRESHOLD_UP_RAW  OR  |tilt| > IMU_TILT_UP_DEG
 *   FORCE_DOWN when: FSR < FSR_THRESHOLD_DOWN_RAW  OR  |tilt| < IMU_TILT_DOWN_DEG
 *
 * Product context: wearable forearm device. Arm lift = FSR pressure up +
 * tilt angle increases. Arm lower = FSR pressure down + tilt returns to 0.
 * Using absolute tilt (fabsf) so the trigger is independent of PCB mounting
 * orientation — whether the lift produces positive or negative tilt.
 *
 * Hysteresis: separate UP and DOWN thresholds for both FSR and tilt prevent
 * oscillation when the sensor value hovers near a single threshold.
 *
 * Debounce: after any trigger fires, subsequent events are suppressed for
 * TRIGGER_DEBOUNCE_MS to prevent rapid state flip-flop.
 */

#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "app/trigger.h"
#include "app/config.h"

static uint8_t rule_eval(const sensor_snapshot_t *snap, fsm_state_t cur)
{
    /* Debounce: suppress events for TRIGGER_DEBOUNCE_MS after the last
     * trigger. Single-writer safe (only T_ML calls this). */
    static TickType_t last_trigger_tick = 0;
    static uint8_t    ever_triggered    = 0;

    if (ever_triggered) {
        TickType_t elapsed = (xTaskGetTickCount() - last_trigger_tick)
                             * portTICK_PERIOD_MS;
        if (elapsed < TRIGGER_DEBOUNCE_MS) {
            return TRIG_EVENT_NONE;
        }
    }

    int16_t fsr       = snap->fsr_raw;
    float   tilt_abs  = fabsf(snap->imu_tilt_x_deg);
    uint8_t event     = TRIG_EVENT_NONE;

    if (cur == FSM_FORCE_DOWN) {
        /* Arm lift detection: FSR pressure increase OR tilt angle increase.
         * Either sensor alone can trigger — OR provides redundancy. */
        if (fsr > (int16_t)FSR_THRESHOLD_UP_RAW ||
            tilt_abs > (float)IMU_TILT_UP_DEG) {
            event = TRIG_EVENT_FORCE_UP;
        }
    } else if (cur == FSM_FORCE_UP) {
        /* Arm lower detection: FSR pressure decrease AND tilt angle small.
         * Using AND here (not OR) because during arm-up the tilt might
         * briefly pass through the low zone while FSR is still high, and
         * we don't want a premature FORCE_DOWN. Both must agree the arm
         * is actually lowered. */
        if (fsr < (int16_t)FSR_THRESHOLD_DOWN_RAW &&
            tilt_abs < (float)IMU_TILT_DOWN_DEG) {
            event = TRIG_EVENT_FORCE_DOWN;
        }
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

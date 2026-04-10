/**
 * @file    trigger.h
 * @brief   Trigger provider abstraction — strategy pattern for FSM events.
 *
 * sensor_snapshot_t is the common data structure packed by T_ML from the
 * IMU and FSR readings each cycle. A trigger_provider_t eval() function
 * inspects the snapshot and the current FSM state, and returns an event
 * (TRIGGER_FORCE_UP, TRIGGER_FORCE_DOWN, or NONE).
 *
 * Compile-time provider selection (TRIGGER_SOURCE in config.h):
 *   TRIG_SRC_RULE  → trig_rule (FSR threshold with hysteresis)
 *   TRIG_SRC_ML    → trig_ml   (Decision Tree, v1 stub)
 *
 * Phase 4b will add IMU tilt to the rule provider.
 * Phase 6 will replace the ML stub with a real model.
 */

#ifndef APP_TRIGGER_H
#define APP_TRIGGER_H

#include <stdint.h>
#include "app/fsm.h"
#include "app/sensors_i2c.h"

/* ========================================================================
 * Sensor snapshot — packed by T_ML, consumed by trigger_eval()
 * ======================================================================== */

typedef struct {
    int16_t   fsr_raw;          /* ADS1115 latest conversion (signed) */
    imu_raw_t imu;              /* ICM42670P accel + gyro raw */
    float     imu_tilt_x_deg;   /* derived: X-axis tilt relative to boot (deg) */
    float     imu_tilt_y_deg;   /* derived: Y-axis tilt relative to boot (deg) */
} sensor_snapshot_t;

/* ========================================================================
 * Trigger event (reuses fsm_event_t values)
 * ======================================================================== */

/* trig_event_t maps 1:1 to the FSM event enum for trigger-related events.
 * We define it here as a plain uint8 to avoid coupling trigger.h to fsm.h
 * enum values beyond the NONE / UP / DOWN triad. */
#define TRIG_EVENT_NONE       ((uint8_t)FSM_EVT_NONE)
#define TRIG_EVENT_FORCE_UP   ((uint8_t)FSM_EVT_TRIGGER_FORCE_UP)
#define TRIG_EVENT_FORCE_DOWN ((uint8_t)FSM_EVT_TRIGGER_FORCE_DOWN)

/* ========================================================================
 * Trigger provider interface
 * ======================================================================== */

typedef uint8_t (*trig_eval_fn)(const sensor_snapshot_t *snap,
                                fsm_state_t current_state);

typedef struct {
    const char    *name;
    trig_eval_fn   eval;
} trigger_provider_t;

/* Built-in providers. Defined in trigger_rule.c and trigger_ml.c. */
extern const trigger_provider_t trig_rule;
extern const trigger_provider_t trig_ml;

/* ========================================================================
 * Public API — called by T_ML each cycle
 * ======================================================================== */

/**
 * @brief  Evaluate the active trigger provider.
 *
 * The active provider is selected at compile time via TRIGGER_SOURCE
 * in config.h. Runtime switching (Phase 6+) would replace the internal
 * pointer assignment.
 *
 * @param  snap           current sensor snapshot (packed by T_ML)
 * @param  current_state  current FSM state (read from g_fsm_state)
 * @return TRIG_EVENT_FORCE_UP, TRIG_EVENT_FORCE_DOWN, or TRIG_EVENT_NONE
 */
uint8_t trigger_eval(const sensor_snapshot_t *snap, fsm_state_t current_state);

/**
 * @brief  Return the name of the currently active provider ("rule" / "ml").
 */
const char *trigger_provider_name(void);

#endif /* APP_TRIGGER_H */

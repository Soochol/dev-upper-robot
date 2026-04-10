/**
 * @file    fsm.h
 * @brief   Finite state machine type definitions for dev-upper-robot.
 *
 * Pure type/enum declarations only — no FreeRTOS or HAL dependencies. This
 * lets fsm.c compile on the host for unit tests.
 *
 * 3-state machine (D7). Buttons are not used (D13). The boot-time initial
 * state is FORCE_DOWN (fail-safe: heater off, fan max).
 *
 * State diagram:
 *
 *   [boot]
 *      │
 *      ▼
 *   FORCE_DOWN ◀──── trig_down ────┐
 *      │                            │
 *      │ trig_up                    │
 *      ▼                            │
 *   FORCE_UP ──── trig_down ────────┤
 *      │                            │
 *      │ safety_timeout (60s)       │
 *      └────────────────────────────┘
 *
 *   Any state ── fault_req ──▶ FAULT
 *                              │
 *                              ▼
 *                  PC5 release → board power off
 *                  (recoverable only by physical
 *                   power cycle by the user)
 */

#ifndef APP_FSM_H
#define APP_FSM_H

#include <stdint.h>
#include <stdbool.h>

/* FSM state enum. uint32_t-aligned via the volatile uint32_t holding it
 * in ipc.h so reads from other tasks (T_PID safety check) are atomic on
 * Cortex-M3. The order matters: state_table.c indexes its LUT by this
 * enum value, so changing the order without updating state_table.c will
 * mis-route every fsm_output() lookup. FSM_STATE_COUNT must stay last. */
typedef enum {
    FSM_FORCE_DOWN = 0,   /* boot-time initial state, fail-safe */
    FSM_FORCE_UP,
    FSM_FAULT,
    FSM_STATE_COUNT
} fsm_state_t;

/* FSM events. T_STATE consumes these and produces state transitions.
 * No button events — buttons are not used in this product (D13). */
typedef enum {
    FSM_EVT_NONE = 0,
    FSM_EVT_TRIGGER_FORCE_UP,    /* from T_ML trigger provider */
    FSM_EVT_TRIGGER_FORCE_DOWN,  /* from T_ML trigger provider */
    FSM_EVT_FAULT_REQUESTED,     /* from T_PID safety invariant */
    FSM_EVT_SAFETY_TIMEOUT,      /* internal: FORCE_UP stayed > 60 s */
    FSM_EVT_COUNT
} fsm_event_t;

/* Per-state output configuration. Looked up from state_table.c each tick. */
typedef struct {
    int16_t  setpoint_c;     /* PID setpoint in °C, ignored if pid_enabled = false */
    uint8_t  fan_duty_pct;   /* fan PWM duty 0..100 */
    uint8_t  led_pattern;    /* led_pattern_t enum, defined in actuators.h */
    bool     pid_enabled;    /* false in FAULT only */
    uint32_t safety_max_ms;  /* 0 = no timeout (FORCE_DOWN, FAULT) */
} fsm_output_t;

/* Pure transition function. No HAL/FreeRTOS calls — host-testable.
 * Returns the next state given the current state and an event. If the
 * event has no transition defined, returns the current state unchanged. */
fsm_state_t fsm_next(fsm_state_t current, fsm_event_t event);

/* Lookup helper. Returns a pointer to a static const fsm_output_t table
 * entry for the given state. The pointer is valid for the program lifetime. */
const fsm_output_t *fsm_output(fsm_state_t state);

/* Debug helper: state name as a string literal (for RTT logging). */
const char *fsm_state_name(fsm_state_t state);

/* Debug helper: event name as a string literal (for RTT transition logs). */
const char *fsm_event_name(fsm_event_t event);

#endif /* APP_FSM_H */

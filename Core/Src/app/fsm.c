/**
 * @file    fsm.c
 * @brief   Pure 3-state FSM transition logic.
 *
 * Side-effect-free, reentrant, host-testable. T_STATE owns the actual
 * state variable and calls fsm_next() per tick to compute the next state.
 *
 * Transition table (D7, D8, D10, D12, D13):
 *
 *   current      | event                       | next
 *   -------------+-----------------------------+-------------
 *   FORCE_DOWN   | TRIGGER_FORCE_UP            | FORCE_UP
 *   FORCE_DOWN   | FAULT_REQUESTED             | FAULT
 *   FORCE_UP     | TRIGGER_FORCE_DOWN          | FORCE_DOWN
 *   FORCE_UP     | SAFETY_TIMEOUT              | FORCE_DOWN
 *   FORCE_UP     | FAULT_REQUESTED             | FAULT
 *   FAULT        | (any)                       | FAULT (terminal)
 *
 * All other (state, event) pairs leave the state unchanged.
 *
 * FAULT is terminal at the FSM level. T_PID forces heater=0/fan=0.
 * Recovery requires power cycle (reboot).
 */

#include "app/fsm.h"

fsm_state_t fsm_next(fsm_state_t current, fsm_event_t event)
{
    /* FAULT is terminal — never transitions out via FSM events. The only
     * way out is hardware reset / power cycle, which restarts at boot. */
    if (current == FSM_FAULT) {
        return FSM_FAULT;
    }

    /* Fault request is the highest-priority transition from any non-FAULT
     * state. Checking it first means a sensor error during a normal
     * trigger event still results in FAULT, never the trigger transition. */
    if (event == FSM_EVT_FAULT_REQUESTED) {
        return FSM_FAULT;
    }

    switch (current) {
    case FSM_FORCE_DOWN:
        if (event == FSM_EVT_TRIGGER_FORCE_UP) {
            return FSM_FORCE_UP;
        }
        return FSM_FORCE_DOWN;

    case FSM_FORCE_UP:
        if (event == FSM_EVT_TRIGGER_FORCE_DOWN ||
            event == FSM_EVT_SAFETY_TIMEOUT) {
            return FSM_FORCE_DOWN;
        }
        return FSM_FORCE_UP;

    case FSM_FAULT:
    case FSM_STATE_COUNT:
    default:
        /* Unreachable: FAULT handled above, STATE_COUNT is a sentinel. */
        return current;
    }
}

const char *fsm_state_name(fsm_state_t state)
{
    switch (state) {
    case FSM_FORCE_DOWN: return "FORCE_DOWN";
    case FSM_FORCE_UP:   return "FORCE_UP";
    case FSM_FAULT:      return "FAULT";
    case FSM_STATE_COUNT:
    default:             return "?";
    }
}

const char *fsm_event_name(fsm_event_t event)
{
    switch (event) {
    case FSM_EVT_NONE:                return "NONE";
    case FSM_EVT_TRIGGER_FORCE_UP:    return "TRIG_UP";
    case FSM_EVT_TRIGGER_FORCE_DOWN:  return "TRIG_DOWN";
    case FSM_EVT_FAULT_REQUESTED:     return "FAULT_REQ";
    case FSM_EVT_SAFETY_TIMEOUT:      return "SAFETY_TO";
    case FSM_EVT_COUNT:
    default:                          return "?";
    }
}

/* fsm_output() is implemented in state_table.c so the LUT lives next to
 * the data definitions and not the transition logic. */

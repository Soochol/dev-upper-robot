/**
 * @file    state_table.c
 * @brief   FSM state → output configuration lookup table.
 *
 * One row per state, indexed by the fsm_state_t enum value. Adding a new
 * state requires adding a new row here AND updating fsm.c transitions.
 * The _Static_assert at the bottom catches the row-count mismatch at
 * compile time.
 *
 * Tunables come from config.h so all numbers live in one place.
 */

#include "app/fsm.h"
#include "app/config.h"

/* LED pattern identifiers. Defined here in Phase 2; will move to a future
 * actuators.h when Phase 6 implements the WS2812 driver. The numeric values
 * stay stable so state_table entries below need no update at that time. */
typedef enum {
    LED_OFF          = 0,
    LED_FADE_YELLOW  = 1,   /* FORCE_DOWN: cool-down indicator */
    LED_RAMP_YELLOW  = 2,   /* FORCE_UP: heating indicator */
    LED_FLASH_RED    = 3,   /* FAULT: alarm */
} led_pattern_t;

/* The output table. Order MUST match fsm_state_t in fsm.h.
 *
 * pid_enabled = false ONLY in FAULT — t_pid forces heater=0/fan=0 there.
 * In FORCE_UP and FORCE_DOWN the PID actively follows setpoint_c.
 *
 * safety_max_ms = 0 means "no timeout, may stay forever". Per D10:
 *   - FORCE_DOWN has no timeout (boot-time idle, may wait for trigger)
 *   - FORCE_UP has 60 s safety net (heater-on state, must not be forgotten)
 *   - FAULT is terminal (no point in counting)
 */
static const fsm_output_t g_state_table[FSM_STATE_COUNT] = {
    [FSM_FORCE_DOWN] = {
        .setpoint_c    = TEMP_COOL_C,
        .fan_duty_pct  = FAN_DUTY_FORCE_DOWN,
        .led_pattern   = LED_FADE_YELLOW,
        .pid_enabled   = true,
        .safety_max_ms = 0,                       /* no timeout */
    },
    [FSM_FORCE_UP] = {
        .setpoint_c    = TEMP_ACTIVE_C,
        .fan_duty_pct  = FAN_DUTY_FORCE_UP,
        .led_pattern   = LED_RAMP_YELLOW,
        .pid_enabled   = true,
        .safety_max_ms = SAFETY_MAX_FORCE_UP_MS,  /* 60 s */
    },
    [FSM_FAULT] = {
        .setpoint_c    = 0,
        .fan_duty_pct  = FAN_DUTY_FAULT,
        .led_pattern   = LED_FLASH_RED,
        .pid_enabled   = false,
        .safety_max_ms = 0,                       /* terminal */
    },
};

/* Compile-time guard: if a new state is added to fsm_state_t and the
 * developer forgets to add a row above, this fires immediately at build
 * time instead of returning an out-of-bounds pointer at runtime. */
_Static_assert(
    sizeof(g_state_table) / sizeof(g_state_table[0]) == FSM_STATE_COUNT,
    "state_table row count must match FSM_STATE_COUNT in fsm.h"
);

const fsm_output_t *fsm_output(fsm_state_t state)
{
    if ((unsigned)state >= FSM_STATE_COUNT) {
        /* Defensive: return FORCE_DOWN row (the safest output) for any
         * out-of-range value. Should never happen because g_fsm_state is
         * only ever assigned via fsm_next() which is type-checked. */
        return &g_state_table[FSM_FORCE_DOWN];
    }
    return &g_state_table[state];
}

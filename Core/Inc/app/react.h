/**
 * @file    react.h
 * @brief   Reaction-time tracker — measures sensor onset → FU dispatch latency.
 *
 * Single-purpose state machine that observes T_ML's per-cycle inputs and
 * emits the elapsed time between "user moved" (gyro magnitude crossing OR
 * FSR press) and "trigger published FORCE_UP." Purely observational — no
 * effect on trigger/FSM behavior.
 *
 * Lifecycle inside a FORCE_DOWN cycle:
 *
 *   onset_tick=0 ──[gyro|fsr crossed]──> armed ──[FU fired]──> measured
 *                                          │
 *                                          ├──[REACT_ONSET_TIMEOUT_MS]──> reset
 *                                          └──[FU→FD edge later]──────> reset
 *
 * The caller owns the tracker_t struct (typically file-static in t_ml.c)
 * so multiple instances are possible for tests. No file-global state.
 *
 * Side-effect free: caller is responsible for logging the returned value.
 */

#ifndef APP_REACT_H
#define APP_REACT_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "app/fsm.h"
#include "app/trigger.h"   /* sensor_snapshot_t */

typedef enum {
    REACT_SRC_NONE = 0,
    REACT_SRC_GYRO = 1,    /* armed by gyro magnitude > REACT_GYRO_ONSET_DPS */
    REACT_SRC_FSR  = 2,    /* armed by fsr_raw > FSR_THRESHOLD_DOWN_RAW */
} react_source_t;

typedef struct {
    TickType_t     onset_tick;     /* 0 = not armed */
    fsm_state_t    prev_state;     /* for FU→FD edge detection */
    react_source_t source;         /* what armed the current measurement */
    uint32_t       last_react_ms;  /* last successful measurement; 0 = never */
} react_tracker_t;

void react_tracker_init(react_tracker_t *t);

/* Update the tracker with one cycle of inputs.
 *
 * Returns react_ms (>0) on the cycle FU fires while armed.
 * Returns 0 on every other cycle (still arming, still armed, timed out,
 * or FU fired without prior arm).
 *
 * fu_fired = true iff the trigger module just published a FORCE_UP event
 * this cycle. The caller decides what counts as "FU fired" — this module
 * deliberately doesn't depend on trigger.h's event enum.
 */
uint32_t react_update(react_tracker_t *t,
                      fsm_state_t cur_state,
                      const sensor_snapshot_t *snap,
                      TickType_t tick,
                      bool fu_fired);

#endif /* APP_REACT_H */

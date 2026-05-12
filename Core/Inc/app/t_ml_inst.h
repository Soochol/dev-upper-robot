/**
 * @file    t_ml_inst.h
 * @brief   T_ML instrumentation surface — what T_ML exposes to T_WDG/GDB.
 *
 * These are NOT inter-task control state (queues/mutexes go in ipc.h).
 * They are write-by-T_ML, read-by-T_WDG-or-GDB debug observables. Defined
 * in t_ml.c so T_ML owns its own diagnostic surface.
 *
 * Single writer (T_ML), single non-blocking reader (T_WDG) — no
 * synchronization needed for these atomic-on-Cortex-M3 word/byte
 * accesses. Volatile prevents the compiler from caching the writes.
 */

#ifndef APP_T_ML_INST_H
#define APP_T_ML_INST_H

#include <stdint.h>

/* Canary counter — incremented at the end of every T_ML loop iteration.
 * T_WDG samples this every 100 ms; if it stops advancing for
 * CANARY_MAX_MISS samples the task is presumed stalled and IWDG fires. */
extern volatile uint32_t g_canary_ml;

/* Current phase inside the T_ML loop. Updated at major boundaries
 * (mutex take, I2C read, feature compute, trigger eval, heartbeat).
 * T_WDG dumps this on STALL so the post-mortem shows which line a
 * frozen task was executing. Values defined as anonymous enum in t_ml.c. */
extern volatile uint8_t  g_phase_ml;

/* Wall-clock duration of the last completed T_ML iteration, in ms.
 * 0 = no cycle completed yet. Lets T_WDG catch gradual slowdowns
 * (steady-state should be a few ms) before they escalate to full stalls. */
extern volatile uint32_t g_cycle_ms_ml;

#endif /* APP_T_ML_INST_H */

/**
 * @file    t_pid_inst.h
 * @brief   T_PID instrumentation surface — what T_PID exposes to T_WDG/GDB.
 *
 * See t_ml_inst.h header comment for the writer/reader contract. T_PID
 * additionally exposes a per-phase wall-clock max array so T_WDG can
 * pinpoint which step inside T_PID's loop is consuming the cycle budget.
 */

#ifndef APP_T_PID_INST_H
#define APP_T_PID_INST_H

#include <stdint.h>

extern volatile uint32_t g_canary_pid;
extern volatile uint8_t  g_phase_pid;
extern volatile uint32_t g_cycle_ms_pid;

/* Per-phase wall-clock max duration (ms) for T_PID, indexed by g_phase_pid.
 * Updated by t_pid.c on every phase transition; read+reset by T_WDG at 1 Hz.
 * Includes preemption time from higher-priority tasks — a phase whose
 * value spikes is where the wall-clock is being lost. */
#define PID_PHASE_COUNT 10
extern volatile uint32_t g_pid_phase_ms[PID_PHASE_COUNT];

#endif /* APP_T_PID_INST_H */

/**
 * @file    t_state_inst.h
 * @brief   T_STATE instrumentation surface — what T_STATE exposes to T_WDG/GDB.
 *
 * See t_ml_inst.h header comment for the writer/reader contract.
 */

#ifndef APP_T_STATE_INST_H
#define APP_T_STATE_INST_H

#include <stdint.h>

extern volatile uint32_t g_canary_state;
extern volatile uint8_t  g_phase_state;
extern volatile uint32_t g_cycle_ms_state;

/* FU dispatch latency — written by T_STATE on every FORCE_UP transition.
 * GDB-readable backup of the [trigger] sensor captured / forceup
 * implemented RTT lines. Survives RTT buffer drops and host disconnects.
 *
 * Usage (GDB):
 *   (gdb) print g_fu_count       — total FU events since boot (liveness)
 *   (gdb) print g_fu_lat_ms_last — last measured dispatch latency (ms)
 *   (gdb) print g_fu_send_ms_last / g_fu_recv_ms_last — endpoint timestamps
 *
 * All zero = no FU has fired yet. g_fu_count not advancing = no new FU. */
extern volatile uint32_t g_fu_count;          /* total FU transitions */
extern volatile uint32_t g_fu_lat_ms_last;    /* last fu_lat_ms measurement */
extern volatile uint32_t g_fu_send_ms_last;   /* T_ML send tick (ms) */
extern volatile uint32_t g_fu_recv_ms_last;   /* T_STATE recv tick (ms) */

#endif /* APP_T_STATE_INST_H */

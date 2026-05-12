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

#endif /* APP_T_STATE_INST_H */

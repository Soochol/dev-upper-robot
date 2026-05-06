/**
 * @file    trigger_timer.h
 * @brief   Time-based test cycle trigger provider — FU/FD infinite loop.
 *
 * Used for hardware burn-in, battery life measurement, and actuator
 * response characterization. Cycles FORCE_UP for TEST_FU_DURATION_MS
 * then FORCE_DOWN for TEST_FD_DURATION_MS, ignoring sensor input.
 *
 * Activated by setting TRIGGER_SOURCE = TRIG_SRC_TIMER (config.h),
 * normally via the Debug-Cycle CMake preset (TEST_CYCLE=ON).
 *
 * FSM, PID, safety timeout (60s), overtemp (100°C), and IWDG remain
 * active — this provider only replaces the trigger eval function.
 */

#ifndef APP_TRIGGER_TIMER_H
#define APP_TRIGGER_TIMER_H

#include "app/trigger.h"

extern const trigger_provider_t trig_timer;

#endif /* APP_TRIGGER_TIMER_H */

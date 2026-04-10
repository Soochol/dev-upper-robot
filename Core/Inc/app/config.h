/**
 * @file    config.h
 * @brief   Application-wide tunable constants for dev-upper-robot.
 *
 * Single source of truth for everything that might need re-tuning during
 * bring-up: task priorities/stacks/periods, FSM setpoints, fan duty LUT,
 * safety timeouts, trigger thresholds, PID gains.
 *
 * Keep this header dependency-free. It must be includable from any task or
 * driver file without pulling in HAL or FreeRTOS types.
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ========================================================================
 * Task scheduling (priorities, stack sizes in words, periods in ms)
 * ======================================================================== */

/* Priorities — higher number = higher priority. Must stay below
 * configMAX_PRIORITIES (= 7 in FreeRTOSConfig.h). T_STATE is highest because
 * it owns the FSM and safety timeout. T_PID is next so the control loop
 * preempts ML work. T_ML is below the control loop. T_LOGGER is idle-class. */
#define PRIO_T_STATE        5
#define PRIO_T_PID          4
#define PRIO_T_ML           3
#define PRIO_T_LOGGER       1
#define PRIO_DEFAULT        1

/* Stack sizes (words, 1 word = 4 bytes on Cortex-M3). configMINIMAL_STACK_SIZE
 * is 128 words. Stack overflow detection is on (configCHECK_FOR_STACK_OVERFLOW
 * = 2) so over-allocation is the safe default during bring-up; we shrink
 * after measuring uxTaskGetStackHighWaterMark in Phase 6. */
#define STK_T_STATE_WORDS   (384 / 4)
#define STK_T_PID_WORDS     (512 / 4)
#define STK_T_ML_WORDS      (640 / 4)
#define STK_T_LOGGER_WORDS  (512 / 4)

/* Task periods (ms). All application tasks run at 20 Hz. The 25 ms offset on
 * T_ML interleaves I2C1 access with T_PID so the PI mutex is almost never
 * contested in normal operation. See plan A1 for the analysis. */
#define PERIOD_T_STATE_MS   50
#define PERIOD_T_PID_MS     50
#define PERIOD_T_ML_MS      50
#define OFFSET_T_ML_MS      25

/* defaultTask heartbeat period (ms). Used by the CubeMX-generated task that
 * we keep as a watchdog/health beacon. */
#define PERIOD_HEARTBEAT_MS 1000

/* ========================================================================
 * FSM setpoints and safety
 * ======================================================================== */

/* Temperature setpoints (degrees Celsius) */
#define TEMP_ACTIVE_C           60   /* FORCE_UP target */
#define TEMP_COOL_C             32   /* FORCE_DOWN target */
#define OVERTEMP_HARD_C        100   /* FAULT trigger threshold */

/* Fan duty cycle (%) — FSM state lookup table. fan is NOT a PID output;
 * it is a hard-coded value per state. PID controls heater only. */
#define FAN_DUTY_FORCE_UP       0
#define FAN_DUTY_FORCE_DOWN   100
#define FAN_DUTY_FAULT          0

/* Safety net timeout (ms). FORCE_UP only — heater-on state must not be
 * forgotten by the trigger provider. FORCE_DOWN has no timeout (it is
 * the boot-time idle state and may wait for a trigger forever). On
 * timeout the FSM forces the transition FORCE_UP → FORCE_DOWN. */
#define SAFETY_MAX_FORCE_UP_MS      60000   /* 60 s */

/* Command watchdog inside T_PID: if no q_ctrl_to_pid message arrives for
 * this long, force heater to 0 (T_STATE has likely died). */
#define PID_CMD_WATCHDOG_MS         2000

/* Phase 2 auto-trigger emulator. Phase 2 has no real trigger source yet
 * (T_ML is still a stub through Phase 4). To exercise the FSM during
 * bring-up, defaultTask emits TRIGGER_FORCE_UP / TRIGGER_FORCE_DOWN
 * alternately every PHASE2_AUTO_TRIGGER_PERIOD_MS milliseconds. Set the
 * macro to 0 in Phase 4 once T_ML starts publishing real triggers. */
#define PHASE2_AUTO_TRIGGER             1
#define PHASE2_AUTO_TRIGGER_PERIOD_MS   10000   /* 10 s alternation */

/* ========================================================================
 * Trigger provider selection (compile-time, v1)
 * ======================================================================== */

#define TRIG_SRC_RULE  0
#define TRIG_SRC_ML    1

/* Default provider for v1. Switch by changing this single macro and
 * rebuilding. Runtime switching is deferred to Phase 4+. */
#ifndef TRIGGER_SOURCE
#define TRIGGER_SOURCE  TRIG_SRC_RULE
#endif

/* Rule-based trigger thresholds. PLACEHOLDER values — these MUST be
 * recalibrated against real hardware once FSR/IMU drivers are wired up
 * in Phase 4. Do not trust the numerical values below for production. */
#define FSR_THRESHOLD_UP_ADC      2000   /* TODO: calibrate */
#define FSR_THRESHOLD_DOWN_ADC     500   /* TODO: calibrate */
#define IMU_TILT_UP_DEG             45   /* TODO: calibrate */
#define IMU_TILT_DOWN_DEG           10   /* TODO: calibrate */

/* ========================================================================
 * PID gains (PLACEHOLDER, Phase 6 tuning)
 * ======================================================================== */

#define PID_KP            2.0f
#define PID_KI            0.1f
#define PID_KD            0.5f

/* PID output range. TIM1 CH1 ARR is 1000 (see Core/Src/tim.c MX_TIM1_Init),
 * so a duty of 0..1000 maps directly to TIM1->CCR1 register values. */
#define PID_OUTPUT_MIN       0
#define PID_OUTPUT_MAX    1000

/* PID sample period in seconds, derived from T_PID period. Used in the
 * integral and derivative terms. */
#define PID_DT_S          (PERIOD_T_PID_MS / 1000.0f)

/* ========================================================================
 * Queue sizes (number of items)
 * ======================================================================== */

#define Q_LOG_DEPTH               16
#define Q_CTRL_TO_PID_DEPTH        4
#define Q_TRIGGER_TO_STATE_DEPTH   4
#define Q_FAULT_REQ_DEPTH          2

#endif /* APP_CONFIG_H */

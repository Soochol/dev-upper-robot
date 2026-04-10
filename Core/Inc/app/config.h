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

/* Phase 2 auto-trigger emulator. Was on through Phase 3 to exercise the
 * FSM without a real sensor source. Disabled in Phase 4 because T_ML now
 * publishes real triggers from the FSR — having both sources active would
 * race for q_trigger_to_state. Re-enable temporarily if you need to test
 * the FSM in isolation again. */
#define PHASE2_AUTO_TRIGGER             0
#define PHASE2_AUTO_TRIGGER_PERIOD_MS   10000   /* unused while disabled */

/* ========================================================================
 * IR temperature sensors (Diwell TBP-H70)
 * ========================================================================
 * SMBus variant: read sequence is [S addr+W cmd Sr addr+R lo hi pec P]
 * which is exactly the shape of HAL_I2C_Mem_Read with MEMADD_SIZE_8BIT
 * and a 3-byte read. The PEC byte (CRC-8 polynomial 0x07) is read as
 * the third data byte; v1 driver does NOT verify PEC and treats the
 * value as opaque — Phase 6 may add verification with a lookup table.
 *
 * Two sensors share I2C1. The default address is 0x3A; one sensor must
 * be reprogrammed to a different address (0x4C in this hardware build,
 * see vendor protocol section 4.3.1). Bring-up uses an I2C scanner at
 * sensors_i2c_init() to verify the live addresses match these constants. */

/* 7-bit slave addresses. Driver functions shift left by 1 internally
 * before passing to HAL_I2C_Mem_Read. Keep the values as the data sheet
 * documents them so the constants match the vendor diagrams 1:1. */
#define IR_SENSOR_1_I2C_ADDR_7B     0x3A    /* default, unmodified */
#define IR_SENSOR_2_I2C_ADDR_7B     0x4C    /* reprogrammed via 4.3.1 procedure */

/* TBP-H70 register/command codes (vendor protocol section 4.2). */
#define TBP_CMD_SENSOR_TEMP         0x06    /* ambient temperature */
#define TBP_CMD_TARGET_TEMP         0x07    /* IR target temperature ← PID input */
#define TBP_CMD_EMISSIVITY          0x24    /* emissivity setting */

/* Timing requirements from the protocol document. */
#define TBP_POWER_ON_DELAY_MS       200     /* min wait after VCC before first command */
#define TBP_READ_PERIOD_MIN_MS      100     /* internal update is 10 Hz */
#define TBP_HAL_TIMEOUT_MS          10      /* per HAL_I2C_Mem_Read call */

/* Bit 15 of the 16-bit raw value is an error flag — if set, the reading
 * is invalid and must be discarded. Bits 0..14 hold the data. */
#define TBP_ERROR_FLAG_MASK         0x8000u
#define TBP_DATA_MASK               0x7FFFu

/* Conversion: raw × 0.02 = absolute temperature in Kelvin.
 *             celsius = kelvin - 273.15
 * The driver returns float Celsius; PID input is float as well. */
#define TBP_RAW_TO_KELVIN_SCALE     0.02f
#define KELVIN_TO_CELSIUS_OFFSET    273.15f

/* I2C scanner range — used at sensors_i2c_init to log every device that
 * ACKs on the bus. Standard 7-bit I2C address space is 0x08..0x77 (the
 * lower and upper ranges are reserved). */
#define I2C_SCAN_ADDR_MIN           0x08
#define I2C_SCAN_ADDR_MAX           0x77

/* ========================================================================
 * ICM42670P 6-axis IMU (TDK InvenSense)
 * ========================================================================
 * I2C device at 0x69 (AP_AD0 pulled high). 6 data registers each for accel
 * and gyro, accessible as a 12-byte burst read starting at ACCEL_DATA_X1.
 *
 * Bring-up sequence (minimal v1):
 *   1. Read WHO_AM_I (0x75), expect 0x67. Sanity check.
 *   2. Write PWR_MGMT0 (0x1F) to enable accel + gyro in low-noise mode.
 *   3. Wait ~10 ms for the ramp-up.
 *   4. From then on, burst-read 12 bytes from ACCEL_DATA_X1 (0x0B). */

#define IMU_I2C_ADDR_7B             0x69
#define ICM42670P_REG_WHO_AM_I      0x75
#define ICM42670P_WHO_AM_I_VALUE    0x67
#define ICM42670P_REG_PWR_MGMT0     0x1F
/* PWR_MGMT0 value: ACCEL_LP_CLK_SEL=0, IDLE=0, GYRO_MODE=11 (Low Noise),
 * ACCEL_MODE=11 (Low Noise). All other bits 0. */
#define ICM42670P_PWR_MGMT0_ON      0x0F
#define ICM42670P_REG_ACCEL_DATA_X1 0x0B  /* burst read start: 12 bytes */
#define ICM42670P_DATA_BURST_LEN    12
#define ICM42670P_STARTUP_DELAY_MS  10

/* ========================================================================
 * ADS1115 16-bit ADC (Texas Instruments) — used as the FSR front-end
 * ========================================================================
 * I2C device at 0x49 (ADDR pin tied to VDD). Single-channel continuous
 * mode on AIN0 single-ended.
 *
 * Config register (0x01) value 0x4483 breakdown:
 *   bit 15    OS = 0       (single-shot bit, ignored in continuous mode)
 *   bit 14:12 MUX = 100    AIN0 single-ended vs GND
 *   bit 11:9  PGA = 010    +/- 2.048 V full-scale
 *   bit 8     MODE = 0     continuous conversion
 *   bit 7:5   DR = 100     128 SPS
 *   bit 4     COMP_MODE=0  traditional comparator (unused)
 *   bit 3     COMP_POL=0   active low comparator (unused)
 *   bit 2     COMP_LAT=0   non-latching (unused)
 *   bit 1:0   COMP_QUE=11  disable comparator
 * Composed: 0100_010_0_100_00011 = 0x4483 */

#define ADS1115_I2C_ADDR_7B         0x49
#define ADS1115_REG_CONVERSION      0x00
#define ADS1115_REG_CONFIG          0x01
#define ADS1115_CONFIG_AIN0_CONT    0x4483u

/* ========================================================================
 * Trigger thresholds (FSR + IMU)
 * ========================================================================
 * Phase 4 v1: rule-based trigger uses FSR only with hysteresis. The two
 * thresholds form a dead zone — values between them leave the FSM state
 * unchanged. Without hysteresis, FSR jitter near a single threshold would
 * fire trigger events every cycle and the FSM would oscillate.
 *
 * IMU thresholds are placeholders for the Phase 4b extension when tilt
 * becomes part of the trigger logic. trigger_rule.c does not read them yet.
 *
 * All values are PLACEHOLDER and must be calibrated against real hardware
 * once the FSR contact surface is mounted. See TODO.md "Trigger threshold
 * calibration" for the procedure. */

#define FSR_THRESHOLD_UP_RAW        2000   /* TODO: calibrate (raw signed ADC) */
#define FSR_THRESHOLD_DOWN_RAW       500   /* TODO: calibrate */
#define IMU_TILT_UP_DEG               45   /* TODO: calibrate, Phase 4b */
#define IMU_TILT_DOWN_DEG             10   /* TODO: calibrate, Phase 4b */

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

/* (Old Phase 2 placeholders removed — now defined in the detailed
 * "Trigger thresholds" section below the ADS1115 block, with proper
 * hysteresis semantics and calibration notes.) */

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

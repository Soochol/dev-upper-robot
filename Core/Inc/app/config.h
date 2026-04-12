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
#define STK_T_ML_WORDS      (4096 / 4)   /* 4096: extra headroom for score() soft-float */
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
#define TEMP_COOL_C             25   /* FORCE_DOWN target (test: fan off) */
#define OVERTEMP_HARD_C         80   /* heater element absolute max (FAULT) */
#define TEMP_DEADBAND_C          2   /* ±2°C: within this range = target reached */

/* Fan duty cycle (%) — state table default. t_pid.c overrides based on
 * temperature vs deadband. PWM is inverted in HW (CCR=0 → full speed). */
#define FAN_DUTY_FORCE_UP       0
#define FAN_DUTY_FORCE_DOWN     0
#define FAN_DUTY_FAULT          0

/* Active cooling duty (%) when temperature is above deadband during
 * FORCE_DOWN. Distinct from FAN_DUTY_FORCE_DOWN which is the state
 * table default — this override kicks in only while actively cooling. */
#define FAN_DUTY_COOLDOWN_PCT   50

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

/* ICM42670P register addresses (BANK0, direct I2C access). */
#define ICM42670P_REG_SIGNAL_PATH_RESET  0x02
#define ICM42670P_REG_DRIVE_CONFIG2      0x04
#define ICM42670P_REG_INT_CONFIG         0x06
#define ICM42670P_REG_PWR_MGMT0         0x1F
#define ICM42670P_REG_GYRO_CONFIG0      0x20
#define ICM42670P_REG_ACCEL_CONFIG0     0x21
#define ICM42670P_REG_GYRO_CONFIG1      0x23
#define ICM42670P_REG_ACCEL_CONFIG1     0x24
#define ICM42670P_REG_INT_SOURCE0       0x2B
#define ICM42670P_REG_INT_STATUS        0x3A
#define ICM42670P_REG_INTF_CONFIG1      0x36
#define ICM42670P_REG_WHO_AM_I          0x75
#define ICM42670P_REG_BLK_SEL_R        0x7C
#define ICM42670P_REG_BLK_SEL_W        0x79
#define ICM42670P_REG_ACCEL_DATA_X1    0x0B  /* burst read: 12 bytes */
#define ICM42670P_DATA_BURST_LEN       12
#define ICM42670P_WHO_AM_I_VALUE       0x67

/* Init register values (from FSR/IMU hardware reference guide). */
#define ICM42670P_DRIVE_CONFIG2_VAL    0x09  /* I2C drive strength */
#define ICM42670P_SOFT_RESET_BIT       0x10  /* SIGNAL_PATH_RESET bit 4 */
/* ACCEL_CONFIG0 (0x21) bit layout:
 *   [7]   reserved = 0
 *   [6:5] ACCEL_UI_FS_SEL: 00=±16g, 01=±8g, 10=±4g, 11=±2g
 *   [4]   reserved = 0
 *   [3:0] ACCEL_ODR: 0101=400Hz, 0110=200Hz, 0111=100Hz, 1000=50Hz, 1001=25Hz
 * ±2g (11) + 100Hz (0111) = 0110_0111 = 0x67 */
#define ICM42670P_ACCEL_CONFIG0_VAL    0x67
/* ACCEL_CONFIG1: ACCEL_UI_FILT_BW=101 (25Hz) → bits[2:0] = 5, rest 0 → 0x05 */
#define ICM42670P_ACCEL_CONFIG1_VAL    0x05
/* GYRO_CONFIG0 (0x20) bit layout:
 *   [6:5] GYRO_UI_FS_SEL: 00=±2000dps, 01=±1000, 10=±500, 11=±250
 *   [3:0] GYRO_ODR: same encoding as accel
 * ±2000dps (00) + 100Hz (0111) = 0000_0111 = 0x07 */
#define ICM42670P_GYRO_CONFIG0_VAL     0x07
/* GYRO_CONFIG1: GYRO_UI_FILT_BW=011 (73Hz) → bits[2:0] = 3, rest 0 → 0x03 */
#define ICM42670P_GYRO_CONFIG1_VAL     0x03
/* PWR_MGMT0: ACCEL=LN(11), GYRO=LN(11) → 0x0F */
#define ICM42670P_PWR_MGMT0_ON        0x0F

/* Timing */
#define ICM42670P_RESET_DELAY_MS       100  /* after soft reset */
#define ICM42670P_STARTUP_DELAY_MS      50  /* after PWR_MGMT0 enable */

/* Conversion: ±2g full-scale, 16384 LSB/g */
#define IMU_ACCEL_SCALE_G   (2.0f / 32768.0f)
/* ±2000 dps full-scale */
#define IMU_GYRO_SCALE_DPS  (2000.0f / 32768.0f)

/* ========================================================================
 * ADS1115 16-bit ADC (Texas Instruments) — used as the FSR front-end
 * ========================================================================
 * I2C device at 0x49 (ADDR pin tied to VDD). Single-shot mode on AIN0
 * single-ended. Config value 0xC283 matches the FSR hardware reference.
 *
 * Config register (0x01) value 0xC283 breakdown:
 *   bit 15    OS = 1       start single-shot conversion
 *   bit 14:12 MUX = 100    AIN0 single-ended vs GND
 *   bit 11:9  PGA = 001    +/- 4.096 V full-scale
 *   bit 8     MODE = 0     single-shot (power-down after conversion)
 *   bit 7:5   DR = 100     128 SPS
 *   bit 4     COMP_MODE=0  traditional comparator (unused)
 *   bit 3     COMP_POL=0   active low
 *   bit 2     COMP_LAT=0   non-latching
 *   bit 1:0   COMP_QUE=11  disable comparator
 * Composed: 1100_0010_1000_0011 = 0xC283
 *
 * Read pattern: each ads1115_read() writes config (OS=1 triggers new
 * conversion) then immediately reads the conversion register (returns
 * previous result). First read after init is stale; second onward is
 * 50 ms old (T_ML period). */

#define ADS1115_I2C_ADDR_7B         0x49
#define ADS1115_REG_CONVERSION      0x00
#define ADS1115_REG_CONFIG          0x01
#define ADS1115_CONFIG_SS           0xC283u

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

/* Trigger debounce: after a trigger fires, ignore further trigger events
 * for this many milliseconds. Prevents FSR jitter near the threshold from
 * causing rapid FORCE_UP ↔ FORCE_DOWN oscillation. 500 ms is a safe
 * starting point — increase if bounce persists, decrease if the system
 * feels sluggish responding to intentional state changes. */
#define TRIGGER_DEBOUNCE_MS         500

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

/* Setpoint weighting: P term uses Kp * (b * sp - meas).
 * b = 1.0 is standard PID, b = 0.7 reduces overshoot on setpoint steps
 * while preserving full disturbance rejection. */
#define PID_SETPOINT_WEIGHT_B   0.7f

/* Derivative filter coefficient N = Td / Tf. Higher N = less filtering.
 * N = 10 limits D-term bandwidth to ~10x the P-term bandwidth. */
#define PID_DERIV_FILTER_N     10.0f

/* PID output range. TIM1 CH1 ARR is 1000 (see Core/Src/tim.c MX_TIM1_Init),
 * so a duty of 0..1000 maps directly to TIM1->CCR1 register values. */
#define PID_OUTPUT_MIN       0
#define PID_OUTPUT_MAX    1000

/* PID sample period in seconds, derived from T_PID period. Used in the
 * integral and derivative terms. */
#define PID_DT_S          (PERIOD_T_PID_MS / 1000.0f)

/* Heater output slew-rate limit: max delta-duty per 50 ms cycle.
 * 50 out of 1000 per cycle → 0 to 100 % in 1.0 s. */
#define PID_SLEW_LIMIT_PER_CYCLE  50

/* ========================================================================
 * Queue sizes (number of items)
 * ======================================================================== */

#define Q_LOG_DEPTH               16
#define Q_CTRL_TO_PID_DEPTH        4
#define Q_TRIGGER_TO_STATE_DEPTH   4
#define Q_FAULT_REQ_DEPTH          4   /* T_PID + T_ML concurrent fault [C3] */

/* ========================================================================
 * ML feature extraction
 * ========================================================================
 * Sliding window size for temporal features (mean, slope, variance).
 * Start with 20 (= 1 second at 20 Hz). Tune via Python training pipeline
 * by comparing accuracy with 10, 20, 30. Rebuild MCU after changing. */

#define ML_WINDOW_SIZE          20

/* ========================================================================
 * Data collection mode (SD card CSV logging)
 * ========================================================================
 * When enabled, T_ML writes raw sensor values to SD card in CSV format
 * for offline ML training. Disable for normal operation (inference only).
 *
 * SD write buffer is 512 bytes (one SDIO block). At 20 Hz with ~60
 * bytes per CSV row, the buffer flushes roughly every 0.4 seconds.
 * Each flush costs ~2 ms (DMA), negligible against the 50 ms tick. */

#ifndef DATA_COLLECT_MODE
#define DATA_COLLECT_MODE       1
#endif

/* SD logging write buffer size. Must be a multiple of the SDIO block
 * size (512) for aligned writes. 2048 = 4 blocks; at 20 Hz / ~45 B per
 * row this flushes roughly every 2 seconds, reducing SDIO traffic vs
 * the previous 512 B (flush every 0.5 s) that triggered write errors. */
#define SD_LOG_BUF_SIZE         2048

/* ========================================================================
 * IWDG + T_WDG watchdog
 * ========================================================================
 * T_WDG is a dedicated watchdog task. Each application task increments a
 * canary counter at the end of its main loop. T_WDG checks all canaries
 * every PERIOD_T_WDG_MS. If all are alive, it kicks the IWDG. If any
 * canary stalls for CANARY_MAX_MISS consecutive checks, T_WDG stops
 * kicking → IWDG resets the MCU after ~400 ms.
 *
 * IWDG config: LSI 40 kHz, prescaler 64, reload 250 → ~400 ms timeout.
 * T_WDG initializes IWDG inside its own task body (not main.c) so the
 * countdown never starts before the kicker is ready [C1]. */

#define PRIO_T_WDG            6     /* [W10] above all monitored tasks */
#define STK_T_WDG_WORDS       (128)   /* configMINIMAL_STACK_SIZE */
#define PERIOD_T_WDG_MS       100
#define CANARY_MAX_MISS       3       /* 3 × 100 ms = 300 ms stall tolerance */
#define WDG_BOOT_GRACE_TICKS  20      /* 2 s unconditional kick at boot */

/* IWDG hardware parameters. LSI can drift 30–60 kHz; with these values
 * the timeout ranges from 267 ms (60 kHz) to 533 ms (30 kHz). */
#define IWDG_PRESCALER_VAL    IWDG_PRESCALER_64
#define IWDG_RELOAD_VAL       250

/* ========================================================================
 * Overtemp sustained-count threshold
 * ========================================================================
 * The existing `overtemp` boolean still kills heater output immediately
 * on the first sample above OVERTEMP_HARD_C. This counter only gates the
 * terminal FAULT transition so noise spikes don't trigger a power-cycle
 * requirement. [W7] */

#define OVERTEMP_SUSTAIN_COUNT  20    /* 20 × 50 ms = 1 s */

/* ========================================================================
 * T_ML sensor failure + I2C bus recovery
 * ======================================================================== */

#define ML_SENSOR_FAIL_LIMIT   40    /* 40 × 50 ms = 2 s */
#define I2C_MAX_RECOVERY        3    /* max bus recovery attempts per event */

#endif /* APP_CONFIG_H */

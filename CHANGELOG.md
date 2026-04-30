# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added
- IWDG reset post-mortem indicator: main.c reads RCC_CSR.IWDGRSTF and blinks GPIO LEDs PB0+PB1 for ~2 s before FreeRTOS starts, giving the user a clear visual signal that the previous reset was watchdog-driven
- `g_reset_cause` global (snapshot of RCC_CSR before RMVF clear) so tasks can branch on the original reset cause after flags are cleared
- `FAULT_REASON_WATCHDOG_RECOVERY` fault reason for logging IWDG-recovery boots
- Best-effort red GPIO LED set in T_WDG when canary stall is detected (covers the ~400 ms window before IWDG fires)
- Stall diagnostics: `g_phase_*` (current phase per task) and `g_cycle_ms_*` (last cycle duration) globals dumped by T_WDG on STALL and at 1 Hz. Pinpoints which code section a frozen task was executing
- Per-phase max wall-clock duration tracking for T_PID (`g_pid_phase_ms[]`), reset every 1 s. Caught the IR_READ phase eating the full T_PID period

### Changed
- T_STATE init now enters FSM_FAULT directly when boot follows an IWDG reset. Previously the system booted into FORCE_DOWN regardless, briefly rendering blue on the LED before FAULT was reached. Sensor-error FAULTs are unaffected (they arrive via q_fault_req after tasks are running)
- IWDG init is now exclusively performed inside T_WDG. Previously main.c also called MX_IWDG_Init() — the duplicate call was idempotent but contradicted the "single init site" intent in t_wdg.c
- `PRIO_T_ML` raised from 3 to 4 (equal to T_PID). Time-sliced round-robin scheduling now prevents T_PID from starving T_ML when its IR_READ phase overruns the 50 ms period. Without this fix, a single ~52 ms T_PID cycle was enough to keep T_ML's canary stuck for 300 ms → IWDG reset

### Removed
- Duplicate `MX_IWDG_Init()` call in main.c (along with the misleading "timeout ~4s" message — actual IWDG timeout is ~400 ms per LSI/prescaler/reload settings)

### Known Issues
- TBP-H70 IR sensor reads via `HAL_I2C_Mem_Read` occasionally take ~25 ms each (vs expected ~3 ms), likely due to slave clock-stretching combined with STM32F1 HAL's internal `I2C_TIMEOUT_FLAG` (~35 ms) overriding the user-supplied timeout. No longer triggers stalls thanks to priority equalization, but root cause investigation is deferred to a follow-up PR

## [0.1.0.0] - 2026-04-12

### Added
- IWDG hardware watchdog with dedicated T_WDG monitoring task (prio 6)
- Canary-based liveness detection for T_STATE, T_PID, T_ML tasks
- I2C bus recovery (SCL 9-clock toggle + HAL reinit) for all 4 sensors
- FSR/IMU sensor failure detection with FAULT escalation
- Overtemp sustained-count filter (1 second) to prevent noise-spike false FAULT

### Changed
- q_fault_req depth increased from 2 to 4 for concurrent fault producers
- T_STATE now drains fault queue completely (logs all reasons)
- Overtemp still kills heater immediately on first detection; only terminal FAULT transition is delayed

### Fixed
- FSR read result was silently ignored (`(void)` cast) causing false FORCE_DOWN triggers
- I2C STOP condition sequence corrected (SCL LOW before SDA transition)

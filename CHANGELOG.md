# Changelog

All notable changes to this project will be documented in this file.

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

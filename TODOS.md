# TODOS

## P1: FAULT recovery for sensor_timeout

**What:** FSM_FAULT에서 FAULT_REASON_SENSOR_TIMEOUT일 때 자동 복구 시도.

**Why:** 현재 I2C glitch (40 ticks = 2초) → 영구 FAULT → power cycle만 복구. 웨어러블 가전에서 사용자가 전원을 재투입해야 하는 UX는 좋지 않음.

**Pros:** 일시적 센서 장애에서 자동 복구, 사용자 중단 없는 세션 유지.

**Cons:** FSM에 FAULT_RECOVERING 상태 추가 → 전이 테이블 복잡도 증가, 안전 분석 재검토 필요.

**Context:** Outside voice (eng review 2026-04-15)에서 식별. overtemp은 영구 FAULT 유지해야 함 (하드웨어 보호). sensor_timeout만 복구 대상. 구현 시 N초 대기 → 센서 재초기화 → 3회 연속 성공 → FORCE_DOWN 복귀 패턴 권장.

**Depends on / blocked by:** IWDG 활성화 완료 (이 PR에서 해결됨).

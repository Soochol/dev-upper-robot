# TODO — dev-upper-robot

작업 추적용 living document. Phase별 deferred 항목, placeholder 상수,
미해결 결정 사항, 미래 polish 작업을 한곳에 모음.

이 파일과 `/home/dev/.claude/plans/sequential-popping-shell.md`의 차이:
- **Plan**은 현재 sprint의 phase별 큰 그림 — sprint 종료 시 archived
- **TODO.md**는 deferred 작업의 living tracker — 항목이 완료되면 ✓
  표시하거나 삭제, 새 항목은 발견 즉시 추가

---

## 🔧 Hardware integration

### Heater 회로 연결 (현재 미연결 — Phase 3b 검증 시 확인)

- **Status**: 회로에 미연결 (gate driver 없음 또는 MOSFET 미장착)
- **증거**: Phase 3b RTT 검증에서 heater duty가 22~123까지 변화하는 동안
  IR 온도가 ~21°C에서 변화 없음 → PWM은 출력되지만 발열체가 없음
- **PID 코드 자체는 검증 완료**: integrator 누적 속도가 이론치와 0.3% 오차로 일치
- **통합 후 검증 항목**:
  - [ ] PA8 (TIM1_CH1) 오실로스코프로 PWM duty 측정
  - [ ] heater duty 50% 시 IR 온도 상승 확인
  - [ ] FORCE_UP 진입 후 60°C에 도달하는 시간 측정
  - [ ] Over-temp 100°C 도달 시 FAULT 진입 + PC5 release 동작 확인
- **선결조건**: 안전한 열 절연 환경 + 과열 보호 (heater fuse 또는 열보호 스위치)

### Fan 회로 검증

- **Status**: TIM1_CH2 (PA9) PWM 출력은 코드상 동작 확인됨
- **검증 필요**:
  - [ ] PA9 PWM duty 측정
  - [ ] FORCE_DOWN(100%) / FORCE_UP(0%) 전환 시 fan 회전수 변화 확인

### LED 회로 검증

- **Status**: PB0 (R1), PB1 (R2) GPIO 동작 확인 필요
- **검증 필요**:
  - [ ] FORCE_DOWN: R1 ON / R2 OFF
  - [ ] FORCE_UP: R1 OFF / R2 ON
  - [ ] FAULT: R1 ON + R2 ON

---

## 🎛 PID tuning (Phase 6)

### Heater PID gains 실측 튜닝

**Current placeholder** (config.h): `Kp=2.0, Ki=0.1, Kd=0.5`

이 값들은 hardware bring-up 단계의 안전한 보수적 값. 실제 heater 연결 후
heating curve 측정이 필요함.

**튜닝 절차** (Ziegler-Nichols open-loop method):

1. **선결조건**:
   - Heater 회로 연결 완료
   - 안전 환경 (열 절연, 과열 보호)
   - IR 센서 정상 위치 (heater 가열면을 직접 측정)

2. **Open-loop step response 측정**:
   - 시스템을 cold state로 안정화 (IR ≈ 실내 온도)
   - PID disable 상태에서 heater duty = 50% (또는 25%) 강제
   - IR 온도를 1초마다 RTT로 기록
   - 60°C 도달 또는 5분 경과 시 heater off
   - Heating curve plot (시간 vs 온도)

3. **시스템 파라미터 추출** (heating curve에서):
   - **Process gain (K)**: 정상 상태 온도 변화 / duty 변화
   - **Time constant (τ)**: 정상 상태 변화의 63.2% 도달 시간
   - **Dead time (L)**: step 입력 후 첫 반응까지의 지연

4. **Ziegler-Nichols 초기 gain 추정**:

   ```
   Kp = 1.2 × τ / (K × L)
   Ki = Kp × 2 × L          (Ti = 2L → Ki = Kp/Ti)
   Kd = Kp × 0.5 × L        (Td = L/2 → Kd = Kp × Td)
   ```

   또는 Cohen-Coon (dead time이 큰 시스템에 더 적합):

   ```
   Kp = (1/K) × (τ/L) × (4/3 + L/(4τ))
   Ti = L × (32 + 6L/τ) / (13 + 8L/τ)
   Td = L × 4 / (11 + 2L/τ)
   ```

5. **Closed-loop fine tuning**:
   - 초기 gain 적용 후 setpoint step (32 → 60°C) 응답 측정
   - **목표 metric**:
     - Settling time: ≤ 60s (FORCE_UP safety timeout 안에 도달)
     - Overshoot: < 5% (안전, 60°C 목표 시 < 63°C 한계)
     - Steady-state error: < 0.5°C (sensor accuracy 한계)
   - 결과에 따라 Kp ±20%, Ki ±50%, Kd ±50% 조정 반복

6. **검증**:
   - 전체 FSM 사이클 (FORCE_DOWN ↔ FORCE_UP) 안정성 확인
   - bumpless transfer 동작 확인 (전이 시 overshoot 없음)
   - over-temp 안전 회로 별도 trigger 테스트

7. **확정값을 `Core/Inc/app/config.h`의 PID_KP/KI/KD에 반영 + commit**

### PID tuning 데이터 수집 도구 (별도 작업)

- [ ] Open-loop step response 측정용 임시 모드
  - `config.h`에 `PID_TUNING_MODE` 매크로 추가
  - 활성화 시 t_pid가 PID 우회하고 fixed duty 출력
  - 매 사이클 IR + heater_duty + tick을 RTT로 기록 (CSV 형태)
- [ ] 호스트 측 plotting 스크립트 (Python matplotlib)

---

## 🧠 Phase 4 — Sensors + ML triggers (다음 sprint)

### ICM42670P IMU driver

- **위치**: `Core/Src/app/sensors_i2c.c`에 추가 (또는 별도 파일)
- **주소**: 0x69 (Phase 3a scanner에서 확인됨)
- **샘플 레이트**: 100 Hz internal, 20 Hz batch read (FIFO 활용)
- **API**: `icm42670p_read_burst(handle, sample_t *out, int max_samples)`
- **변환**: raw → g (가속도), dps (각속도)

### ADS1115 FSR ADC driver

- **위치**: `Core/Src/app/sensors_i2c.c` 또는 별도 파일
- **주소**: 0x49 (Phase 3a scanner에서 확인됨)
- **모드**: continuous (단일 채널, channel switching v1엔 미사용)
- **샘플 레이트**: 250~475 SPS
- **API**: `ads1115_read_latest(handle, int16_t *raw)`

### Feature extraction

- **위치**: `Core/Src/app/features.c`
- v1 features:
  - IMU tilt angle (`atan2(accel_x, accel_z)` 같은 기본 변환)
  - FSR raw ADC value
  - (선택) accel RMS, gyro variance 등 ML 피처

### Trigger provider strategy pattern

- **파일**:
  - `Core/Inc/app/trigger.h` — `trigger_provider_t` 구조체, sensor_snapshot_t
  - `Core/Src/app/trigger.c` — selector + snapshot 정의
  - `Core/Src/app/trigger_rule.c` — FSR/IMU 임계값 비교
  - `Core/Src/app/trigger_ml.c` — Decision Tree stub
- **컴파일 타임 선택**: `config.h`의 `TRIGGER_SOURCE` 매크로 (TRIG_SRC_RULE / TRIG_SRC_ML)
- **인터페이스**:
  ```c
  typedef trig_event_t (*trig_eval_fn)(const sensor_snapshot_t *s, fsm_state_t cur);
  typedef struct { const char *name; trig_eval_fn eval; } trigger_provider_t;
  ```

### Rule-based trigger 임계값 (placeholder)

**Current** (config.h):
```
FSR_THRESHOLD_UP_ADC      = 2000
FSR_THRESHOLD_DOWN_ADC    = 500
IMU_TILT_UP_DEG           = 45
IMU_TILT_DOWN_DEG         = 10
```

**튜닝 절차**:
1. 사용자가 받침대/접촉면에 정상 압력 가했을 때 ADS1115 raw 측정 (RTT 로그)
2. 정상 / 떼었을 때 두 상태의 ADC 값 분포 기록
3. 두 분포가 분리되는 중간값을 임계값으로 + hysteresis margin
4. 동일하게 IMU tilt도 상태별 분포 측정
5. config.h 업데이트

### t_ml.c 본체 구현

- 현재 stub (mtx_i2c1 try만)
- Phase 4 후:
  - mtx_i2c1 acquire
  - ICM42670P FIFO burst read
  - ADS1115 read
  - mtx_i2c1 release
  - feature extraction
  - trigger_eval (provider 호출)
  - q_trigger_to_state 발행

### PHASE2_AUTO_TRIGGER 비활성화

- **현재**: defaultTask가 10초마다 UP/DOWN trigger 자동 발행
- **Phase 4 완료 시**: `config.h`의 `PHASE2_AUTO_TRIGGER` 1 → 0으로 변경
- freertos.c의 #if 블록이 자동 컴파일 제외됨

---

## 🌟 Phase 6 — Polish + finalization

### WS2812 RGB LED driver

- **하드웨어**: TIM2_CH2 (PB3) + DMA1_Channel7 (이미 .ioc에서 link됨)
- **프레임 포맷**: 24-bit GRB per LED (또는 RGB, 부품에 따라 다름)
- **LED 개수**: 미정 (사용자 확인 필요)
- **LED_FLASH_RED 패턴 업그레이드**: 현재 solid → blink (1 Hz)

### LED pattern blink engine

- 현재 actuators_set_led_pattern은 stateless (solid only)
- Phase 6: tick 기반 blink/fade state machine
- T_PID 또는 별도 task에서 매 사이클 actuators_tick(t) 호출

### Decision Tree ML model

- **선결조건**: Q-ML1 (제품 용도) 확정 + 학습 데이터 수집
- **절차**:
  1. T_ML stub에서 raw 센서 데이터를 RTT/SD로 수집
  2. 호스트 측 sklearn DecisionTreeClassifier 학습
  3. m2cgen 또는 micromlgen으로 C 코드 변환
  4. trigger_ml.c stub 교체
  5. trigger_provider 런타임 전환 (compile-time → runtime, optional)

### TBP-H70 PEC verification

- 현재 PEC 바이트는 read만 하고 검증 안 함
- CRC-8 polynomial `0x07` (X^8 + X^2 + X^1 + 1)
- 256 B lookup table → ~50 cycles per check
- mismatch 시 log warning + 1회 재시도

### Stack high-water mark 측정 + 축소

- 현재 stack 크기는 generous (bring-up용)
- `uxTaskGetStackHighWaterMark()`로 측정 후 config.h `STK_*_WORDS` 축소
- defaultTask heartbeat에 watermark 출력 추가 (1 Hz)

### t_state idle heartbeat 제거 (선택적)

- Phase 2 디버깅용으로 추가한 1초 heartbeat
- 시스템 신뢰도 확보 후 제거하면 RTT noise 감소

### `_Static_assert`를 더 많이

- state_table 길이 vs FSM_STATE_COUNT는 이미 적용됨
- PID gain range, queue depth 일관성 등에 추가 가능

---

## 🚫 Out of scope (v1 sprint)

이 항목들은 명시적으로 제외됨. 미래 sprint에서 별도 평가:

- **SPI2 오디오 codec 드라이버** (PB13/14/15 + PC6/7)
- **TIM3 초음파 input capture** (PB4)
- **SDIO + FATFS 로깅** (드라이버 init만 유지)
- **USART3 ESP32 명령 파서**
- **Left side IMU/FSR/IR 이중화** (드라이버는 배열 설계, init만 right)
- **ICM42670P INT1 하드웨어 인터럽트** (현재 polling, .ioc 변경 필요)
- **런타임 trigger provider 전환** (현재 컴파일 타임)
- **OTA 부트로더**

---

## ❓ Open decisions

### Q-ML1: 제품의 실제 용도

- **영향**: ML 유스케이스 결정, rule-based trigger 방향
- **임시 가정**: "FSR 압력 > threshold → FORCE_UP" (사용자가 받침대에
  접촉하면 가열 시작, 떼면 냉각). Phase 4 코드는 이 가정으로 진행하되
  임계값/방향은 config.h에서 즉시 변경 가능
- **확정 시점**: Phase 4 hardware tuning 단계

### LED count + 색상 순서 (WS2812)

- **영향**: Phase 6 WS2812 frame buffer 크기, 드라이버 구현
- **확정 시점**: Phase 6 시작 전

### Hot-path printf 제거 여부

- **현재**: t_pid heartbeat (1 Hz)는 RTT 로그 출력
- **고려 사항**: 시스템 안정 후 production build에서 제거하면 Flash + CPU 절감
- **결정 시점**: Phase 6

---

## 📋 Notes

- 작업 완료 시 항목을 ✓ 표시 또는 삭제
- 새 deferred 작업 발견 시 즉시 이 파일에 추가
- 주요 결정 변경은 plan 파일과 동기화 (commit message에 둘 다 언급)
- 이 파일 자체는 git에 commit (history 보존)

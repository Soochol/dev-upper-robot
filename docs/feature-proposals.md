# 추가 기능 제안

STM32F103RC + FreeRTOS 상체 로봇 프로젝트의 다음 단계 기능 제안. 현재 baseline(T_STATE/T_PID/T_ML/T_LOGGER 4-태스크, 20Hz FSM, m2cgen RandomForest, SD CSV 수집)은 동작 중. 이제 "진짜 로봇처럼 만드는 층"을 얹을 단계.

## 큰 그림 — 빠진 3개의 층

| 층 | 현재 | 문제 |
|---|---|---|
| **안전/회복** | SW 스택 오버플로 훅만 | HW 워치독 없음, FAULT는 터미널(전원 사이클만 복구) |
| **현장 튜닝** | `config.h` 컴파일타임 상수 | FSR 임계값/PID 게인을 현장에서 못 바꿈 |
| **호스트 제어** | RTT 단방향 + SD 오프라인 | 실시간 양방향 명령/텔레메트리 채널 없음 |

## 우선순위 제안

### P0 — 안 넣으면 다음에 물림

#### 1. IWDG (독립 워치독) + T_PID 킥

- [Core/Src/freertos.c](../Core/Src/freertos.c) `vApplicationStackOverflowHook`만 있고 HW 워치독 0. 데드락/I2C 행이면 로봇이 PWM을 물고 멈춤.
- 팬 PWM 반전 특성([CLAUDE.md](../CLAUDE.md)) 때문에 CCR=0으로 굳으면 **풀스피드 고정**. 위험.
- T_PID가 20Hz로 `HAL_IWDG_Refresh` 호출, 다른 태스크는 카나리 플래그를 T_PID에 전달.
- 공수: CC 기준 ~15분.

#### 2. FAULT 복구 경로

- [fsm.c](../Core/Src/app/fsm.c) FAULT 터미널. 현장 센서 일시 글리치(T_PID 센서 타임아웃)에도 전원 재투입 요구는 가혹.
- "FAULT → 10초 쿨다운 → FORCE_DOWN 복귀" 조건부 전환 추가.
- **안전 조건**: 팔이 사람을 누르고 있을 때 자동 복귀는 위험하니 "FSR 해제 + IMU 정지" 둘 다 만족해야 복귀.

### P1 — ML 제품화 가속

#### 3. Flash 파라미터 저장 (EEPROM 에뮬레이션)

- STM32F103RC 내장 Flash 2KB 페이지 2개면 충분.
- 저장 대상: PID Kp/Ki/Kd, FSR offset/gain, ML 임계값(0.6), 연속확인 틱(5), 자이로 게이팅(30dps).
- ST AN2594 EEPROM emulation 레퍼런스 그대로 포팅. 신규 파일 `params.c`/`params.h` 2개.
- 라이브러리 없이 `FLASH_Erase`/`FLASH_Program` 직접 호출 (검증된 패턴).

#### 4. 호스트 명령 프로토콜 (USART1 반이중, ASCII 라인)

- [usart.c](../Core/Src/usart.c) 존재하나 로거도 "Phase 6"로 미완([t_logger.c](../Core/Src/app/t_logger.c)). 바로 채울 자리.
- 라인 기반 명령:
  ```
  GET pid.kp
  SET pid.kp 1.25
  MODE collect
  DUMP csv
  SAVE          # → Flash 커밋
  ```
- 파서 200줄 이하.
- **USB-CDC 아니고 USART 선택 이유**: USB는 enumeration/스택 복잡도 큰데 로봇은 FTDI 1개면 충분. 보링 기술 선호.

#### 5. ML 모델 런타임 버전 플래그

- 현재 `model_rf.c` 947줄 하드코딩. A/B 없음. 재학습 후 빠른 롤백 경로 없음.
- 간단하게: `model_rf_v1.c` / `model_rf_v2.c` 두 개 링크, `params`에 저장된 모델 ID로 `score_fn_ptr` 스왑. 함수 포인터 1개.

### P2 — 나중에 고마워질 것

#### 6. 온보드 FSR 캘리브레이션 루틴

- "무부하 10초 → 최대부하 10초" 시퀀스로 offset/scale 자동 계산 → Flash 저장.
- 호스트 명령에서 트리거.
- FSR는 센서마다 개체차 크니까 공장 출하 절차로도 바로 써먹음.

#### 7. 전원 홀드 세이프 셧다운

- [t_state.c](../Core/Src/app/t_state.c)의 `OUT_PWR_HOLD_pin` 아직 미활용.
- SD 카드 write 중 전원 끊기면 FatFs 파티션 깨짐.
- 시퀀스: 전원 버튼 → T_STATE가 `flush_pending` 플래그 → T_LOGGER/sd_logger close → OUT_PWR_HOLD low.

#### 8. 성능 프로파일 카운터

- `vTaskGetRunTimeStats`용 TIM4 16bit free-run. 100단어 수준.
- ML 분류가 몇 ms 먹는지, I2C 블록이 얼마나 되는지 현장에서 눈으로 확인 가능.

## 안 추천

- **OTA 부트로더**: 1대 프로토타입에 혁신 토큰 낭비. SWD 물릴 수 있으면 오버킬.
- **USB-CDC**: UART로 충분. USB 스택은 F103RC 64KB RAM에 부담.
- **RTOS-aware 디버그 설정 복잡화**: 이미 되고 있으면 건드리지 말기.

## 첫 한 걸음

하나만 고른다면 **P0-1 (IWDG + T_PID 킥)**. 20분 작업, 이후 모든 디버깅/개발이 "로봇 안 행해서 땀 안 남" 상태에서 진행됨. 팬 CCR 반전 특성 때문에 진짜 안전 이슈.

## 현재 상태 요약 (참고)

### FreeRTOS 태스크

| 태스크 | PRIO | 주기 | 역할 |
|--------|------|------|------|
| T_STATE | 5 | 20Hz | FSM 틱, 트리거/결함 처리 |
| T_PID | 4 | 20Hz | IR×2 읽기, PID, 슬루 제한, heater/fan PWM |
| T_ML | 3 | 20Hz | IMU+FSR 수집, 특성 추출, trigger_eval |
| T_LOGGER | 1 | on-demand | q_log 드레인, RTT 출력 |
| defaultTask | 1 | 1Hz | 헬스 비콘, 힙/스택 고수위마크 |

I2C1 뮤텍스로 T_PID/T_ML 직렬화, 25ms 오프셋으로 경합 제거.

### FSM

- 상태: `FORCE_DOWN`(초기) / `FORCE_UP`(가열) / `FAULT`(터미널)
- 이벤트: `TRIGGER_FORCE_UP/DOWN`, `FAULT_REQUESTED`, `SAFETY_TIMEOUT`
- 60s 안전 타임아웃 (FORCE_UP만)

### ML 파이프라인

FSR + 가속도 → `tilt_update()` → `ml_window()`(슬라이딩 윈도우) → `score()` 호출 (m2cgen RandomForest 947줄). 9개 입력 → P(FORCE_DOWN), P(FORCE_UP). 의사결정: 0.6 임계값 + 5틱 연속확인 + 상태별 방향필터 + 30dps 자이로 게이팅.

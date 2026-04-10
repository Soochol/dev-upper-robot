---
name: cubemx-regen
description: STM32CubeMX 헤드리스 모드로 PSP.ioc를 재생성하고 변경사항을 검증. git stash 안전망으로 작업 손실 방지, User Code 섹션 보존 여부 확인, 페리페럴 설정 diff 요약까지 자동 수행.
disable-model-invocation: true
---

# CubeMX Regen

STM32CubeMX `.ioc` 파일을 헤드리스 모드로 재생성하는 안전 래퍼.

## 언제 쓰나

- `PSP.ioc`의 페리페럴 설정 변경 후
- GPIO/클럭 트리/DMA 채널 수정 후
- 새 페리페럴(I2C, SPI, TIM 등) 추가 후
- CubeMX 버전 업그레이드 후 코드 재생성이 필요할 때

## 실행 흐름

1. **사전 검증**
   - 작업 디렉토리가 깨끗한지 확인 (`git status`)
   - `PSP.ioc`가 존재하는지 확인
   - `~/STM32CubeMX/STM32CubeMX` 실행 파일 존재 확인
2. **안전망 생성**
   - 현재 상태를 `git stash`로 백업 (변경사항 있으면)
   - 혹은 사용자에게 먼저 커밋할지 물어봄
3. **헤드리스 재생성**
   - `scripts/cubemx_regen.sh` 실행
   - CubeMX 로그 캡처 후 파싱
4. **검증**
   - `git diff --stat`로 변경된 파일 요약
   - 다음 항목 자동 검사:
     - `/* USER CODE BEGIN */` 마커 카운트가 변하지 않았는지
     - `Core/`의 사용자 편집 파일(main.c, freertos.c 등)에서 USER CODE 섹션 사이 코드가 보존됐는지
     - `Drivers/`, `Middlewares/`는 검증 대상에서 제외 (CubeMX가 매번 다시 쓰기 때문)
5. **보고**
   - 변경된 페리페럴 목록 (CubeMX 로그의 `Load IP Config File for X` 파싱)
   - 신규/삭제된 소스 파일
   - 경고 (User Code 블록 소실 감지 시)

## 사용 예시

```
User: /cubemx-regen
Claude:
  [1/5] 사전 검증... OK (작업 디렉토리 깨끗)
  [2/5] 안전망: 변경사항 없어서 stash 생략
  [3/5] CubeMX 헤드리스 실행... (3.2초)
  [4/5] 검증:
    - 변경된 파일: 3개
      - Core/Inc/main.h         (5줄)
      - Core/Src/main.c          (12줄)
      - Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c  (skipped)
    - User Code 블록 보존: OK (42개 마커 → 42개 유지)
  [5/5] 감지된 페리페럴 변경:
    + I2C2 추가됨
    ~ TIM2 클럭 소스 변경
```

## 지원 스크립트

[cubemx_regen.sh](cubemx_regen.sh) — 실제 재생성을 수행하는 bash 스크립트.
  - 인자 없이 실행: `PSP.ioc` 재생성
  - 인자로 다른 `.ioc` 경로 지정 가능

## 주의사항

- **재생성 중에는 CubeMX를 GUI로 열어두지 말 것** — 파일 잠금 충돌 발생
- **User Code 블록을 반드시 확인** — 마커(`/* USER CODE BEGIN X */`)를 실수로 지우면 그 안의 코드가 증발
- `Drivers/`와 `Middlewares/Third_Party/`는 매 재생성마다 다시 쓰여지므로 diff가 많이 뜨는 게 정상
- 재생성 후에는 반드시 빌드(`cmake --build build/Debug`)해서 정상 동작 확인

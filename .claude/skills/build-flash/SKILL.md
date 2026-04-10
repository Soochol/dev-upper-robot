---
name: build-flash
description: STM32F103 프로젝트를 빌드하고 ST-Link로 보드에 플래시한 뒤 벡터 테이블 첫 64바이트를 읽어 검증. Debug 또는 Release 프리셋 선택 가능. 사용자가 "빌드해서 올려줘", "보드에 플래시해줘", "펌웨어 올려", "build and flash", "upload to board" 같은 요청을 하면 이 스킬을 호출해 cmake 빌드 → STM32_Programmer_CLI 플래시 → 벡터 테이블 검증까지 한 번에 수행.
---

# Build & Flash

STM32F103 프로젝트의 빌드 → 플래시 → 검증 루프를 한 커맨드로 자동화하는 스킬.

## 언제 쓰나

- 코드 수정 후 보드에 올려 동작 확인할 때
- CubeMX 재생성 후 새 설정이 실제로 반영됐는지 검증할 때
- FreeRTOS 태스크 추가/수정 후 빠르게 런타임 확인할 때
- 릴리스 빌드로 플래시해 최종 사이즈/성능 측정할 때

## 실행 방법

**사용자 직접 호출:**
```
/build-flash                           # 기본: Debug 프리셋
/build-flash Release                   # Release 프리셋
/build-flash Debug -DML_TRIGGER=ON     # ML 트리거로 빌드
```

**Claude 자동 호출 트리거:**
- "빌드해서 보드에 올려줘"
- "플래시해줘"
- "펌웨어 다시 올려"
- "build and flash"
- "upload to the board"

## 실행 흐름 (6단계)

스킬이 호출되면 [build_flash.sh](build_flash.sh)를 실행하며 다음을 수행합니다:

1. **사전 검증** — 프로젝트 루트 해결, `CMakePresets.json` 존재 확인, 프리셋 유효성 검사
2. **PATH 설정** — `STM32CubeCLT`의 Programmer CLI와 GCC 툴체인을 PATH에 주입
3. **CMake 빌드** — `cmake --preset <preset>` (no-op if cached) + `cmake --build build/<preset> -j$(nproc)`. `post-build-size.sh` 훅이 자동 발동해 Flash/RAM 사이즈 출력.
4. **프로브 연결 확인** — `STM32_Programmer_CLI --list st-link`로 ST-Link V2/V3 프로브 감지. FW 버전이 `V2J47S7` 미만이면 업그레이드 안내.
5. **플래시 + 전체 verify** — `STM32_Programmer_CLI -c port=SWD freq=4000 mode=UR -w <elf> -v -rst`. `-v` 플래그로 programmer가 내부적으로 전체 플래시 영역 검증. `-rst`로 소프트웨어 리셋.
6. **벡터 테이블 빠른 검증** — `0x08000000`에서 64바이트 읽어 초기 SP(SRAM 범위)와 Reset_Handler(Flash 범위)가 유효한지 확인.

## 인자

| 인자 | 값 | 기본값 |
|---|---|---|
| `$1` | 프리셋 이름 (`Debug` 또는 `Release`) | `Debug` |
| `$2...` | 추가 CMake configure 옵션 (예: `-DML_TRIGGER=ON`) | 없음 |

추가 프리셋은 `CMakePresets.json`에 정의되어 있어야 합니다. 추가 CMake 옵션은 `cmake --preset` 호출 시 그대로 전달됩니다.

## 종료 코드

| 코드 | 의미 |
|---|---|
| `0` | 성공 (빌드 + 플래시 + 검증 모두 통과) |
| `1` | 사용법 또는 인자 오류 (프리셋 미존재 등) |
| `2` | 빌드 실패 (`cmake --build` 실패) |
| `3` | ST-Link 프로브 미연결 |
| `4` | 플래시 실패 (Programmer CLI 에러) |
| `5` | 벡터 테이블 검증 실패 (플래시는 됐지만 바이너리 이상) |

## 주의사항

- **하드웨어 변경 액션**: 이 스킬은 보드의 플래시 메모리를 **덮어씁니다**. 실행 전에 현재 펌웨어가 중요한 상태가 아닌지 확인하세요.
- **프로브 단독 점유**: `STM32_Programmer_CLI`는 ST-Link USB를 단독 점유합니다. 동시에 `gdb_inspect.sh`를 돌리면 충돌합니다.
- **`mode=UR` 사용**: Connect Under Reset 모드로 붙기 때문에 이미 동작 중인 펌웨어가 워치독/락업 상태여도 안전하게 플래시할 수 있습니다.
- **Release 빌드 최적화**: `Release` 프리셋은 `-O2` 이상으로 컴파일되어 디버깅이 어려울 수 있습니다. 보통은 `Debug`로 충분합니다.

## 트러블슈팅

- **"ST-Link FW upgrade required"** → `/opt/st/stm32cubeclt_1.21.0/STLinkUpgrade.sh` 실행해 프로브 FW 업그레이드
- **"Cannot connect to target"** → SWD 케이블 연결 확인, BOOT0 점퍼가 GND에 있는지 확인, 타겟 전원 공급 확인
- **"File not found: PSP.elf"** → 빌드가 실패한 것. `[3/6]` 단계 로그를 확인
- **"Invalid preset"** → `cmake --list-presets`로 사용 가능한 프리셋 확인

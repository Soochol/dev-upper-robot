#!/bin/bash
# PreToolUse 훅: CubeMX 생성 디렉토리(Drivers/, Middlewares/Third_Party/) 편집 차단
#
# STDIN으로 받는 JSON 예시:
#   {
#     "tool_name": "Edit",
#     "tool_input": { "file_path": "/path/to/file.c", ... }
#   }
#
# 종료 코드:
#   0  허용
#   2  차단 (stderr 메시지가 Claude에게 전달됨)

input=$(cat)

# Edit/Write 외에는 무조건 허용
tool_name=$(echo "$input" | python3 -c 'import sys, json; print(json.load(sys.stdin).get("tool_name", ""))' 2>/dev/null)
case "$tool_name" in
    Edit|Write|NotebookEdit) ;;
    *) exit 0 ;;
esac

# file_path 추출
file_path=$(echo "$input" | python3 -c 'import sys, json; print(json.load(sys.stdin).get("tool_input", {}).get("file_path", ""))' 2>/dev/null)

if [ -z "$file_path" ]; then
    exit 0
fi

# 차단 대상 경로 패턴
case "$file_path" in
    */Drivers/STM32F1xx_HAL_Driver/*|*/Drivers/CMSIS/*)
        cat >&2 <<EOF
BLOCKED: $file_path

이 파일은 STM32CubeMX가 생성한 HAL/CMSIS 드라이버입니다.
직접 편집하면 다음 CubeMX 재생성 시 변경 사항이 소실됩니다.

올바른 변경 방법:
  1. PSP.ioc를 CubeMX로 편집 (또는 직접 수정)
  2. ~/STM32CubeMX/STM32CubeMX -q regen.script 로 재생성
  3. /* USER CODE BEGIN */ 섹션에 사용자 코드 작성

HAL 동작을 변경해야 한다면 Core/Src/stm32f1xx_hal_msp.c 또는
Core/Inc/stm32f1xx_hal_conf.h의 USER CODE 섹션을 사용하세요.
EOF
        exit 2
        ;;
    */Middlewares/Third_Party/*)
        cat >&2 <<EOF
BLOCKED: $file_path

이 파일은 FreeRTOS/FatFs 등 써드파티 미들웨어 원본입니다.
CubeMX가 관리하므로 직접 편집하면 재생성 시 덮어써집니다.

FreeRTOS 애플리케이션 코드는 Core/Src/freertos.c의 USER CODE 섹션에,
FatFs 사용자 코드는 FATFS/App/ 디렉토리에 작성하세요.
EOF
        exit 2
        ;;
esac

exit 0

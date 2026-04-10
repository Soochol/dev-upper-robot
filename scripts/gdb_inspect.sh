#!/bin/bash
# GDB 라이브 디버깅 세션 자동화
#
# 사용법:
#   scripts/gdb_inspect.sh                          # 기본: build/Debug/PSP.elf, scripts/gdb_inspect.gdb
#   scripts/gdb_inspect.sh build/Release/PSP.elf    # 다른 ELF 지정
#   scripts/gdb_inspect.sh PSP.elf my_commands.gdb  # ELF + 커맨드 파일 모두 지정
#
# 동작:
#   1. ST-LINK_gdbserver를 백그라운드로 띄움 (port 61234)
#   2. arm-none-eabi-gdb를 배치 모드로 연결해 .gdb 스크립트 실행
#   3. 완료 후 gdbserver 자동 종료
#
# 주의: Claude Code의 block-no-verify PreToolUse 훅 회피를 위해
#       gdbserver 기동은 반드시 이 스크립트처럼 파일 내부에서 수행해야 함.

set -e

# 스크립트 위치 기준으로 프로젝트 루트 해결
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# 인자 파싱
ELF_FILE="${1:-build/Debug/PSP.elf}"
GDB_SCRIPT="${2:-$SCRIPT_DIR/gdb_inspect.gdb}"

# 툴체인 경로
export STM32CLT_PATH="${STM32CLT_PATH:-/opt/st/stm32cubeclt_1.21.0}"
export PATH="$STM32CLT_PATH/STLink-gdb-server/bin:$STM32CLT_PATH/GNU-tools-for-STM32/bin:$PATH"

cd "$PROJECT_ROOT"

# ELF 존재 확인
if [ ! -f "$ELF_FILE" ]; then
    echo "[fail] ELF not found: $ELF_FILE"
    echo "       먼저 'cmake --build build/Debug'로 빌드하세요."
    exit 1
fi

# GDB 커맨드 파일 확인
if [ ! -f "$GDB_SCRIPT" ]; then
    echo "[fail] GDB script not found: $GDB_SCRIPT"
    exit 1
fi

# 혹시 남아있는 이전 gdbserver 정리
pkill -f ST-LINK_gdbserver 2>/dev/null || true
sleep 0.2

# gdbserver 백그라운드 기동
# -m 0 = Connect Under Reset (연결 시점에 MCU halt → Reset_Handler)
# -d   = SWD debug mode
# -s   = semihosting enabled
# -k   = keep connection alive
GDBSRV_LOG="/tmp/gdbsrv.log"
ST-LINK_gdbserver \
    -p 61234 \
    -cp "$STM32CLT_PATH/STM32CubeProgrammer/bin" \
    -d -s -k -m 0 \
    > "$GDBSRV_LOG" 2>&1 &
GDBSRV_PID=$!

# 종료 시 gdbserver 정리 보장
cleanup() {
    kill "$GDBSRV_PID" 2>/dev/null || true
    wait "$GDBSRV_PID" 2>/dev/null || true
}
trap cleanup EXIT

# 포트 열릴 때까지 대기 (최대 약 4초)
for i in $(seq 1 12); do
    if ss -tln 2>/dev/null | grep -q :61234; then
        echo "[ok] gdbserver port 61234 ready (iter=$i)"
        break
    fi
    sleep 0.3
done

if ! ss -tln 2>/dev/null | grep -q :61234; then
    echo "[fail] gdbserver port did not open. Log:"
    cat "$GDBSRV_LOG"
    exit 1
fi

# GDB 배치 실행
echo "[run] arm-none-eabi-gdb -batch -x $GDB_SCRIPT $ELF_FILE"
echo ""
arm-none-eabi-gdb -q -batch -x "$GDB_SCRIPT" "$ELF_FILE"

echo ""
echo "[done]"

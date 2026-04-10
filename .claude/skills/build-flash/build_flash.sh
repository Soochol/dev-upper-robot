#!/bin/bash
# STM32F103 빌드 + 플래시 + 검증 자동화 스크립트
#
# 사용법:
#   .claude/skills/build-flash/build_flash.sh              # Debug (기본)
#   .claude/skills/build-flash/build_flash.sh Release      # Release
#   .claude/skills/build-flash/build_flash.sh Debug -DML_TRIGGER=ON  # 추가 CMake 옵션
#   .claude/skills/build-flash/build_flash.sh --help
#
# 동작:
#   [1/6] 사전 검증 (프리셋, CubeCLT)
#   [2/6] PATH 설정
#   [3/6] cmake 빌드
#   [4/6] ST-Link 프로브 확인
#   [5/6] 플래시 + 전체 verify (-v)
#   [6/6] 벡터 테이블 빠른 검증
#
# 종료 코드:
#   0 성공
#   1 사용법/인자 오류
#   2 빌드 실패
#   3 프로브 미연결
#   4 플래시 실패
#   5 벡터 테이블 검증 실패

# --help 처리
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
fi

PRESET="${1:-Debug}"
shift 2>/dev/null || true
CMAKE_EXTRA_ARGS=("$@")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$PROJECT_ROOT"

ELF_FILE="build/$PRESET/PSP.elf"
MIN_FW="V2J47S7"

# ============================================================
# [1/6] 사전 검증
# ============================================================
echo "[1/6] 사전 검증..."

if [ ! -f "CMakePresets.json" ]; then
    echo "  ✗ CMakePresets.json 없음. 프로젝트 루트가 맞는지 확인하세요." >&2
    exit 1
fi

# STM32CubeCLT 경로
export STM32CLT_PATH="${STM32CLT_PATH:-/opt/st/stm32cubeclt_1.21.0}"
if [ ! -d "$STM32CLT_PATH" ]; then
    echo "  ✗ STM32CubeCLT 없음: $STM32CLT_PATH" >&2
    echo "    환경변수 STM32CLT_PATH로 경로를 지정할 수 있습니다." >&2
    exit 1
fi

# 사용 가능한 프리셋 확인
available_presets=$(cmake --list-presets 2>/dev/null | awk -F'"' '/"/ {print $2}')
if ! echo "$available_presets" | grep -qx "$PRESET"; then
    echo "  ✗ 프리셋 '$PRESET'이 CMakePresets.json에 없습니다." >&2
    echo "    사용 가능한 프리셋:" >&2
    echo "$available_presets" | sed 's/^/      - /' >&2
    exit 1
fi
echo "  ✓ preset=$PRESET"

# ============================================================
# [2/6] PATH 설정
# ============================================================
echo "[2/6] PATH 설정..."
export PATH="$STM32CLT_PATH/STM32CubeProgrammer/bin:$STM32CLT_PATH/GNU-tools-for-STM32/bin:$PATH"
if ! command -v STM32_Programmer_CLI >/dev/null 2>&1; then
    echo "  ✗ STM32_Programmer_CLI를 PATH에서 찾을 수 없습니다." >&2
    exit 1
fi
echo "  ✓ PATH 설정 완료"

# ============================================================
# [3/6] CMake 빌드
# ============================================================
echo "[3/6] 빌드..."
if [ ${#CMAKE_EXTRA_ARGS[@]} -gt 0 ]; then
    echo "  cmake 추가 옵션: ${CMAKE_EXTRA_ARGS[*]}"
fi
if ! cmake --preset "$PRESET" "${CMAKE_EXTRA_ARGS[@]}" >/dev/null 2>&1; then
    # --preset 자체가 실패하는 경우는 드물지만 일단 출력
    cmake --preset "$PRESET" "${CMAKE_EXTRA_ARGS[@]}" 2>&1 | tail -20
fi

if ! cmake --build "build/$PRESET" -j"$(nproc)"; then
    echo "  ✗ 빌드 실패" >&2
    exit 2
fi

if [ ! -f "$ELF_FILE" ]; then
    echo "  ✗ ELF 산출물 없음: $ELF_FILE" >&2
    exit 2
fi

# arm-none-eabi-size는 post-build-size 훅이 자동 실행하지만,
# 스킬이 스크립트로 호출될 때는 훅이 발동하지 않으므로 여기서 직접 호출.
if command -v arm-none-eabi-size >/dev/null 2>&1; then
    echo ""
    arm-none-eabi-size "$ELF_FILE"
    echo ""
fi

# ============================================================
# [4/6] ST-Link 프로브 연결 확인
# ============================================================
echo "[4/6] ST-Link 프로브 확인..."
probe_list=$(STM32_Programmer_CLI --list st-link 2>&1)
if ! echo "$probe_list" | grep -q "ST-Link Probe 0"; then
    echo "  ✗ ST-Link 프로브 미연결" >&2
    echo "    - USB 케이블 연결 확인" >&2
    echo "    - 'lsusb | grep -i stlink'으로 USB 인식 여부 확인" >&2
    echo "    - 프로브 드라이버(udev rules) 설치 확인" >&2
    exit 3
fi

clean_probe=$(echo "$probe_list" | sed 's/\x1b\[[0-9;]*m//g')
probe_sn=$(echo "$clean_probe" | grep "ST-LINK SN" | head -1 | sed 's/.*: *//' | tr -d ' \r\n')
probe_fw=$(echo "$clean_probe" | grep "ST-LINK FW" | head -1 | sed 's/.*: *//' | tr -d ' \r\n')
echo "  ✓ 프로브 감지: SN=$probe_sn, FW=$probe_fw"

# FW 버전 비교: V2JxxSy 형식에서 xx 숫자 추출
fw_num=$(echo "$probe_fw" | sed -n 's/^V2J\([0-9]*\)S.*/\1/p')
min_num=$(echo "$MIN_FW" | sed -n 's/^V2J\([0-9]*\)S.*/\1/p')
if [ -n "$fw_num" ] && [ -n "$min_num" ] && [ "$fw_num" -lt "$min_num" ]; then
    echo "  ⚠ FW가 최소 요구 버전($MIN_FW)보다 낮습니다." >&2
    echo "    GDB 연동이 필요하면 $STM32CLT_PATH/STLinkUpgrade.sh 로 업그레이드하세요." >&2
    echo "    (플래시는 계속 진행)" >&2
fi

# ============================================================
# [5/6] 플래시 + 전체 verify
# ============================================================
echo "[5/6] 플래시 + 검증..."
flash_out=$(STM32_Programmer_CLI \
    -c port=SWD freq=4000 mode=UR \
    -w "$ELF_FILE" -v -rst 2>&1)
flash_rc=$?

# ANSI 이스케이프 제거 + 핵심 라인 추출
clean_out=$(echo "$flash_out" | sed 's/\x1b\[[0-9;]*m//g')

if [ $flash_rc -ne 0 ] || ! echo "$clean_out" | grep -q "File download complete"; then
    echo "  ✗ 플래시 실패" >&2
    echo "$clean_out" | tail -15 >&2
    exit 4
fi

# 사이즈와 시간 파싱
flash_size=$(echo "$clean_out" | grep -E "Size\s*:" | head -1 | sed -E 's/.*Size\s*:\s*//' | tr -d '\r')
flash_time=$(echo "$clean_out" | grep -E "Time elapsed" | head -1 | sed -E 's/.*Time elapsed during download operation:\s*//' | tr -d '\r')
echo "  ✓ Size:  ${flash_size:-?}"
echo "  ✓ Time:  ${flash_time:-?}"
echo "  ✓ File download complete (전체 verify 포함)"
if echo "$clean_out" | grep -q "Software reset"; then
    echo "  ✓ Software reset 수행"
fi

# ============================================================
# [6/6] 벡터 테이블 빠른 검증
# ============================================================
echo "[6/6] 벡터 테이블 빠른 검증..."
dump_out=$(STM32_Programmer_CLI -c port=SWD freq=4000 mode=UR -r32 0x08000000 0x40 2>&1)
clean_dump=$(echo "$dump_out" | sed 's/\x1b\[[0-9;]*m//g')

# "0x08000000 : W0 W1 W2 W3" 형식에서 첫 줄 파싱
first_line=$(echo "$clean_dump" | grep -E "^0x08000000\s*:" | head -1)
if [ -z "$first_line" ]; then
    echo "  ✗ 벡터 테이블 읽기 실패" >&2
    echo "$clean_dump" | tail -10 >&2
    exit 5
fi

# 공백으로 split해서 3번째 토큰(W0) 추출
initial_sp=$(echo "$first_line" | awk '{print $3}')
reset_handler=$(echo "$first_line" | awk '{print $4}')

echo "  0x08000000: $initial_sp  (초기 SP)"
echo "  0x08000004: $reset_handler  (Reset_Handler)"

# STM32F103RC: SRAM 48KB at 0x20000000, Flash 256KB at 0x08000000
# SP 유효 범위: 0x20000001 ~ 0x2000C000 (RAM top, 초기값은 top과 같음)
# Reset_Handler 유효 범위: 0x08000000 ~ 0x0803FFFF (Thumb 주소는 LSB=1일 수 있음)

# SP가 SRAM 영역인지 확인 (첫 4 nibble이 "2000")
sp_ok=0
case "$initial_sp" in
    2000[0-9A-Fa-f][0-9A-Fa-f][0-9A-Fa-f][0-9A-Fa-f]) sp_ok=1 ;;
esac

# Reset_Handler가 Flash 영역인지 확인 (첫 3 nibble이 "080", 4번째 nibble이 0-3)
rh_ok=0
case "$reset_handler" in
    080[0-3][0-9A-Fa-f][0-9A-Fa-f][0-9A-Fa-f][0-9A-Fa-f]) rh_ok=1 ;;
esac

if [ "$sp_ok" = "1" ] && [ "$rh_ok" = "1" ]; then
    echo "  ✓ 벡터 테이블 정상 (SP→SRAM, Reset_Handler→Flash)"
else
    echo "  ✗ 벡터 테이블 이상" >&2
    [ "$sp_ok" = "0" ] && echo "    - 초기 SP가 SRAM 범위(0x20000000~0x2000FFFF) 밖입니다" >&2
    [ "$rh_ok" = "0" ] && echo "    - Reset_Handler가 Flash 범위(0x08000000~0x0803FFFF) 밖입니다" >&2
    exit 5
fi

echo ""
echo "✓ done (build + flash + verify)"
exit 0

#!/bin/bash
# STM32CubeMX 헤드리스 재생성 스크립트
#
# 사용법:
#   .claude/skills/cubemx-regen/cubemx_regen.sh                # PSP.ioc 재생성
#   .claude/skills/cubemx-regen/cubemx_regen.sh Other.ioc      # 다른 .ioc 지정
#
# 동작:
#   1. 사전 검증 (ioc 파일, CubeMX 실행 파일, git 상태)
#   2. git stash (변경사항 있을 때)
#   3. CubeMX -q 헤드리스 실행
#   4. git diff --stat 요약
#   5. User Code 블록 마커 카운트 검증

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$PROJECT_ROOT"

IOC_FILE="${1:-PSP.ioc}"
CUBEMX_BIN="${CUBEMX_BIN:-$HOME/STM32CubeMX/STM32CubeMX}"

# --- [1/5] 사전 검증 ---
echo "[1/5] 사전 검증..."
if [ ! -f "$IOC_FILE" ]; then
    echo "  ✗ ioc 파일 없음: $IOC_FILE" >&2
    exit 1
fi
if [ ! -x "$CUBEMX_BIN" ]; then
    echo "  ✗ CubeMX 실행 파일 없음: $CUBEMX_BIN" >&2
    echo "  힌트: 환경변수 CUBEMX_BIN으로 경로를 지정할 수 있습니다." >&2
    exit 1
fi
echo "  ✓ ioc: $IOC_FILE"
echo "  ✓ cubemx: $CUBEMX_BIN"

# --- [2/5] 안전망: 변경사항 있으면 stash ---
echo ""
echo "[2/5] 안전망..."
STASHED=0
if ! git diff --quiet HEAD 2>/dev/null; then
    STASH_NAME="cubemx-regen-safety-$(date +%s)"
    if git stash push -u -m "$STASH_NAME" >/dev/null 2>&1; then
        echo "  ✓ 현재 변경사항을 stash에 백업: $STASH_NAME"
        echo "  (복구: git stash pop)"
        STASHED=1
    fi
else
    echo "  ✓ 작업 디렉토리 깨끗 (stash 불필요)"
fi

# 재생성 전 USER CODE 마커 카운트 (Core/ 하위만)
PRE_MARKER_COUNT=$(grep -rn "USER CODE BEGIN" Core/ 2>/dev/null | wc -l)
echo "  · 기존 USER CODE BEGIN 마커 수: $PRE_MARKER_COUNT (Core/)"

# --- [3/5] CubeMX 헤드리스 실행 ---
echo ""
echo "[3/5] CubeMX 헤드리스 실행..."
SCRIPT_FILE=$(mktemp /tmp/cubemx_regen.XXXXXX.script)
trap "rm -f $SCRIPT_FILE" EXIT

cat > "$SCRIPT_FILE" <<EOF
config load $PROJECT_ROOT/$IOC_FILE
project generate
exit
EOF

LOG_FILE="/tmp/cubemx_regen.log"
START_TIME=$(date +%s.%N)
if timeout 180 "$CUBEMX_BIN" -q "$SCRIPT_FILE" > "$LOG_FILE" 2>&1; then
    END_TIME=$(date +%s.%N)
    ELAPSED=$(awk "BEGIN {printf \"%.1f\", $END_TIME - $START_TIME}")
    echo "  ✓ 재생성 완료 (${ELAPSED}초)"
else
    echo "  ✗ 재생성 실패. 로그:" >&2
    tail -30 "$LOG_FILE" >&2
    exit 1
fi

# --- [4/5] 검증 ---
echo ""
echo "[4/5] 검증..."

# 변경된 파일 요약
CHANGED=$(git diff --stat 2>/dev/null | tail -1)
CHANGED_FILES=$(git diff --name-only 2>/dev/null | wc -l)
if [ "$CHANGED_FILES" -eq 0 ]; then
    echo "  ✓ 변경된 파일 없음 (멱등성 확인 — .ioc 설정이 이미 반영된 상태)"
else
    echo "  · 변경된 파일: $CHANGED_FILES개"
    git diff --stat 2>/dev/null | head -20 | sed 's/^/    /'
fi

# USER CODE 마커 카운트 비교
POST_MARKER_COUNT=$(grep -rn "USER CODE BEGIN" Core/ 2>/dev/null | wc -l)
if [ "$POST_MARKER_COUNT" -eq "$PRE_MARKER_COUNT" ]; then
    echo "  ✓ USER CODE 마커 보존: $PRE_MARKER_COUNT개 유지"
elif [ "$POST_MARKER_COUNT" -gt "$PRE_MARKER_COUNT" ]; then
    echo "  ℹ USER CODE 마커 증가: $PRE_MARKER_COUNT → $POST_MARKER_COUNT (신규 페리페럴 추가?)"
else
    echo "  ⚠ USER CODE 마커 감소: $PRE_MARKER_COUNT → $POST_MARKER_COUNT" >&2
    echo "     일부 USER CODE 섹션이 소실됐을 가능성! git diff Core/ 로 확인하세요." >&2
fi

# --- [5/5] 감지된 페리페럴 ---
echo ""
echo "[5/5] 재생성된 페리페럴 (CubeMX 로그 기준):"
grep "Load IP Config File for" "$LOG_FILE" 2>/dev/null \
    | sed 's/.*Load IP Config File for /    · /' \
    | sort -u \
    | head -20 || echo "  (없음)"

# --- 요약 ---
echo ""
if [ "$STASHED" -eq 1 ]; then
    echo "↻ 원래 작업을 되살리려면: git stash pop"
fi
if [ "$CHANGED_FILES" -gt 0 ]; then
    echo "▶ 다음 단계: cmake --build build/Debug -j\$(nproc)"
fi
echo "✓ done"

#!/bin/bash
# PostToolUse 훅: cmake --build 실행 직후 ELF 사이즈 자동 출력
#
# STDIN으로 받는 JSON 예시:
#   {
#     "tool_name": "Bash",
#     "tool_input": { "command": "cmake --build build/Debug -j8", ... },
#     "tool_response": { "stdout": "...", ... }
#   }

input=$(cat)

tool_name=$(echo "$input" | python3 -c 'import sys, json; print(json.load(sys.stdin).get("tool_name", ""))' 2>/dev/null)
if [ "$tool_name" != "Bash" ]; then
    exit 0
fi

command=$(echo "$input" | python3 -c 'import sys, json; print(json.load(sys.stdin).get("tool_input", {}).get("command", ""))' 2>/dev/null)

# cmake --build 커맨드에서만 동작
if ! echo "$command" | grep -q "cmake --build"; then
    exit 0
fi

# 빌드 디렉토리 추정 (build/Debug 또는 build/Release)
for build_dir in build/Debug build/Release; do
    elf="$build_dir/PSP.elf"
    if [ -f "$elf" ]; then
        size_bin=/opt/st/stm32cubeclt_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-size
        if [ -x "$size_bin" ]; then
            echo "📊 [$build_dir/PSP.elf]"
            "$size_bin" "$elf" 2>/dev/null

            # 이전 빌드 대비 증감 추적 (선택)
            cache_file="/tmp/psp_size_$(basename $build_dir).cache"
            current=$("$size_bin" "$elf" 2>/dev/null | awk 'NR==2 {print $1, $2, $3}')
            if [ -f "$cache_file" ]; then
                prev=$(cat "$cache_file")
                if [ "$current" != "$prev" ]; then
                    echo "   (변경 감지: 이전=$prev → 현재=$current)"
                fi
            fi
            echo "$current" > "$cache_file"
        fi
        break
    fi
done

exit 0

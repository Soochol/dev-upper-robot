# CLAUDE.md

STM32F103RC + FreeRTOS 베어메탈 프로젝트. CubeMX(`PSP.ioc`) + CMake + STM32CubeCLT 기반.

세부 규칙은 `.claude/rules/` 참조:

| 파일 | 내용 |
|------|------|
| `toolchain.md` | STM32CubeCLT 경로 + 셸 환경 |
| `debugging.md` | GDB 디버깅 + 훅 우회 |
| `safety.md` | RDP 경고, ST-Link FW 버전 |
| `project-structure.md` | 디렉토리 구조 + 페리페럴 |

## 팬 제어 (HW 특성)

- **PA9 (TIM1_CH2)**: PWM 극성 반전. CCR=0 → 풀스피드, CCR=1000 → 최저속. `actuators_set_fan_duty_pct()` 내부에서 `CCR = 1000 - pct*10` 변환 처리.
- **PA10 (OUT_FAN_PWR)**: 팬 전원 master enable. HIGH=on, LOW=off. pct > 0이면 HIGH.
- **PA11 (OUT_BLOWER)**: 현재 HW 미연결 (효과 없음). V1에서는 간헐 송풍용 별도 블로워 제어.

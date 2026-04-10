---
description: 하드웨어 안전 경고 (RDP, ST-Link FW 버전)
globs: "*"
---

# 안전 규칙

- 옵션 바이트 `RDP` Level 2는 **복구 불가** -- 쓰기 전 `-ob displ`로 반드시 확인.
- **ST-Link FW 최소 버전**: `V2J47S7` 이상. 낮으면 `STM32_Programmer_CLI`는 동작해도 `ST-LINK_gdbserver`가 `firmware upgrade required` 에러로 거부. 업그레이드 툴: `/opt/st/stm32cubeclt_1.21.0/STLinkUpgrade.sh` (Java GUI).

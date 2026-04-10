---
description: STM32CubeCLT 툴체인 경로 및 셸 환경 설정
globs: "*"
---

# 툴체인 경로

- **STM32CubeCLT**: `/opt/st/stm32cubeclt_1.21.0/`
  - Programmer: `$STM32CLT_PATH/STM32CubeProgrammer/bin/STM32_Programmer_CLI`
  - GCC: `$STM32CLT_PATH/GNU-tools-for-STM32/bin/arm-none-eabi-*`
  - GDB server: `$STM32CLT_PATH/STLink-gdb-server/bin/ST-LINK_gdbserver`
  - SVD: `$STM32CLT_PATH/STMicroelectronics_CMSIS_SVD/`
- **STM32CubeMX**: `~/STM32CubeMX/STM32CubeMX` (헤드리스 `-q` 모드 지원)

셸 환경 설정:

```bash
export STM32CLT_PATH=/opt/st/stm32cubeclt_1.21.0
export PATH=$STM32CLT_PATH/STM32CubeProgrammer/bin:$PATH
export PATH=$STM32CLT_PATH/GNU-tools-for-STM32/bin:$PATH
export PATH=$STM32CLT_PATH/STLink-gdb-server/bin:$PATH
```

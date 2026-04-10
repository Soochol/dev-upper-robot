---
description: 프로젝트 디렉토리 구조 및 활성 페리페럴
globs: "*"
---

# 프로젝트 구조

- `Core/` -- 애플리케이션
  - `Src/main.c` -- 엔트리, HAL/페리페럴 초기화 후 `osKernelStart()`
  - `Src/freertos.c` -- FreeRTOS 애플리케이션 코드 (`StartDefaultTask` 등)
  - `Src/{adc,dma,gpio,i2c,sdio,spi,tim,usart}.c` -- 페리페럴 초기화 (CubeMX 생성)
- `Drivers/` -- HAL/LL/CMSIS (생성물)
- `Middlewares/` -- FreeRTOS, FatFs (생성물)
- `FATFS/` -- FatFs 애플리케이션(`App/`) 및 타겟 바인딩(`Target/`)
- `PSP.ioc` -- CubeMX 설정 (단일 진실 공급원)
- `CMakeLists.txt` / `CMakePresets.json` / `cmake/` -- 빌드
- `STM32F103XX_FLASH.ld` / `startup_stm32f103xe.s` -- 링커/스타트업
- `build/` -- 빌드 산출물

# FreeRTOS 상태

FreeRTOS 동작 중. 현재 `defaultTask` 1개만 실행되는 clean baseline 상태 (commit `3cd61a1`). 이전 커밋(`fa1f5da`)엔 app 태스크 4개가 있었지만 디버깅 베이스라인 확보를 위해 스트립됨. 디버깅 시 RTOS-aware 모드 권장.

# 활성 페리페럴

PSP.ioc 기준: ADC1, I2C1, SDIO, SPI1/2, TIM1/2/3, USART1/3, USB_DEVICE

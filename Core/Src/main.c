/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "spi.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SEGGER_RTT.h"
#include "iwdg.h"
#include "stm32f1xx_hal.h"
#include "app/config.h"
#include "app/ipc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ADC1 DMA circular buffer — filled in background by DMA1_Channel1.
 * Indices (matching .ioc rank order):
 *   [0] ADC_BATT     (PA0)  battery voltage
 *   [1] ADC_TEMP_BD  (PA1)  board temperature
 *   [2] ADC_TEMP_AIR (PA2)  air / enclosure temperature
 *   [3] ADC_CURRENT  (PA3)  motor / heater current
 * Word alignment matches DMA_MDATAALIGN_WORD from .ioc.
 *
 * Externally visible (no `static`) so sensor_task in freertos.c can read
 * the latest values via an extern declaration. Still volatile because the
 * DMA controller writes it asynchronously to CPU loads.
 */
volatile uint32_t adc_buf[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SDIO_SD_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* --- 1. Power latch ------------------------------------------------------
   * PC5 = OUT_PWR_HOLD. The board power supply is latched by this pin: when
   * the user presses the power button (PC0 = IN_PWR_SW), power is applied
   * just long enough for the MCU to boot. We MUST drive PC5 HIGH as soon as
   * possible, otherwise the supply drops the moment the user releases the
   * button and we brown out mid-boot.
   *
   * MX_GPIO_Init() has already configured PC5 as push-pull output above,
   * so HAL_GPIO_WritePin is safe here. We do this FIRST in block 2, before
   * any peripheral start-up that might delay execution.
   */
  HAL_GPIO_WritePin(OUT_PWR_HOLD_GPIO_Port, OUT_PWR_HOLD_Pin, GPIO_PIN_SET);

  /* --- 1c. IWDG reset post-mortem indicator -------------------------------
   * If RCC_CSR.IWDGRSTF is set, the previous reset was caused by the
   * watchdog (a task deadlocked or starved). Snapshot RCC_CSR into the
   * global g_reset_cause so T_STATE can enter FSM_FAULT directly at init,
   * then clear all reset flags via RMVF. While still in pre-FreeRTOS
   * code, blink GPIO LEDs PB0+PB1 to give the user a clear visual signal
   * that this boot is *not* normal. SK6812 is not used here because it
   * depends on TIM2/DMA + actuators_init() which only runs after T_PID
   * starts. GPIO LEDs are init'd by MX_GPIO_Init() above and need no
   * extra setup. */
  g_reset_cause = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;
  if (g_reset_cause & RCC_CSR_IWDGRSTF) {
    for (int i = 0; i < BOOT_FAULT_BLINK_COUNT; i++) {
      HAL_GPIO_WritePin(OUT_LED_R1_GPIO_Port, OUT_LED_R1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(OUT_LED_R2_GPIO_Port, OUT_LED_R2_Pin, GPIO_PIN_SET);
      HAL_Delay(BOOT_FAULT_BLINK_MS);
      HAL_GPIO_WritePin(OUT_LED_R1_GPIO_Port, OUT_LED_R1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(OUT_LED_R2_GPIO_Port, OUT_LED_R2_Pin, GPIO_PIN_RESET);
      HAL_Delay(BOOT_FAULT_BLINK_MS);
    }
  }

  /* --- 1b. SEGGER RTT init -------------------------------------------------
   * RTT auto-initializes on first SEGGER_RTT_Write(), but calling Init
   * explicitly here lets us see the control block in memory before the first
   * printf — useful if the debugger is attached but logs aren't showing.
   */
  SEGGER_RTT_Init();

  /* --- 2. printf retargeting (unbuffered stdout) ---------------------------
   * newlib defaults stdout to line-buffered, which delays output until '\n'
   * and may also fail silently if the heap is too small. Force unbuffered
   * mode so every printf() flushes immediately via __io_putchar -> USART1.
   */
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\n=== PSP V1.00 boot ===\r\n");
  printf("SYSCLK=%lu Hz, HCLK=%lu Hz\r\n",
         HAL_RCC_GetSysClockFreq(), HAL_RCC_GetHCLKFreq());

  /* --- 3. ADC1 DMA background sampling -------------------------------------
   * 4 channels, continuous conversion, circular DMA (see .ioc). Once started
   * the adc_buf[] array is kept up-to-date by DMA1_Channel1 with zero CPU
   * overhead. Readers just access adc_buf[i] directly.
   */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 4) != HAL_OK)
  {
    printf("ERR: HAL_ADC_Start_DMA failed\r\n");
    Error_Handler();
  }

  /* --- 4. Hardware watchdog ------------------------------------------------
   * IWDG is started inside T_WDG (Core/Src/app/t_wdg.c), not here. Starting
   * it from the kicker task avoids the timing gap between "IWDG counting"
   * and "kicker is ready" [C1]. Do NOT call MX_IWDG_Init() in main.c. */
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* osKernelStart() above never returns under normal operation. If we
   * reach this loop the scheduler failed to launch (heap exhaustion in
   * MX_FREERTOS_Init, MPU misconfig, etc.) — fail loud rather than
   * silently fall back to a super-loop that no longer matches the rest
   * of the firmware. */
  printf("FATAL: scheduler exited\r\n");
  Error_Handler();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Pre-main RTT initializer (constructor).
 *
 * Runs from __libc_init_array() BEFORE main() is entered. This ensures that
 * the _SEGGER_RTT control block has its magic string and ring buffer
 * pointers set up by the time the debugger halts at main(), so Cortex-Debug's
 * "monitor rtt setup" succeeds on the very first try and the user does not
 * need to press F5 twice to get logs flowing.
 *
 * Note: the control block's absolute address depends on the current .bss
 * layout and changes whenever new global/static variables are added. Do
 * NOT hardcode the address in launch.json — use `monitor rtt setup
 * 0x20000000 0xC000 "SEGGER RTT"` which asks OpenOCD to scan the entire
 * STM32F103RC RAM (48 KB) for the SEGGER magic marker and locate the
 * control block dynamically. Query the current address with:
 *     arm-none-eabi-nm build/Debug/PSP.elf | grep _SEGGER_RTT
 *
 * Mechanism: GCC __attribute__((constructor)) registers this function in the
 * .init_array section. The startup_stm32f103xe.s Reset_Handler calls
 * __libc_init_array() right before main(), which iterates that section and
 * invokes every registered constructor. Same mechanism C++ uses for global
 * object construction.
 *
 * The duplicate call inside USER CODE BEGIN 2 is intentional defensive
 * redundancy — calling SEGGER_RTT_Init() twice is harmless (it just rewrites
 * the same magic and pointers).
 */
__attribute__((constructor))
static void rtt_pre_main_init(void)
{
  SEGGER_RTT_Init();
}

/**
 * @brief  printf() retarget hook — dual output to SEGGER RTT + USART1.
 *
 * Overrides the weak __io_putchar stub in Core/Src/syscalls.c so newlib's
 * _write() routes every character through BOTH transports:
 *
 *   1. SEGGER RTT channel 0 (non-blocking, ~10 ns per byte)
 *      - Visible in Cortex-Debug "RTT Ch:0 console" when a debugger is
 *        attached via OpenOCD + ST-Link
 *      - Uses SEGGER_RTT_MODE_NO_BLOCK_SKIP: drops bytes if buffer full,
 *        never stalls the CPU
 *
 *   2. USART1 PB6 TX @ 115200 8N1 (blocking, ~87 µs per byte)
 *      - Visible on any USB-TTL adapter or ST-Link VCP connected to PB6
 *      - Keeps logs available when no debugger is attached
 *
 * RTT is called first so that debug-attached sessions get the log at
 * effectively zero latency, with the slower UART path following. Blocking
 * UART transmit is acceptable because printf() is only used from the main
 * loop; do NOT call printf() from an ISR (deadlock risk).
 */
int __io_putchar(int ch)
{
  SEGGER_RTT_Write(0, &ch, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

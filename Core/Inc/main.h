/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OUT_ESP32_EN_Pin GPIO_PIN_13
#define OUT_ESP32_EN_GPIO_Port GPIOC
#define IN_PWR_SW_Pin GPIO_PIN_0
#define IN_PWR_SW_GPIO_Port GPIOC
#define IN_FUNC_SW1_Pin GPIO_PIN_1
#define IN_FUNC_SW1_GPIO_Port GPIOC
#define IN_FUNC_SW2_Pin GPIO_PIN_2
#define IN_FUNC_SW2_GPIO_Port GPIOC
#define ADC_BATT_Pin GPIO_PIN_0
#define ADC_BATT_GPIO_Port GPIOA
#define ADC_TEMP_BD_Pin GPIO_PIN_1
#define ADC_TEMP_BD_GPIO_Port GPIOA
#define ADC_TEMP_AIR_Pin GPIO_PIN_2
#define ADC_TEMP_AIR_GPIO_Port GPIOA
#define ADC_CURRENT_Pin GPIO_PIN_3
#define ADC_CURRENT_GPIO_Port GPIOA
#define SPI_IMU_SCK_Pin GPIO_PIN_5
#define SPI_IMU_SCK_GPIO_Port GPIOA
#define SPI_IMU_MISO_Pin GPIO_PIN_6
#define SPI_IMU_MISO_GPIO_Port GPIOA
#define SPI_IMU_MOSI_Pin GPIO_PIN_7
#define SPI_IMU_MOSI_GPIO_Port GPIOA
#define OUT_IMU_CS_Pin GPIO_PIN_4
#define OUT_IMU_CS_GPIO_Port GPIOC
#define OUT_PWR_HOLD_Pin GPIO_PIN_5
#define OUT_PWR_HOLD_GPIO_Port GPIOC
#define OUT_LED_R1_Pin GPIO_PIN_0
#define OUT_LED_R1_GPIO_Port GPIOB
#define OUT_LED_R2_Pin GPIO_PIN_1
#define OUT_LED_R2_GPIO_Port GPIOB
#define IN_CODEC_REQ_Pin GPIO_PIN_2
#define IN_CODEC_REQ_GPIO_Port GPIOB
#define UART3_TX_ESP32_Pin GPIO_PIN_10
#define UART3_TX_ESP32_GPIO_Port GPIOB
#define UART3_RX_ESP32_Pin GPIO_PIN_11
#define UART3_RX_ESP32_GPIO_Port GPIOB
#define OUT_CODEC_CS_Pin GPIO_PIN_12
#define OUT_CODEC_CS_GPIO_Port GPIOB
#define SPI2_SCK_CODEC_Pin GPIO_PIN_13
#define SPI2_SCK_CODEC_GPIO_Port GPIOB
#define SPI2_MISO_CODEC_Pin GPIO_PIN_14
#define SPI2_MISO_CODEC_GPIO_Port GPIOB
#define SPI2_MOSI_CODEC_Pin GPIO_PIN_15
#define SPI2_MOSI_CODEC_GPIO_Port GPIOB
#define OUT_CODEC_RESET_Pin GPIO_PIN_6
#define OUT_CODEC_RESET_GPIO_Port GPIOC
#define OUT_CODE_DCS_Pin GPIO_PIN_7
#define OUT_CODE_DCS_GPIO_Port GPIOC
#define TIM1_CH1_HEATER_Pin GPIO_PIN_8
#define TIM1_CH1_HEATER_GPIO_Port GPIOA
#define TIM1_CH2_FAN_Pin GPIO_PIN_9
#define TIM1_CH2_FAN_GPIO_Port GPIOA
#define OUT_FAN_PWR_Pin GPIO_PIN_10
#define OUT_FAN_PWR_GPIO_Port GPIOA
#define OUT_BLOWER_Pin GPIO_PIN_11
#define OUT_BLOWER_GPIO_Port GPIOA
#define IN_SDIO_CHK_Pin GPIO_PIN_15
#define IN_SDIO_CHK_GPIO_Port GPIOA
#define TIM2_CH2_RGB_LED_Pin GPIO_PIN_3
#define TIM2_CH2_RGB_LED_GPIO_Port GPIOB
#define PW_Ultrasonic_Pin GPIO_PIN_4
#define PW_Ultrasonic_GPIO_Port GPIOB
#define I2C1_SCL_IR_Pin GPIO_PIN_8
#define I2C1_SCL_IR_GPIO_Port GPIOB
#define I2C1_SDA_IR_Pin GPIO_PIN_9
#define I2C1_SDA_IR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

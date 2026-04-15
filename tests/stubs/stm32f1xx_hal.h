/**
 * @file    stm32f1xx_hal.h  (HOST STUB)
 * @brief   Minimal stub so sensors_i2c.h compiles on the host.
 *
 * Only the types referenced by sensors_i2c.h are defined here.
 * No actual HAL functionality — tests never call HAL functions.
 */

#ifndef STM32F1XX_HAL_H_STUB
#define STM32F1XX_HAL_H_STUB

#include <stdint.h>

typedef enum {
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

/* Opaque handle — sensors_i2c.h declares pointers to this but test code
 * never dereferences them. */
typedef struct {
    uint32_t dummy;
} I2C_HandleTypeDef;

#endif /* STM32F1XX_HAL_H_STUB */

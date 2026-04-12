/* Minimal HAL stub for host-side unit tests.
 * Only provides types that app headers reference. */
#ifndef STM32F1XX_HAL_H_STUB
#define STM32F1XX_HAL_H_STUB

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

/* I2C handle stub — not used in pure-logic tests but needed by
 * sensors_i2c.h which is included transitively by some app headers. */
typedef struct {
    void *Instance;
} I2C_HandleTypeDef;

#endif

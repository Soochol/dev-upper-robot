/**
 * @file    sensors_i2c.h
 * @brief   I2C sensor drivers — TBP-H70 IR temperature sensor + bus scanner.
 *
 * v1 scope: only the IR temperature sensors (TBP-H70 from Diwell). The
 * ICM42670P (IMU) and ADS1115 (FSR) drivers will be added in Phase 4.
 *
 * All functions in this header expect to be called from a single owning
 * task (T_PID for IR reads). Thread safety on the I2C1 bus is provided
 * by the caller via mtx_i2c1, NOT by this driver. The driver does not
 * touch FreeRTOS primitives so it stays unit-testable on the host with
 * a HAL stub.
 */

#ifndef APP_SENSORS_I2C_H
#define APP_SENSORS_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"   /* HAL_StatusTypeDef + I2C_HandleTypeDef */

/* Note: we use I2C_HandleTypeDef directly rather than a forward
 * declaration. HAL's typedef-on-anonymous-struct pattern prevents
 * forward declaring the type, and stm32f1xx_hal.h is already
 * included by every other file in this project. */

/* ========================================================================
 * Lifecycle
 * ======================================================================== */

/**
 * @brief  Initialize the I2C sensor subsystem.
 *
 * Performs three steps in order:
 *   1. Wait TBP_POWER_ON_DELAY_MS so the sensors finish their internal
 *      reset sequence after VCC. Required by the vendor protocol.
 *   2. Walk the bus from I2C_SCAN_ADDR_MIN to I2C_SCAN_ADDR_MAX with
 *      HAL_I2C_IsDeviceReady and log every device that ACKs. This is
 *      bring-up evidence: the user can compare against the expected
 *      addresses (0x3A, 0x4C for IR; 0x49 for ADS1115; 0x69 for IMU)
 *      and immediately spot a wiring or address-programming mistake.
 *   3. Ping the two configured IR sensor addresses specifically and
 *      log presence/absence of each.
 *
 * Must be called from a task context (not from MX_FREERTOS_Init or any
 * pre-scheduler hook) because step 1 uses vTaskDelay. T_PID is the
 * intended caller, on its first iteration before any read attempt.
 *
 * @param  hi2c   I2C handle (always &hi2c1 in this project).
 * @return HAL_OK if both expected IR sensors responded, HAL_ERROR otherwise.
 *         HAL_ERROR is informational only — the caller may choose to
 *         continue and let individual reads fail later.
 */
HAL_StatusTypeDef sensors_i2c_init(I2C_HandleTypeDef *hi2c);

/* ========================================================================
 * TBP-H70 IR temperature sensor
 * ======================================================================== */

/**
 * @brief  Read the IR target temperature from a single TBP-H70 sensor.
 *
 * Issues the [W cmd Sr R lo hi pec] sequence via HAL_I2C_Mem_Read with
 * MEMADD_SIZE_8BIT and a 3-byte read. The PEC byte is read but NOT
 * verified in v1 — Phase 6 may add CRC-8 verification.
 *
 * The raw 16-bit value (low byte first) has bit 15 as an error flag.
 * If the flag is set the function returns HAL_ERROR and *out_celsius
 * is left unchanged. Otherwise:
 *
 *     out_celsius = (raw & 0x7FFF) * 0.02 - 273.15
 *
 * Caller responsibility:
 *   - Hold mtx_i2c1 across this call.
 *   - Wait at least 100 ms between successive reads of the same sensor
 *     (sensor internal update rate is 10 Hz). Reading more often is
 *     safe but returns stale values.
 *
 * @param  hi2c         I2C handle (always &hi2c1).
 * @param  addr_7b      7-bit slave address (e.g. IR_SENSOR_1_I2C_ADDR_7B).
 *                      Driver shifts left by 1 internally for HAL.
 * @param  out_celsius  output pointer, written only on HAL_OK.
 * @return HAL_OK on a valid reading; HAL_ERROR on bus error, sensor
 *         NACK, timeout, or sensor-reported error flag.
 */
HAL_StatusTypeDef tbp_h70_read_target_c(I2C_HandleTypeDef *hi2c,
                                        uint8_t addr_7b,
                                        float *out_celsius);

/* ========================================================================
 * ICM42670P 6-axis IMU
 * ======================================================================== */

/**
 * @brief  Raw 6-axis data from one IMU burst read (12 bytes, big-endian).
 *         Values are in raw signed 16-bit ADC codes — not yet converted
 *         to g or dps. Feature extraction layer does the conversion.
 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_raw_t;

/**
 * @brief  ICM42670P init phase 1 — soft reset.
 *         I2C writes only, no delay. Caller must wait 100 ms after
 *         this returns before calling icm42670p_configure.
 */
HAL_StatusTypeDef icm42670p_reset(I2C_HandleTypeDef *hi2c);

/**
 * @brief  ICM42670P init phase 2 — WHO_AM_I check + accel/gyro config.
 *         I2C writes only, no delay. Caller must wait 50 ms after
 *         this returns before first data read.
 * @return HAL_OK if WHO_AM_I matches and all config writes succeed.
 */
HAL_StatusTypeDef icm42670p_configure(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Burst-read 12 bytes of accel + gyro data.
 *         big-endian → host-endian conversion is done internally.
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef icm42670p_read(I2C_HandleTypeDef *hi2c, imu_raw_t *out);

/* ========================================================================
 * ADS1115 16-bit ADC (FSR front-end)
 * ======================================================================== */

/**
 * @brief  Configure ADS1115 for continuous conversion on AIN0.
 *         Writes the config register once. Subsequent reads are just
 *         conversion register reads.
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef ads1115_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read the latest conversion result.
 * @param  out_raw  signed 16-bit ADC value (positive = upward pressure).
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef ads1115_read(I2C_HandleTypeDef *hi2c, int16_t *out_raw);

#endif /* APP_SENSORS_I2C_H */

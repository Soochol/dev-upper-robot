/**
 * @file    sensors_i2c.c
 * @brief   TBP-H70 IR temperature sensor driver + I2C bus scanner.
 *
 * Implementation notes
 * --------------------
 *
 * Read sequence (vendor protocol section 4.2.1) decomposes cleanly into
 * one HAL_I2C_Mem_Read call:
 *
 *     [S][slv+W][cmd][A] [Sr][slv+R][A][lo][A][hi][A][pec][N][P]
 *      \_____________/                \_____________/
 *      MEMADDRESS phase                DATA phase (3 bytes)
 *
 *   HAL_I2C_Mem_Read(hi2c, addr<<1, cmd, MEMADD_SIZE_8BIT, buf, 3, timeout);
 *
 * The PEC byte arrives as buf[2]. We currently store it for future PEC
 * verification but do not check it (TBP_PEC_CHECK_ENABLED would gate
 * that path in Phase 6).
 *
 * Raw value is 16-bit little-endian:
 *   raw = buf[0] | (buf[1] << 8)
 * Bit 15 is the error flag. Bits 0..14 hold the data.
 *
 * Conversion uses single-precision float because the Cortex-M3 has no
 * FPU but the per-call cost (~10 µs for one multiply + one subtract via
 * libgcc soft-float) is negligible compared to the I2C transaction
 * itself (~2 ms).
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "i2c.h"             /* hi2c1 — only included from this .c file */
#include "app/config.h"
#include "app/sensors_i2c.h"
#include "app/rtt_log.h"

/* ------------------------------------------------------------------------ */
/* Internal helpers                                                          */
/* ------------------------------------------------------------------------ */

/* Convert a raw 16-bit TBP-H70 reading to Celsius. Returns false if the
 * sensor's internal error flag is set, in which case *out_celsius is
 * unchanged. */
static bool tbp_raw_to_celsius(uint16_t raw, float *out_celsius)
{
    if (raw & TBP_ERROR_FLAG_MASK) {
        return false;
    }
    uint16_t data15 = raw & TBP_DATA_MASK;
    float kelvin = (float)data15 * TBP_RAW_TO_KELVIN_SCALE;
    *out_celsius = kelvin - KELVIN_TO_CELSIUS_OFFSET;
    return true;
}

/* I2C bus scanner. Walks 7-bit address space and logs every device that
 * acknowledges. Used once at init for bring-up evidence — never called
 * again, so the linear sweep cost (~20 ms) is acceptable. */
static void i2c_scan_log(I2C_HandleTypeDef *hi2c)
{
    rtt_log_str("[i2c-scan] start");
    uint32_t found = 0;
    for (uint8_t addr7 = I2C_SCAN_ADDR_MIN; addr7 <= I2C_SCAN_ADDR_MAX; addr7++) {
        /* trials=2 mitigates the occasional false negative caused by
         * a device still busy from its own internal cycle. timeout=2 ms
         * keeps the full sweep under ~250 ms worst case. */
        HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(hi2c,
                                                     (uint16_t)(addr7 << 1),
                                                     2, 2);
        if (st == HAL_OK) {
            rtt_log_kv_hex("[i2c-scan] found 7b=", (uint32_t)addr7);
            found++;
        }
    }
    rtt_log_kv("[i2c-scan] done count=", found);
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef sensors_i2c_init(I2C_HandleTypeDef *hi2c)
{
    /* Step 1: respect the TBP-H70 power-on delay before any I2C traffic.
     * The protocol spec says >= 200 ms after VCC. We measure from now
     * because main.c starts the I2C peripheral well before the scheduler
     * launches; by the time T_PID calls this function the delay has
     * usually already elapsed, but we sleep the full 200 ms anyway as
     * a safety margin. */
    vTaskDelay(pdMS_TO_TICKS(TBP_POWER_ON_DELAY_MS));

    /* Step 2: full bus scan for bring-up visibility. */
    i2c_scan_log(hi2c);

    /* Step 3: directly probe each configured IR sensor address. The bus
     * scan above already covers these, but a dedicated check produces
     * a clearer pass/fail line in the log and lets the caller act on
     * the return value. */
    HAL_StatusTypeDef st1 = HAL_I2C_IsDeviceReady(hi2c,
                                                  IR_SENSOR_1_I2C_ADDR_7B << 1,
                                                  3, 5);
    HAL_StatusTypeDef st2 = HAL_I2C_IsDeviceReady(hi2c,
                                                  IR_SENSOR_2_I2C_ADDR_7B << 1,
                                                  3, 5);
    rtt_log_kv_hex("[ir] sensor1 7b=", (uint32_t)IR_SENSOR_1_I2C_ADDR_7B);
    rtt_log_kv("[ir] sensor1 ok=", (uint32_t)(st1 == HAL_OK ? 1 : 0));
    rtt_log_kv_hex("[ir] sensor2 7b=", (uint32_t)IR_SENSOR_2_I2C_ADDR_7B);
    rtt_log_kv("[ir] sensor2 ok=", (uint32_t)(st2 == HAL_OK ? 1 : 0));

    return (st1 == HAL_OK && st2 == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef tbp_h70_read_target_c(I2C_HandleTypeDef *hi2c,
                                        uint8_t addr_7b,
                                        float *out_celsius)
{
    /* HAL_I2C_Mem_Read pattern matches the TBP-H70 read sequence exactly:
     *   START → addr+W → cmd → repeated START → addr+R → 3 bytes → STOP */
    uint8_t buf[3] = {0};
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c,
                                            (uint16_t)(addr_7b << 1),
                                            TBP_CMD_TARGET_TEMP,
                                            I2C_MEMADD_SIZE_8BIT,
                                            buf, sizeof(buf),
                                            TBP_HAL_TIMEOUT_MS);
    if (st != HAL_OK) {
        return st;
    }

    /* buf[0] = low byte, buf[1] = high byte, buf[2] = PEC (not verified). */
    uint16_t raw = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);

    if (!tbp_raw_to_celsius(raw, out_celsius)) {
        /* Sensor reported its error flag — treat as a soft failure so
         * the caller can decide whether to retry or escalate. */
        return HAL_ERROR;
    }
    return HAL_OK;
}

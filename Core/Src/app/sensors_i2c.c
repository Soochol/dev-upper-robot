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

/* ====================================================================
 * ICM42670P IMU
 * ==================================================================== */

/* Helper: write one byte to an ICM42670P BANK0 register. */
static HAL_StatusTypeDef imu_write_reg(I2C_HandleTypeDef *hi2c,
                                       uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, IMU_I2C_ADDR_7B << 1,
                             reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, TBP_HAL_TIMEOUT_MS);
}

/* Helper: read one byte from an ICM42670P BANK0 register. */
static HAL_StatusTypeDef imu_read_reg(I2C_HandleTypeDef *hi2c,
                                      uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(hi2c, IMU_I2C_ADDR_7B << 1,
                            reg, I2C_MEMADD_SIZE_8BIT,
                            val, 1, TBP_HAL_TIMEOUT_MS);
}

HAL_StatusTypeDef icm42670p_reset(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef st;
    uint8_t val;

    /* Step 0: I2C drive config. */
    st = imu_write_reg(hi2c, ICM42670P_REG_DRIVE_CONFIG2,
                       ICM42670P_DRIVE_CONFIG2_VAL);
    if (st != HAL_OK) {
        rtt_log_str("[imu] reset: DRIVE_CONFIG2 failed");
        return st;
    }

    /* Disable I3C (clear bits 4:3 in INTF_CONFIG1). */
    st = imu_read_reg(hi2c, ICM42670P_REG_INTF_CONFIG1, &val);
    if (st == HAL_OK) {
        val &= ~0x18u;
        imu_write_reg(hi2c, ICM42670P_REG_INTF_CONFIG1, val);
    }

    /* Step 1: Soft reset. Clear bank selectors first. */
    imu_write_reg(hi2c, ICM42670P_REG_BLK_SEL_R, 0x00);
    imu_write_reg(hi2c, ICM42670P_REG_BLK_SEL_W, 0x00);
    st = imu_write_reg(hi2c, ICM42670P_REG_SIGNAL_PATH_RESET,
                       ICM42670P_SOFT_RESET_BIT);
    if (st != HAL_OK) {
        rtt_log_str("[imu] reset: soft reset write failed");
        return st;
    }

    /* NO vTaskDelay here — caller must wait ICM42670P_RESET_DELAY_MS
     * (100 ms) with the I2C mutex released before calling configure(). */
    rtt_log_str("[imu] reset: done (caller waits 100ms)");
    return HAL_OK;
}

HAL_StatusTypeDef icm42670p_configure(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef st;
    uint8_t val;

    /* Clear reset-done interrupt. */
    imu_read_reg(hi2c, ICM42670P_REG_INT_STATUS, &val);

    /* Step 2: WHO_AM_I check. */
    st = imu_read_reg(hi2c, ICM42670P_REG_WHO_AM_I, &val);
    if (st != HAL_OK) {
        rtt_log_str("[imu] cfg: WHO_AM_I read failed");
        return st;
    }
    if (val != ICM42670P_WHO_AM_I_VALUE) {
        rtt_log_kv_hex("[imu] cfg: bad WHO_AM_I=", val);
        return HAL_ERROR;
    }
    rtt_log_kv_hex("[imu] cfg: WHO_AM_I ok=", val);

    /* Step 4: Accel — ±2g, 100 Hz ODR, 25 Hz LPF. */
    imu_write_reg(hi2c, ICM42670P_REG_ACCEL_CONFIG0,
                  ICM42670P_ACCEL_CONFIG0_VAL);
    imu_write_reg(hi2c, ICM42670P_REG_ACCEL_CONFIG1,
                  ICM42670P_ACCEL_CONFIG1_VAL);

    /* Step 5: Gyro — ±2000 dps, 100 Hz ODR, 73 Hz LPF. */
    imu_write_reg(hi2c, ICM42670P_REG_GYRO_CONFIG0,
                  ICM42670P_GYRO_CONFIG0_VAL);
    imu_write_reg(hi2c, ICM42670P_REG_GYRO_CONFIG1,
                  ICM42670P_GYRO_CONFIG1_VAL);

    /* Step 6: Power on — accel + gyro Low Noise mode. */
    imu_write_reg(hi2c, ICM42670P_REG_PWR_MGMT0,
                  ICM42670P_PWR_MGMT0_ON);

    /* NO vTaskDelay here — caller waits ICM42670P_STARTUP_DELAY_MS
     * (50 ms) with the I2C mutex released before first read. */
    rtt_log_str("[imu] cfg: +-2g 100Hz, +-2000dps 100Hz, LN (caller waits 50ms)");
    return HAL_OK;
}

HAL_StatusTypeDef icm42670p_read(I2C_HandleTypeDef *hi2c, imu_raw_t *out)
{
    uint8_t buf[ICM42670P_DATA_BURST_LEN];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c,
                                            IMU_I2C_ADDR_7B << 1,
                                            ICM42670P_REG_ACCEL_DATA_X1,
                                            I2C_MEMADD_SIZE_8BIT,
                                            buf, ICM42670P_DATA_BURST_LEN,
                                            TBP_HAL_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    /* ICM42670P register data is big-endian: high byte first. */
    out->accel_x = (int16_t)((uint16_t)buf[0]  << 8 | buf[1]);
    out->accel_y = (int16_t)((uint16_t)buf[2]  << 8 | buf[3]);
    out->accel_z = (int16_t)((uint16_t)buf[4]  << 8 | buf[5]);
    out->gyro_x  = (int16_t)((uint16_t)buf[6]  << 8 | buf[7]);
    out->gyro_y  = (int16_t)((uint16_t)buf[8]  << 8 | buf[9]);
    out->gyro_z  = (int16_t)((uint16_t)buf[10] << 8 | buf[11]);
    return HAL_OK;
}

/* ====================================================================
 * ADS1115 ADC (FSR front-end)
 * ==================================================================== */

HAL_StatusTypeDef ads1115_init(I2C_HandleTypeDef *hi2c)
{
    /* Write config register to trigger the first single-shot conversion.
     * Subsequent reads will each trigger a new conversion via OS=1.
     * Config value 0xC283 from the FSR hardware reference guide:
     * AIN0 single-ended, ±4.096V PGA, single-shot, 128 SPS. */
    uint8_t cfg[2] = {
        (uint8_t)(ADS1115_CONFIG_SS >> 8),
        (uint8_t)(ADS1115_CONFIG_SS & 0xFF),
    };
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(hi2c,
                                             ADS1115_I2C_ADDR_7B << 1,
                                             ADS1115_REG_CONFIG,
                                             I2C_MEMADD_SIZE_8BIT,
                                             cfg, 2,
                                             TBP_HAL_TIMEOUT_MS);
    if (st == HAL_OK) {
        rtt_log_str("[fsr] init: ADS1115 single-shot mode (0xC283)");
    } else {
        rtt_log_str("[fsr] init: ADS1115 config write failed");
    }
    return st;
}

HAL_StatusTypeDef ads1115_read(I2C_HandleTypeDef *hi2c, int16_t *out_raw)
{
    /* Single-shot pattern: write config (OS=1 triggers new conversion),
     * then read the conversion register which returns the PREVIOUS
     * completed result. The new conversion completes ~8 ms later and
     * will be returned by the next call to this function.
     *
     * This "trigger + read-previous" approach avoids an 8 ms blocking
     * wait inside the I2C mutex. The first read after init returns a
     * stale value; from the second onward each result is 50 ms old
     * (one T_ML period), which is fine for trigger threshold logic. */
    uint8_t cfg[2] = {
        (uint8_t)(ADS1115_CONFIG_SS >> 8),
        (uint8_t)(ADS1115_CONFIG_SS & 0xFF),
    };
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(hi2c,
                                             ADS1115_I2C_ADDR_7B << 1,
                                             ADS1115_REG_CONFIG,
                                             I2C_MEMADD_SIZE_8BIT,
                                             cfg, 2,
                                             TBP_HAL_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    uint8_t buf[2];
    st = HAL_I2C_Mem_Read(hi2c,
                          ADS1115_I2C_ADDR_7B << 1,
                          ADS1115_REG_CONVERSION,
                          I2C_MEMADD_SIZE_8BIT,
                          buf, 2,
                          TBP_HAL_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    /* ADS1115 conversion register is big-endian. */
    *out_raw = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
    return HAL_OK;
}

/* ====================================================================
 * I2C bus recovery
 * ==================================================================== */

void i2c1_bus_recover(I2C_HandleTypeDef *hi2c)
{
    /* Must be called with mtx_i2c1 held [C2]. */

    rtt_log_str("[i2c] bus recovery start");

    /* Step 1: tear down the HAL I2C state machine. This clears any
     * HAL_BUSY / HAL_ERROR latched state in the handle. */
    HAL_I2C_DeInit(hi2c);

    /* Step 2: reconfigure SCL and SDA as GPIO open-drain outputs.
     * I2C1 remapped to PB8 (SCL) / PB9 (SDA) on this board. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = I2C1_SCL_IR_Pin | I2C1_SDA_IR_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Ensure SCL starts HIGH, SDA starts HIGH (idle bus). */
    HAL_GPIO_WritePin(GPIOB, I2C1_SCL_IR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, I2C1_SDA_IR_Pin, GPIO_PIN_SET);

    /* Step 3: toggle SCL 9 times. If a slave is holding SDA low
     * mid-byte, this clocks out the remaining bits so the slave
     * releases SDA. Each half-cycle is ~5 µs (200 kHz effective). */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, I2C1_SCL_IR_Pin, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 20; d++) { __asm__ volatile ("nop"); }
        HAL_GPIO_WritePin(GPIOB, I2C1_SCL_IR_Pin, GPIO_PIN_SET);
        for (volatile int d = 0; d < 20; d++) { __asm__ volatile ("nop"); }

        /* If SDA is already HIGH, the slave released — we can stop early. */
        if (HAL_GPIO_ReadPin(GPIOB, I2C1_SDA_IR_Pin) == GPIO_PIN_SET) {
            break;
        }
    }

    /* Step 4: generate a proper STOP condition.
     * Sequence: SCL LOW → SDA LOW → SCL HIGH → SDA HIGH (= STOP).
     * [C5] Must pull SCL LOW first to avoid generating a spurious START
     * (SDA falling while SCL is HIGH). */
    HAL_GPIO_WritePin(GPIOB, I2C1_SCL_IR_Pin, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 20; d++) { __asm__ volatile ("nop"); }
    HAL_GPIO_WritePin(GPIOB, I2C1_SDA_IR_Pin, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 20; d++) { __asm__ volatile ("nop"); }
    HAL_GPIO_WritePin(GPIOB, I2C1_SCL_IR_Pin, GPIO_PIN_SET);
    for (volatile int d = 0; d < 20; d++) { __asm__ volatile ("nop"); }
    HAL_GPIO_WritePin(GPIOB, I2C1_SDA_IR_Pin, GPIO_PIN_SET);

    /* Step 5: re-initialize the HAL I2C peripheral.
     * HAL_I2C_Init → MspInit restores GPIO AF + AFIO remap + clock
     * automatically. No manual GPIO reconfiguration needed [C1]. */
    HAL_I2C_Init(hi2c);

    rtt_log_str("[i2c] bus recovery done");
}

/* ====================================================================
 * TBP-H70 IR temperature
 * ==================================================================== */

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

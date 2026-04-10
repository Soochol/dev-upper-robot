/**
 * @file    sd_logger.h
 * @brief   SD card CSV logger for ML training data collection.
 *
 * Writes raw sensor snapshots to a CSV file on the SD card via FatFS.
 * Enabled only when DATA_COLLECT_MODE is 1 in config.h.
 *
 * Usage from T_ML:
 *   1. Call sd_logger_init() once at task start.
 *   2. Call sd_logger_write_row() each tick with raw sensor values.
 *   3. Call sd_logger_flush() periodically or at shutdown.
 *
 * Write strategy: rows accumulate in a 512-byte RAM buffer (one SDIO
 * block). When the buffer is full, sd_logger_write_row() triggers an
 * f_write() + f_sync(). This batches writes to minimize FatFS overhead.
 *
 * No FreeRTOS dependency beyond the tick counter for timestamps. The
 * caller (T_ML) is responsible for holding mtx_i2c1 during sensor
 * reads; this module only touches the SDIO peripheral via FatFS.
 */

#ifndef APP_SD_LOGGER_H
#define APP_SD_LOGGER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  Mount SD card and open/create the CSV log file.
 *
 * File naming: "ml_NNN.csv" where NNN increments to avoid overwriting
 * previous sessions. Scans existing files to find the next number.
 * Writes the CSV header row on a new file.
 *
 * @return true on success, false if SD mount or file open fails.
 *         On failure, subsequent write_row calls are no-ops.
 */
bool sd_logger_init(void);

/**
 * @brief  Append one CSV row of raw sensor data to the write buffer.
 *
 * If the buffer is full after formatting, triggers an f_write()+f_sync().
 * The flush costs ~2 ms (DMA-based SDIO write).
 *
 * @param  timestamp_ms  system tick in milliseconds
 * @param  fsr_raw       ADS1115 raw reading (signed)
 * @param  ax,ay,az      ICM42670P accel raw (signed, +-2g scale)
 * @param  gx,gy,gz      ICM42670P gyro raw (signed, +-2000dps scale)
 * @param  tilt_x        filtered tilt X in degrees (relative to boot)
 * @param  tilt_y        filtered tilt Y in degrees
 */
void sd_logger_write_row(uint32_t timestamp_ms,
                         int16_t fsr_raw,
                         int16_t ax, int16_t ay, int16_t az,
                         int16_t gx, int16_t gy, int16_t gz,
                         float tilt_x, float tilt_y);

/**
 * @brief  Force-flush the write buffer to SD and sync.
 *         Call before removing the SD card or at shutdown.
 */
void sd_logger_flush(void);

#endif /* APP_SD_LOGGER_H */

/**
 * @file    sd_logger.h
 * @brief   SD card CSV logger for ML training data collection.
 *
 * Button-controlled recording with auto-incrementing filenames:
 *   ml_000.csv, ml_001.csv, ...
 *
 * Lifecycle:
 *   1. sd_logger_init()       — mount SD, scan for next file number
 *   2. sd_logger_start()      — open new file, begin recording
 *   3. sd_logger_write_row()  — append sensor data (no-op if not recording)
 *   4. sd_logger_stop()       — flush + close file
 *   5. sd_logger_dump_rtt()   — stream last file over RTT to host PC
 *
 * Step 2-5 can repeat for multiple sessions without remounting.
 */

#ifndef APP_SD_LOGGER_H
#define APP_SD_LOGGER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  Mount SD card and scan for the next available file number.
 *
 * Speeds up SDIO clock after mount. Does NOT open a file — call
 * sd_logger_start() to begin recording.
 *
 * @return true on success, false if mount fails.
 */
bool sd_logger_init(void);

/**
 * @brief  Open a new CSV file (ml_NNN.csv) and begin recording.
 *
 * Auto-increments the file number. Writes CSV header.
 * No-op if already recording or if init failed.
 *
 * @return true on success, false if file open fails.
 */
bool sd_logger_start(void);

/**
 * @brief  Stop recording — flush buffer, close file.
 *
 * After this call, write_row() becomes a no-op until the next start().
 */
void sd_logger_stop(void);

/**
 * @brief  True if currently recording (between start and stop).
 */
bool sd_logger_is_recording(void);

/**
 * @brief  Append one CSV row of raw sensor data to the write buffer.
 *
 * No-op if not recording. If the buffer is full, triggers f_write+f_sync.
 */
void sd_logger_write_row(uint32_t timestamp_ms,
                         int16_t fsr_raw,
                         int16_t ax, int16_t ay, int16_t az,
                         int16_t gx, int16_t gy, int16_t gz,
                         float tilt_x, float tilt_y);

/**
 * @brief  Force-flush the write buffer to SD and sync.
 */
void sd_logger_flush(void);

/**
 * @brief  True if any f_write call failed during the current recording.
 *
 * Latched until the next sd_logger_start() (which resets the counter).
 * T_PID polls this to switch the LED from steady "recording" blink to
 * a fast warning blink when the SD card stops accepting writes — without
 * this, recording silently corrupts and the user only finds out post-hoc.
 */
bool sd_logger_has_write_error(void);

/**
 * @brief  Stream the last recorded file over RTT channel 0.
 *
 * Reads the file in 256-byte chunks and writes to SEGGER_RTT.
 * Framing:
 *   [sd:dump:start:<filename>]\n
 *   ... CSV data ...
 *   [sd:dump:end]\n
 *
 * Handles RTT backpressure with vTaskDelay(1) when the buffer is full.
 * Call from a FreeRTOS task context (not ISR).
 */
void sd_logger_dump_rtt(void);

#endif /* APP_SD_LOGGER_H */

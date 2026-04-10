/**
 * @file    sd_logger.c
 * @brief   SD card CSV logger for ML training data collection.
 *
 * Writes one CSV row per T_ML tick (20 Hz) containing raw sensor values
 * and derived tilt angles. Data is buffered in a 512-byte RAM buffer
 * and flushed to the SD card when the buffer is full.
 *
 * File naming: ml_000.csv, ml_001.csv, ... Auto-increments on each
 * power cycle / init call to avoid overwriting previous sessions.
 *
 * CSV format (header + data):
 *   ms,fsr,ax,ay,az,gx,gy,gz,tx,ty
 *   12340,1523,1024,-234,16100,12,-5,3,1520,-210
 *
 * Tilt values are stored as fixed-point (×100) to avoid float formatting
 * on the MCU. The PC-side Python script divides by 100 to recover degrees.
 */

#include "app/sd_logger.h"
#include "app/config.h"
#include "app/rtt_log.h"
#include "fatfs.h"
#include "bsp_driver_sd.h"

#include <stdio.h>
#include <string.h>

/* Write buffer — one SDIO block for aligned DMA writes. */
static char s_buf[SD_LOG_BUF_SIZE];
static uint16_t s_buf_pos;
static FIL s_file;
static bool s_ready;

/* Debug trace — readable via GDB without RTT.
 * 0=not started, 1=about to mount, 2=mount returned, 3=mount ok,
 * 4=clock set, 5=file open ok, 6=ready.
 * 20=about to f_open, 40=f_open retry. Negative = error. */
volatile int32_t sd_dbg_step = 0;
volatile uint32_t sd_dbg_tick = 0;  /* set by t_ml each tick — proves task runs */
/* g_hf[13] is defined in stm32f1xx_it.c (always compiled). */

/* ------------------------------------------------------------------ */
/* Internal helpers                                                     */
/* ------------------------------------------------------------------ */

/* Flush the buffer to SD card. */
static void flush_buf(void)
{
    if (!s_ready || s_buf_pos == 0) return;

    UINT bw;
    FRESULT fr = f_write(&s_file, s_buf, s_buf_pos, &bw);
    if (fr != FR_OK || bw != s_buf_pos) {
        rtt_log_str("[sd] write err");
    }
    f_sync(&s_file);
    s_buf_pos = 0;
}

/* Append a string to the buffer. Flush if full. */
static void buf_append(const char *str, uint16_t len)
{
    while (len > 0) {
        uint16_t space = SD_LOG_BUF_SIZE - s_buf_pos;
        uint16_t chunk = (len < space) ? len : space;
        memcpy(&s_buf[s_buf_pos], str, chunk);
        s_buf_pos += chunk;
        str += chunk;
        len -= chunk;
        if (s_buf_pos >= SD_LOG_BUF_SIZE) {
            flush_buf();
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

bool sd_logger_init(void)
{
    s_ready = false;
    s_buf_pos = 0;

    sd_dbg_step = 1;  /* about to f_mount */
    /* Mount the SD card filesystem. */
    FRESULT fr = f_mount(&SDFatFS, SDPath, 1);
    sd_dbg_step = 3;  /* f_mount returned */
    if (fr != FR_OK) {
        sd_dbg_step = -(int32_t)fr;  /* negative FRESULT = mount fail */
        rtt_log_kv("[sd] mount fail fr=", (uint32_t)fr);
        return false;
    }
    sd_dbg_step = 4;  /* mount ok */
    rtt_log_str("[sd] mounted");

    /* Speed up SDIO clock now that card init is done.
     * Init ran at ClockDiv=248 (288KHz) per SD spec.
     * Now set ClockDiv=10 → HCLK/(10+2) = 72/12 = 6MHz.
     * Modify CLKCR register directly — safer than SDIO_Init which
     * resets other CLKCR bits. */
    {
        uint32_t clkcr = SDIO->CLKCR;
        clkcr &= ~0xFFu;   /* clear CLKDIV[7:0] */
        clkcr |= 10u;      /* ClockDiv=10 → 6MHz */
        SDIO->CLKCR = clkcr;
        sd_dbg_step = 5;
        rtt_log_str("[sd] clk→6MHz");
    }

    /* Fixed filename — overwrites on each boot. User renames the
     * file on PC before re-collecting. */
    static const char fname[] = "ml_data.csv";

    sd_dbg_step = 20;  /* about to f_open session file */
    fr = f_open(&s_file, fname, FA_WRITE | FA_CREATE_ALWAYS);
    sd_dbg_step = 20 + (int32_t)fr;  /* 20=ok, 21+=FRESULT */
    if (fr != FR_OK) {
        extern SD_HandleTypeDef hsd;
        rtt_log_kv("[sd] open fail fr=", (uint32_t)fr);
        rtt_log_kv("[sd] hsd.err=", hsd.ErrorCode);
        rtt_log_kv("[sd] hsd.state=", (uint32_t)hsd.State);

        /* Retry once: some cards need a warm-up write after mount. */
        fr = f_open(&s_file, fname, FA_WRITE | FA_CREATE_ALWAYS);
        sd_dbg_step = 40 + (int32_t)fr;  /* 40=retry ok, 41+=retry fail */
        if (fr != FR_OK) {
            rtt_log_kv("[sd] retry fail fr=", (uint32_t)fr);
            f_mount(NULL, SDPath, 0);
            return false;
        }
        rtt_log_str("[sd] retry OK");
    }
    sd_dbg_step = 5;  /* session file open ok */
    rtt_log_str("[sd] file=ml_data.csv");

    /* Write CSV header. */
    static const char hdr[] = "ms,fsr,ax,ay,az,gx,gy,gz,tx100,ty100\n";
    buf_append(hdr, (uint16_t)(sizeof(hdr) - 1));

    sd_dbg_step = 6;  /* fully ready */
    s_ready = true;
    return true;
}

void sd_logger_write_row(uint32_t timestamp_ms,
                         int16_t fsr_raw,
                         int16_t ax, int16_t ay, int16_t az,
                         int16_t gx, int16_t gy, int16_t gz,
                         float tilt_x, float tilt_y)
{
    if (!s_ready) return;

    /* Format as CSV. Tilt is stored as fixed-point ×100 to avoid
     * float-to-string on the MCU (saves ~2KB of formatting code). */
    int32_t tx100 = (int32_t)(tilt_x * 100.0f);
    int32_t ty100 = (int32_t)(tilt_y * 100.0f);

    char row[80];
    int n = snprintf(row, sizeof(row),
                     "%lu,%d,%d,%d,%d,%d,%d,%d,%ld,%ld\n",
                     (unsigned long)timestamp_ms,
                     (int)fsr_raw,
                     (int)ax, (int)ay, (int)az,
                     (int)gx, (int)gy, (int)gz,
                     (long)tx100, (long)ty100);
    if (n > 0 && n < (int)sizeof(row)) {
        buf_append(row, (uint16_t)n);
    }
}

void sd_logger_flush(void)
{
    flush_buf();
}

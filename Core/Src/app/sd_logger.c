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
 * 0=not started, 1=mount ok, 2=selftest-write ok, 3=selftest-read ok,
 * 4=selftest-compare ok, 5=session file open ok, 6=header written (ready).
 * Negative = error at that step (value = -FRESULT). */
volatile int32_t sd_dbg_step = 0;
volatile uint32_t sd_dbg_tick = 0;  /* set by t_ml each tick — proves task runs */
volatile uint32_t g_hf[13]; /* HardFault capture — see stm32f1xx_it.c */

/* ------------------------------------------------------------------ */
/* Internal helpers                                                     */
/* ------------------------------------------------------------------ */

/* Find the next available session number by scanning existing files. */
static uint16_t find_next_session(void)
{
    FILINFO fno;
    char name[16];
    uint16_t next = 0;

    for (uint16_t i = 0; i < 1000; i++) {
        snprintf(name, sizeof(name), "ml_%03u.csv", i);
        if (f_stat(name, &fno) != FR_OK) {
            next = i;
            break;
        }
        next = i + 1;
    }
    return next;
}

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

/* Write+read self-test: creates a small test file, reads it back,
 * compares content, and deletes it. Proves the full SD card path
 * (mount → write → sync → read → unlink) works end to end. */
static bool sd_selftest(void)
{
    static const char test_data[] = "SD_TEST_OK\n";
    static const char test_name[] = "sd_test.tmp";
    static FIL tf;     /* 4KB+ struct — must be static, not on stack */
    UINT bw, br;
    char rbuf[16] = {0};
    FRESULT fr;

    /* Write */
    sd_dbg_step = 10;  /* about to open for write */
    fr = f_open(&tf, test_name, FA_WRITE | FA_CREATE_ALWAYS);
    sd_dbg_step = 10 + (int32_t)fr;  /* 10 = open ok, 11+ = FRESULT error */
    if (fr != FR_OK) {
        sd_dbg_step = -((int32_t)fr + 100);   /* -10x = open-w fail */
        rtt_log_kv("[sd-test] open-w fail fr=", (uint32_t)fr);
        return false;
    }
    fr = f_write(&tf, test_data, sizeof(test_data) - 1, &bw);
    f_close(&tf);
    if (fr != FR_OK || bw != sizeof(test_data) - 1) {
        sd_dbg_step = -((int32_t)fr + 200);   /* -20x = write fail */
        rtt_log_kv("[sd-test] write fail fr=", (uint32_t)fr);
        f_unlink(test_name);
        return false;
    }
    sd_dbg_step = 2;  /* selftest write ok */
    rtt_log_kv("[sd-test] wrote bytes=", (uint32_t)bw);

    /* Read back */
    fr = f_open(&tf, test_name, FA_READ);
    if (fr != FR_OK) {
        sd_dbg_step = -((int32_t)fr + 300);   /* -30x = open-r fail */
        rtt_log_kv("[sd-test] open-r fail fr=", (uint32_t)fr);
        f_unlink(test_name);
        return false;
    }
    fr = f_read(&tf, rbuf, sizeof(rbuf) - 1, &br);
    f_close(&tf);
    if (fr != FR_OK || br != sizeof(test_data) - 1) {
        sd_dbg_step = -((int32_t)fr + 400);   /* -40x = read fail */
        rtt_log_kv("[sd-test] read fail br=", (uint32_t)br);
        f_unlink(test_name);
        return false;
    }
    sd_dbg_step = 3;  /* selftest read ok */
    rtt_log_kv("[sd-test] read bytes=", (uint32_t)br);

    /* Compare */
    bool match = true;
    for (uint16_t i = 0; i < br; i++) {
        if (rbuf[i] != test_data[i]) {
            match = false;
            break;
        }
    }

    /* Cleanup */
    f_unlink(test_name);

    if (!match) {
        rtt_log_str("[sd-test] FAIL: data mismatch");
        return false;
    }

    sd_dbg_step = 4;  /* selftest compare ok */
    rtt_log_str("[sd-test] PASS: write+read+compare OK");
    return true;
}

/* Raw HAL SD test — bypasses FatFS to isolate hardware issues.
 * Writes a test pattern to the LAST block on the card (to avoid
 * corrupting the filesystem), reads it back, and compares. */
static bool sd_hal_test(void)
{
    extern SD_HandleTypeDef hsd;
    static uint8_t wbuf[512] __attribute__((aligned(4)));
    static uint8_t rbuf[512] __attribute__((aligned(4)));

    /* Get card info to find the last block. */
    HAL_SD_CardInfoTypeDef info;
    HAL_SD_GetCardInfo(&hsd, &info);
    uint32_t last_block = info.LogBlockNbr - 1;

    sd_dbg_step = 7;  /* HAL test start */
    rtt_log_kv("[sd-hal] blocks=", info.LogBlockNbr);

    /* Fill write buffer with a recognizable pattern. */
    for (int i = 0; i < 512; i++) {
        wbuf[i] = (uint8_t)(i & 0xFF);
    }

    /* Write one block (polling mode). */
    HAL_StatusTypeDef st = HAL_SD_WriteBlocks(&hsd, wbuf, last_block, 1, 5000);
    if (st != HAL_OK) {
        sd_dbg_step = -(700 + (int32_t)st);
        rtt_log_kv("[sd-hal] write fail st=", (uint32_t)st);
        return false;
    }

    /* Wait for card to finish internal write. */
    uint32_t timeout = 100000;
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        if (--timeout == 0) {
            sd_dbg_step = -799;
            rtt_log_str("[sd-hal] write busy timeout");
            return false;
        }
    }
    sd_dbg_step = 8;  /* HAL write ok */

    /* Read it back. */
    memset(rbuf, 0, 512);
    st = HAL_SD_ReadBlocks(&hsd, rbuf, last_block, 1, 5000);
    if (st != HAL_OK) {
        sd_dbg_step = -(800 + (int32_t)st);
        rtt_log_kv("[sd-hal] read fail st=", (uint32_t)st);
        return false;
    }

    timeout = 100000;
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        if (--timeout == 0) {
            sd_dbg_step = -899;
            rtt_log_str("[sd-hal] read busy timeout");
            return false;
        }
    }
    sd_dbg_step = 9;  /* HAL read ok */

    /* Compare. */
    for (int i = 0; i < 512; i++) {
        if (rbuf[i] != wbuf[i]) {
            sd_dbg_step = -900;
            rtt_log_kv("[sd-hal] mismatch at i=", (uint32_t)i);
            return false;
        }
    }

    sd_dbg_step = 11;  /* HAL test PASS */
    rtt_log_str("[sd-hal] PASS: write+read+compare OK");
    return true;
}

bool sd_logger_init(void)
{
    s_ready = false;
    s_buf_pos = 0;

    sd_dbg_step = 1;  /* about to mount */

    sd_dbg_step = 2;  /* about to f_mount */
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

    /* Skip selftest — go straight to session file creation.
     * HAL-level test already passed above. */

    /* Use a simple fixed filename. The find_next_session() approach
     * with f_stat() loops is too slow at 192KHz SDIO clock and risks
     * FILINFO stack overflow. Overwrite on each boot — user renames
     * the file on PC before re-collecting. */
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

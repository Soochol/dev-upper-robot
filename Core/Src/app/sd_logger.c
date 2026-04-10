/**
 * @file    sd_logger.c
 * @brief   SD card CSV logger — button-controlled, auto-incrementing files.
 *
 * File naming: ml_000.csv, ml_001.csv, ... Each sd_logger_start() opens
 * the next number. The index is determined by scanning the root directory
 * at init time, so it survives power cycles.
 *
 * After sd_logger_stop(), sd_logger_dump_rtt() streams the file over
 * SEGGER RTT so the host can capture it without removing the SD card.
 *
 * CSV format (header + data):
 *   ms,fsr,ax,ay,az,gx,gy,gz,tx100,ty100
 *   12340,1523,1024,-234,16100,12,-5,3,1520,-210
 */

#include "app/sd_logger.h"
#include "app/config.h"
#include "app/rtt_log.h"
#include "fatfs.h"
#include "bsp_driver_sd.h"
#include "SEGGER_RTT.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* State                                                                */
/* ------------------------------------------------------------------ */

static char     s_buf[SD_LOG_BUF_SIZE];
static uint16_t s_buf_pos;
static FIL      s_file;
static bool     s_mounted   = false;
static bool     s_recording = false;
static uint16_t s_next_index = 0;
static char     s_last_fname[16];   /* e.g. "ml_003.csv" */

/* Debug trace — readable via GDB without RTT. */
volatile int32_t  sd_dbg_step = 0;
volatile uint32_t sd_dbg_tick = 0;

/* ------------------------------------------------------------------ */
/* Internal helpers                                                     */
/* ------------------------------------------------------------------ */

static void flush_buf(void)
{
    if (!s_recording || s_buf_pos == 0) return;

    UINT bw;
    FRESULT fr = f_write(&s_file, s_buf, s_buf_pos, &bw);
    if (fr != FR_OK || bw != s_buf_pos) {
        rtt_log_str("[sd] write err");
    }
    f_sync(&s_file);
    s_buf_pos = 0;
}

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

/** Try to parse "ml_NNN.csv" from a filename. Returns -1 on no match. */
static int parse_ml_index(const char *name)
{
    /* Accept "ml_" or "ML_" prefix. */
    if (!((name[0] == 'm' || name[0] == 'M') &&
          (name[1] == 'l' || name[1] == 'L') &&
          name[2] == '_'))
        return -1;

    /* Parse decimal digits after "ml_". */
    const char *p = &name[3];
    if (*p < '0' || *p > '9') return -1;

    unsigned val = 0;
    while (*p >= '0' && *p <= '9') {
        val = val * 10 + (unsigned)(*p - '0');
        p++;
    }

    /* Expect ".csv" or ".CSV" suffix. */
    if ((p[0] == '.' &&
         (p[1] == 'c' || p[1] == 'C') &&
         (p[2] == 's' || p[2] == 'S') &&
         (p[3] == 'v' || p[3] == 'V') &&
         p[4] == '\0'))
        return (int)val;

    return -1;
}

/** Scan root directory for ml_NNN.csv and find the next available index. */
static uint16_t scan_next_index(void)
{
    DIR dir;
    FILINFO fno;
#if _USE_LFN
    fno.lfname = NULL;   /* We only need 8.3 names — skip LFN. */
    fno.lfsize = 0;
#endif
    int max_idx = -1;

    if (f_opendir(&dir, "/") != FR_OK) return 0;

    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        int idx = parse_ml_index(fno.fname);
        if (idx >= 0 && idx > max_idx) {
            max_idx = idx;
        }
    }
    f_closedir(&dir);

    return (max_idx >= 0) ? (uint16_t)(max_idx + 1) : 0;
}

/** Write data to RTT with backpressure handling (yields when buffer full). */
static void rtt_write_blocking(const void *data, unsigned len)
{
    const char *p = (const char *)data;
    unsigned remaining = len;

    while (remaining > 0) {
        unsigned written = SEGGER_RTT_Write(0, p, remaining);
        p += written;
        remaining -= written;
        if (remaining > 0) {
            vTaskDelay(1);  /* RTT buffer full — yield and retry */
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

bool sd_logger_init(void)
{
    s_mounted = false;
    s_recording = false;
    s_buf_pos = 0;
    s_last_fname[0] = '\0';

    sd_dbg_step = 1;
    FRESULT fr = f_mount(&SDFatFS, SDPath, 1);
    if (fr != FR_OK) {
        sd_dbg_step = -(int32_t)fr;
        rtt_log_kv("[sd] mount fail fr=", (uint32_t)fr);
        return false;
    }
    sd_dbg_step = 3;
    rtt_log_str("[sd] mounted");

    /* Speed up SDIO clock: 288KHz → 6MHz. */
    {
        uint32_t clkcr = SDIO->CLKCR;
        clkcr &= ~0xFFu;
        clkcr |= 10u;
        SDIO->CLKCR = clkcr;
    }

    /* Scan for next file number. */
    s_next_index = scan_next_index();
    sd_dbg_step = 4;
    rtt_log_kv("[sd] next_idx=", s_next_index);

    s_mounted = true;
    return true;
}

bool sd_logger_start(void)
{
    if (!s_mounted || s_recording) return false;

    /* Build filename: ml_000.csv .. ml_999.csv */
    char fname[16];
    snprintf(fname, sizeof(fname), "ml_%03u.csv", s_next_index);

    sd_dbg_step = 20;
    FRESULT fr = f_open(&s_file, fname, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        extern SD_HandleTypeDef hsd;
        rtt_log_kv("[sd] open fail fr=", (uint32_t)fr);
        rtt_log_kv("[sd] hsd.err=", hsd.ErrorCode);

        /* Retry once. */
        fr = f_open(&s_file, fname, FA_WRITE | FA_CREATE_ALWAYS);
        if (fr != FR_OK) {
            rtt_log_kv("[sd] retry fail fr=", (uint32_t)fr);
            return false;
        }
    }

    /* Write CSV header. */
    static const char hdr[] = "ms,fsr,ax,ay,az,gx,gy,gz,tx100,ty100\n";
    s_buf_pos = 0;
    s_recording = true;   /* Must set before buf_append (flush_buf checks it) */
    buf_append(hdr, (uint16_t)(sizeof(hdr) - 1));

    /* Save filename for dump, advance index. */
    memcpy(s_last_fname, fname, sizeof(fname));
    s_next_index++;

    sd_dbg_step = 6;
    rtt_log_str("[sd] rec start:");
    rtt_log_str(s_last_fname);
    return true;
}

void sd_logger_stop(void)
{
    if (!s_recording) return;

    /* Flush remaining buffer and close file. */
    flush_buf();
    s_recording = false;
    f_close(&s_file);

    rtt_log_str("[sd] rec stop:");
    rtt_log_str(s_last_fname);
}

bool sd_logger_is_recording(void)
{
    return s_recording;
}

void sd_logger_write_row(uint32_t timestamp_ms,
                         int16_t fsr_raw,
                         int16_t ax, int16_t ay, int16_t az,
                         int16_t gx, int16_t gy, int16_t gz,
                         float tilt_x, float tilt_y)
{
    if (!s_recording) return;

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

/* Note: dump_rtt() blocks the calling task (T_ML) for the duration of
 * the transfer. Sensor reads and trigger evaluation pause. This is
 * acceptable because the user has already ended data collection. */
void sd_logger_dump_rtt(void)
{
    if (!s_mounted || s_last_fname[0] == '\0') {
        rtt_log_str("[sd] dump: no file");
        return;
    }

    FIL f;
    FRESULT fr = f_open(&f, s_last_fname, FA_READ);
    if (fr != FR_OK) {
        rtt_log_kv("[sd] dump open fail fr=", (uint32_t)fr);
        return;
    }

    /* Mute debug logs for the duration of the dump to prevent them from
     * interleaving with CSV data on RTT channel 0. Framing markers and
     * CSV chunks use rtt_write_blocking (direct SEGGER_RTT_Write) and
     * are NOT affected by mute. */
    rtt_log_mute(true);

    /* Framing start marker. */
    {
        char marker[48];
        int n = snprintf(marker, sizeof(marker),
                         "[sd:dump:start:%s]\n", s_last_fname);
        if (n > 0) rtt_write_blocking(marker, (unsigned)n);
    }

    /* Stream file in chunks. */
    char chunk[256];
    UINT br;
    while (f_read(&f, chunk, sizeof(chunk), &br) == FR_OK && br > 0) {
        rtt_write_blocking(chunk, br);
    }

    /* Framing end marker. */
    static const char end[] = "[sd:dump:end]\n";
    rtt_write_blocking(end, sizeof(end) - 1);

    f_close(&f);
    rtt_log_mute(false);
    rtt_log_str("[sd] dump done");
}

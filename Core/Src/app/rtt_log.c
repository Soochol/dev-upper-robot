/**
 * @file    rtt_log.c
 * @brief   Tiny printf-free RTT logging helpers.
 *
 * Implementation notes:
 *  - All output is buffered on the caller's stack (max 96 B) and flushed
 *    with a single SEGGER_RTT_Write call. This prevents interleaving when
 *    two tasks log at the same instant — RTT_Write is internally protected
 *    by SEGGER's lock, but only at the granularity of one call.
 *  - No floating point, no varargs, no malloc. Safe to call from any task
 *    or from a hook function (e.g., vApplicationStackOverflowHook).
 *  - The longest decimal representation of a 32-bit unsigned int is 10
 *    digits ("4294967295"), so the temporary digit buffer is 11 bytes.
 */

#include "app/rtt_log.h"
#include "SEGGER_RTT.h"

/* When true, all rtt_log_* functions return immediately. Set by
 * rtt_log_mute() during SD card CSV dump to prevent debug logs from
 * interleaving with dump data on RTT channel 0. */
static volatile bool s_muted = false;

/* Append a NUL-terminated string to dst, returning the new write pointer. */
char *append_str(char *dst, const char *src)
{
    if (src == 0) return dst;
    while (*src) *dst++ = *src++;
    return dst;
}

/* Append a uint32_t in decimal form, returning the new write pointer. */
char *append_u32(char *dst, uint32_t v)
{
    char tmp[11];
    int  n = 0;

    if (v == 0) {
        *dst++ = '0';
        return dst;
    }
    while (v > 0 && n < 10) {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (n > 0) {
        *dst++ = tmp[--n];
    }
    return dst;
}

/* Append an int32_t in decimal form (with '-' prefix if negative). */
char *append_i32(char *dst, int32_t v)
{
    if (v < 0) {
        *dst++ = '-';
        return append_u32(dst, -(uint32_t)v);  /* negate in unsigned to avoid INT32_MIN UB */
    }
    return append_u32(dst, (uint32_t)v);
}

/* Append a uint32_t in uppercase hexadecimal with leading-zero
 * suppression. Always emits at least one digit ("0" for v == 0).
 * No "0x" prefix — the caller adds that if it wants. */
static char *append_u32_hex(char *dst, uint32_t v)
{
    static const char digits[] = "0123456789ABCDEF";
    char tmp[8];
    int  n = 0;

    if (v == 0) {
        *dst++ = '0';
        return dst;
    }
    while (v > 0 && n < 8) {
        tmp[n++] = digits[v & 0xFu];
        v >>= 4;
    }
    while (n > 0) {
        *dst++ = tmp[--n];
    }
    return dst;
}

void rtt_log_str(const char *msg)
{
    if (s_muted) return;
    char  buf[96];
    char *p   = buf;
    char *end = buf + sizeof(buf) - 2;  /* reserve 2 for CRLF */
    while (*msg && p < end) *p++ = *msg++;
    *p++ = '\r';
    *p++ = '\n';
    SEGGER_RTT_Write(0, buf, (unsigned)(p - buf));
}

void rtt_log_kv(const char *prefix, uint32_t value)
{
    if (s_muted) return;
    char  buf[96];
    char *p = buf;
    p = append_str(p, prefix);
    p = append_u32(p, value);
    *p++ = '\r';
    *p++ = '\n';
    SEGGER_RTT_Write(0, buf, (unsigned)(p - buf));
}

void rtt_log_hb(const char *tag,
                const char *ka, uint32_t va,
                const char *kb, uint32_t vb,
                const char *kc, uint32_t vc,
                const char *kd, uint32_t vd)
{
    if (s_muted) return;
    char  buf[96];
    char *p   = buf;
    char *end = buf + sizeof(buf) - 4;  /* leave room for "\r\n" + safety */

    p = append_str(p, tag);
    if (ka && p < end) { p = append_str(p, ka); p = append_u32(p, va); }
    if (kb && p < end) { p = append_str(p, kb); p = append_u32(p, vb); }
    if (kc && p < end) { p = append_str(p, kc); p = append_u32(p, vc); }
    if (kd && p < end) { p = append_str(p, kd); p = append_u32(p, vd); }
    *p++ = '\r';
    *p++ = '\n';
    SEGGER_RTT_Write(0, buf, (unsigned)(p - buf));
}

void rtt_log_hb_s(const char *tag,
                  const char *ka, int32_t va,
                  const char *kb, int32_t vb,
                  const char *kc, int32_t vc,
                  const char *kd, int32_t vd)
{
    if (s_muted) return;
    char  buf[96];
    char *p   = buf;
    char *end = buf + sizeof(buf) - 4;

    p = append_str(p, tag);
    if (ka && p < end) { p = append_str(p, ka); p = append_i32(p, va); }
    if (kb && p < end) { p = append_str(p, kb); p = append_i32(p, vb); }
    if (kc && p < end) { p = append_str(p, kc); p = append_i32(p, vc); }
    if (kd && p < end) { p = append_str(p, kd); p = append_i32(p, vd); }
    *p++ = '\r';
    *p++ = '\n';
    SEGGER_RTT_Write(0, buf, (unsigned)(p - buf));
}

void rtt_log_kv_hex(const char *prefix, uint32_t value)
{
    if (s_muted) return;
    char  buf[96];
    char *p = buf;
    p = append_str(p, prefix);
    *p++ = '0';
    *p++ = 'x';
    p = append_u32_hex(p, value);
    *p++ = '\r';
    *p++ = '\n';
    SEGGER_RTT_Write(0, buf, (unsigned)(p - buf));
}

void rtt_log_mute(bool muted)
{
    s_muted = muted;
}

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

/* Append a NUL-terminated string to dst, returning the new write pointer. */
static char *append_str(char *dst, const char *src)
{
    if (src == 0) return dst;
    while (*src) *dst++ = *src++;
    return dst;
}

/* Append a uint32_t in decimal form, returning the new write pointer. */
static char *append_u32(char *dst, uint32_t v)
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

void rtt_log_str(const char *msg)
{
    char  buf[96];
    char *p = buf;
    p = append_str(p, msg);
    *p++ = '\r';
    *p++ = '\n';
    SEGGER_RTT_Write(0, buf, (unsigned)(p - buf));
}

void rtt_log_kv(const char *prefix, uint32_t value)
{
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

/**
 * @file    rtt_log.h
 * @brief   Tiny RTT logging helpers — printf-free integer + string output.
 *
 * The full SEGGER_RTT_printf() implementation lives in a separate file
 * (SEGGER_RTT_printf.c) that is not bundled with this project, and the
 * variadic format machinery costs ~3 KB of Flash even when only used for
 * decimal integers. These helpers cover everything Phase 1 stubs need
 * (string + 1..4 unsigned values) at a fraction of the cost.
 *
 * Output goes to RTT channel 0. Each call buffers the formatted line on
 * the caller's stack and emits it with one SEGGER_RTT_Write call so log
 * lines never interleave between tasks.
 */

#ifndef APP_RTT_LOG_H
#define APP_RTT_LOG_H

#include <stdbool.h>
#include <stdint.h>

/* Simple string log. Appends "\r\n". */
void rtt_log_str(const char *msg);

/* Single key=value pair, e.g. rtt_log_kv("[t_state] tick=", 42). */
void rtt_log_kv(const char *prefix, uint32_t value);

/* Event-style line with explicit timestamp prefix.
 * Output: "<tag> t=<ts_ms> ka=va kb=vb kc=vc kd=vd\r\n"
 * Pass NULL for any key to skip that field. Values are signed; unsigned
 * counters that fit in int32_t (e.g. stack high-water marks, tick deltas
 * within 24 days at 1ms tick) can be cast directly. */
void rtt_log_hb_st(const char *tag, uint32_t ts_ms,
                   const char *ka, int32_t va,
                   const char *kb, int32_t vb,
                   const char *kc, int32_t vc,
                   const char *kd, int32_t vd);

/* Periodic heartbeat: gated by `tick % RTT_HEARTBEAT_TICKS == 0` and
 * stamped with current xTaskGetTickCount() ms. Wraps rtt_log_hb_st with
 * the two concerns every task duplicates (cycle gate + timestamp). Call
 * once per task loop iteration; the gate decides whether to actually emit. */
void rtt_heartbeat(uint32_t tick, const char *tag,
                   const char *ka, int32_t va,
                   const char *kb, int32_t vb,
                   const char *kc, int32_t vc,
                   const char *kd, int32_t vd);

/* Single key=value pair with the value in hexadecimal (uppercase, no
 * leading zeros, "0x" prefix added automatically). Useful for I2C
 * addresses, register dumps, and bitfields where decimal output makes
 * the value harder to interpret. */
void rtt_log_kv_hex(const char *prefix, uint32_t value);

/* Suppress all rtt_log_* output while true.
 * Used by sd_logger_dump_rtt() to prevent debug-log interleaving with
 * CSV dump data on RTT channel 0. Dump framing markers and CSV data
 * use SEGGER_RTT_Write directly so they are NOT affected by mute. */
void rtt_log_mute(bool muted);

/* Printf-free formatting helpers — shared with sd_logger.c.
 * Each appends to dst and returns the new write pointer. */
char *append_str(char *dst, const char *src);
char *append_u32(char *dst, uint32_t v);
char *append_i32(char *dst, int32_t v);

#endif /* APP_RTT_LOG_H */

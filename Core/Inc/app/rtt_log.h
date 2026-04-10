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

/* Heartbeat-style line: "<tag> a=A b=B c=C d=D\r\n".
 * Pass NULL for any key to skip that field. */
void rtt_log_hb(const char *tag,
                const char *ka, uint32_t va,
                const char *kb, uint32_t vb,
                const char *kc, uint32_t vc,
                const char *kd, uint32_t vd);

/* Signed variant of rtt_log_hb. Negative values print with '-' prefix. */
void rtt_log_hb_s(const char *tag,
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

#endif /* APP_RTT_LOG_H */

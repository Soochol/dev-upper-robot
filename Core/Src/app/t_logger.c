/**
 * @file    t_logger.c
 * @brief   T_LOGGER task — drain q_log, format, and emit via RTT + UART.
 *
 * Phase 5 implementation. T_LOGGER blocks on q_log and formats each
 * log_msg_t into a human-readable line. Output goes to SEGGER RTT
 * channel 0 via rtt_log helpers, and (future Phase 6) to UART1 under
 * mtx_uart1 protection.
 *
 * The q_log queue is the single sink for structured telemetry from all
 * tasks. This centralised path is the future attachment point for SD
 * card logging (FATFS) and ESP32 serial forwarding. For now it simply
 * echoes to RTT.
 *
 * Note: each producer task (t_pid, t_ml) also has its own direct RTT
 * heartbeat log for bring-up convenience. Those will be removed or
 * gated in Phase 6 once T_LOGGER is the trusted single output path.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "app/config.h"
#include "app/ipc.h"
#include "app/rtt_log.h"

/* Source ID → short tag for the formatted log line. */
static const char *src_tag(uint8_t src)
{
    switch (src) {
    case LOG_SRC_STATE:  return "ST";
    case LOG_SRC_PID:    return "PD";
    case LOG_SRC_ML:     return "ML";
    case LOG_SRC_LOGGER: return "LG";
    default:             return "??";
    }
}

void t_logger_run(void *arg)
{
    (void)arg;

    rtt_log_str("[t_logger] start (Phase 5: q_log drain)");

    uint32_t msg_count = 0;
    log_msg_t msg;

    for (;;) {
        /* Block indefinitely until a log message arrives. This is the
         * only task that reads q_log, so no contention. */
        if (xQueueReceive(q_log, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        msg_count++;

        /* Format: [LOG:<SRC>] t=<ms> code=<N> val=<V> ex=<E>
         * Using rtt_log_hb which supports 4 key-value pairs. The source
         * tag is embedded in the prefix string. */
        const char *tag = src_tag(msg.source);

        /* Build a compact prefix: "[L:XX] " where XX is the 2-char tag.
         * We use a small stack buffer because rtt_log_hb's tag param is
         * a plain string pointer. */
        char prefix[12];
        prefix[0] = '[';
        prefix[1] = 'L';
        prefix[2] = ':';
        prefix[3] = tag[0];
        prefix[4] = tag[1];
        prefix[5] = ']';
        prefix[6] = '\0';

        rtt_log_hb_s(prefix,
                     " t=",  (int32_t)msg.timestamp_ms,
                     " c=",  (int32_t)msg.code,
                     " v=",  msg.value,
                     " x=",  msg.extra);
    }
}

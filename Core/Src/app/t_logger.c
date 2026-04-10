/**
 * @file    t_logger.c
 * @brief   T_LOGGER task — drain q_log to RTT (and UART1 in Phase 5).
 *
 * Phase 1 stub: blocks on q_log forever. Since no producer publishes
 * messages yet in Phase 1, this task should idle at the queue receive and
 * never consume CPU. That fact alone is a useful invariant — if heartbeat
 * shows T_LOGGER spinning, something is producing log messages we did not
 * intend.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "app/config.h"
#include "app/ipc.h"
#include "app/rtt_log.h"

void t_logger_run(void *arg)
{
    (void)arg;

    rtt_log_str("[t_logger] start");

    log_msg_t msg;
    for (;;) {
        if (xQueueReceive(q_log, &msg, portMAX_DELAY) == pdTRUE) {
            /* Phase 1: just echo to RTT. Phase 5 adds dual UART transport
             * with mtx_uart1 protection and proper formatting. */
            rtt_log_hb("[log]",
                       " t=",    msg.timestamp_ms,
                       " src=",  (uint32_t)msg.source,
                       " code=", (uint32_t)msg.code,
                       " val=",  (uint32_t)msg.value);
        }
    }
}

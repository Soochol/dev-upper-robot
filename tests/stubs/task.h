/**
 * @file    task.h  (HOST STUB)
 * @brief   Minimal stub so trigger_rule.c compiles on the host.
 *
 * The real xTaskGetTickCount() is replaced by a mock that reads
 * g_mock_tick from the test file. The test file defines g_mock_tick
 * and this stub declares the function as extern.
 */

#ifndef TASK_H_STUB
#define TASK_H_STUB

#include "FreeRTOS.h"

/* Defined in test_trigger.c — the test controls the mock tick value. */
extern TickType_t xTaskGetTickCount(void);

#endif /* TASK_H_STUB */

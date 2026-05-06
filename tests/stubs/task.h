/* FreeRTOS task.h stub */
#ifndef TASK_H_STUB
#define TASK_H_STUB
#include "FreeRTOS.h"

/* Minimal API surface used by trigger_*.c host tests. The test binary
 * provides xTaskGetTickCount() definition that drives g_mock_tick. */
TickType_t xTaskGetTickCount(void);
#define portTICK_PERIOD_MS  ((TickType_t)1)

#endif

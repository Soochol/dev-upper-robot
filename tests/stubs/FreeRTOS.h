/* FreeRTOS stub for host-side unit tests. */
#ifndef FREERTOS_H_STUB
#define FREERTOS_H_STUB
#include <stdint.h>
typedef uint32_t TickType_t;
typedef void* TaskHandle_t;
typedef void* SemaphoreHandle_t;
typedef void* QueueHandle_t;
typedef void* xTaskHandle;
#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  1
typedef int BaseType_t;
#define configMINIMAL_STACK_SIZE 128
#endif

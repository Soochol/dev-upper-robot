/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "semphr.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Single-producer thermal state, written by thermal_ctrl_task only.
 * Other tasks read these fields without locking — each field fits in a
 * single 32-bit word so reads are naturally atomic on Cortex-M3. */
typedef struct {
  float    ir_temp_c;       /* latest IR sensor reading, °C */
  uint32_t pwm_heater;      /* TIM1 CH1 compare value */
  uint32_t pwm_fan;         /* TIM1 CH2 compare value */
  uint32_t timestamp_ms;    /* HAL_GetTick() at last update */
} thermal_state_t;

/* Logger queue payload — kept small (24 bytes) so q_log fits in heap. */
typedef struct {
  uint32_t timestamp_ms;
  uint16_t adc_batt;
  uint16_t adc_temp_bd;
  uint16_t adc_temp_air;
  uint16_t adc_current;
  float    ir_temp_c;
  uint32_t pwm_heater;
} log_msg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Application task handles — created with raw FreeRTOS API in MX_FREERTOS_Init
 * (USER CODE BEGIN RTOS_THREADS). The CubeMX-generated defaultTask above stays
 * as the watchdog/heartbeat. */
TaskHandle_t      h_thermal_ctrl;
TaskHandle_t      h_sensor;
TaskHandle_t      h_logger;
TaskHandle_t      h_comm;

/* Inter-task primitives. q_log decouples sensor producers from the slow
 * SD-card logger; q_setpoint lets comm push new thermal targets without
 * blocking on the control loop. */
QueueHandle_t     q_log;
QueueHandle_t     q_setpoint;
SemaphoreHandle_t sem_ultrasonic;   /* TIM3 input-capture ISR → sensor task */
SemaphoreHandle_t mtx_printf;       /* protects __io_putchar dual transport */

/* Shared thermal state — read by sensor/logger, written by thermal_ctrl. */
volatile thermal_state_t g_thermal;

/* ADC DMA buffer is owned by main.c (static there). We re-declare an extern
 * here so sensor_task can read the latest values without going through HAL. */
extern volatile uint32_t adc_buf[4];

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void thermal_ctrl_task(void *argument);
static void sensor_task     (void *argument);
static void logger_task     (void *argument);
static void comm_task       (void *argument);

/* IR sensor read stub — replace with real driver (e.g. MLX90614 over I2C1)
 * once the hardware part number is confirmed. Returns HAL_OK on success and
 * writes the temperature in Celsius to *out. The current stub returns a
 * fixed value so the control loop links cleanly for the first build. */
static HAL_StatusTypeDef ir_temp_read(I2C_HandleTypeDef *hi2c, float *out);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Protects __io_putchar so concurrent printf() from multiple tasks does
   * not interleave bytes on UART1 / RTT. Created BEFORE any task that may
   * call printf() — main.c boot prints before scheduler starts and use a
   * NULL-check fallback in __io_putchar. */
  mtx_printf = xSemaphoreCreateMutex();
  configASSERT(mtx_printf != NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* Binary semaphore signaled from TIM3 input-capture ISR each time a new
   * ultrasonic echo is captured. sensor_task waits on it. */
  sem_ultrasonic = xSemaphoreCreateBinary();
  configASSERT(sem_ultrasonic != NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* Sensor → logger pipeline. Depth 16 covers ~1.6s of buffering at the
   * planned 10 Hz sensor rate, enough to absorb SD-card write stalls. */
  q_log = xQueueCreate(16, sizeof(log_msg_t));
  configASSERT(q_log != NULL);

  /* comm → thermal_ctrl setpoint updates. Depth 4 because new setpoints
   * are infrequent and we'd rather drop than block the comm parser. */
  q_setpoint = xQueueCreate(4, sizeof(float));
  configASSERT(q_setpoint != NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Application tasks created with raw FreeRTOS API. Priority ladder:
   *   5 = thermal_ctrl  (IR-feedback heater control loop, deterministic 50ms)
   *   4 = comm          (USART3 command parsing, latency-sensitive)
   *   3 = sensor        (ADC + ultrasonic polling)
   *   1 = logger        (SD card + UART output, blocks freely)
   *   1 = defaultTask   (created above by CubeMX, repurposed as heartbeat)
   * Priority 0 is reserved for the FreeRTOS idle task. */
  BaseType_t ok;
  ok = xTaskCreate(thermal_ctrl_task, "thermal", 384, NULL, 5, &h_thermal_ctrl);
  configASSERT(ok == pdPASS);
  ok = xTaskCreate(comm_task,         "comm",    256, NULL, 4, &h_comm);
  configASSERT(ok == pdPASS);
  ok = xTaskCreate(sensor_task,       "sensor",  256, NULL, 3, &h_sensor);
  configASSERT(ok == pdPASS);
  ok = xTaskCreate(logger_task,       "logger", 1024, NULL, 1, &h_logger);
  configASSERT(ok == pdPASS);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* defaultTask doubles as the system heartbeat: 1 Hz log + (future) IWDG
   * kick. We keep it on the CMSIS-OS v1 wrapper because that is what CubeMX
   * generates by default and it lets PSP.ioc stay regenerable. */
  TickType_t last_wake = xTaskGetTickCount();
  for (;;)
  {
    /* TODO: HAL_IWDG_Refresh(&hiwdg) once watchdog is enabled in .ioc */
    printf("hb up=%lus thermal=%.1fC pwm_h=%lu free_heap=%u\r\n",
           (unsigned long)(HAL_GetTick() / 1000u),
           (double)g_thermal.ir_temp_c,
           (unsigned long)g_thermal.pwm_heater,
           (unsigned int)xPortGetFreeHeapSize());
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ===========================================================================
 *  IR temperature sensor — STUB
 * ---------------------------------------------------------------------------
 *  TODO: replace with real driver once the part number is confirmed. The
 *  most likely candidates for this board are MLX90614 (I2C addr 0x5A,
 *  ambient register 0x06, object register 0x07) or AMG8833. Both speak SMBus
 *  PEC which our HAL_I2C_Mem_Read does not generate by default — the real
 *  driver may need a custom transaction.
 *
 *  Until then this stub returns a constant 25.0 °C so the control loop and
 *  log pipeline can be exercised end-to-end.
 * ===========================================================================*/
static HAL_StatusTypeDef ir_temp_read(I2C_HandleTypeDef *hi2c, float *out)
{
  (void)hi2c;
  if (out == NULL) return HAL_ERROR;
  *out = 25.0f;            /* placeholder */
  return HAL_OK;
}

/* ===========================================================================
 *  thermal_ctrl_task  —  highest-priority closed-loop heater control
 * ---------------------------------------------------------------------------
 *  Period: 50 ms (vTaskDelayUntil — jitter-free).
 *  Reads IR sensor over I2C1, applies a stub on/off controller, drives the
 *  TIM1 CH1 (heater) and CH2 (fan) PWM compares, and publishes state to
 *  g_thermal for sensor/logger consumption.
 *
 *  PID is intentionally NOT wired up yet — we want first-build to link and
 *  the user to confirm the IR sensor works before tuning gains.
 * ===========================================================================*/
static void thermal_ctrl_task(void *argument)
{
  (void)argument;
  TickType_t last_wake = xTaskGetTickCount();
  float setpoint_c = 25.0f;

  for (;;)
  {
    /* 1. Pick up new setpoint from comm_task if any (non-blocking). */
    float new_sp;
    if (xQueueReceive(q_setpoint, &new_sp, 0) == pdTRUE)
    {
      setpoint_c = new_sp;
    }

    /* 2. Read IR sensor (currently stub, ~0 ms). */
    float temp_c;
    if (ir_temp_read(&hi2c1, &temp_c) != HAL_OK)
    {
      /* Sensor failure: heater off, fan off, keep loop running. */
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
      continue;
    }

    /* 3. Trivial bang-bang controller — replace with PID once gains known. */
    uint32_t pwm_heater = (temp_c < setpoint_c) ? 800u : 0u;
    uint32_t pwm_fan    = (temp_c > setpoint_c + 2.0f) ? 800u : 0u;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_heater);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_fan);

    /* 4. Publish state (single writer → other tasks read field-by-field). */
    g_thermal.ir_temp_c    = temp_c;
    g_thermal.pwm_heater   = pwm_heater;
    g_thermal.pwm_fan      = pwm_fan;
    g_thermal.timestamp_ms = HAL_GetTick();

    /* 5. 20 Hz exact period regardless of how long steps 2–4 took. */
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
  }
}

/* ===========================================================================
 *  sensor_task  —  10 Hz ADC + ultrasonic snapshotting
 * ---------------------------------------------------------------------------
 *  Reads the DMA-filled adc_buf[], stamps a log_msg_t, and pushes to q_log.
 *  Does not block on the queue — if the logger falls behind we drop samples
 *  rather than introduce backpressure into the sense path.
 * ===========================================================================*/
static void sensor_task(void *argument)
{
  (void)argument;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;)
  {
    log_msg_t msg = {
      .timestamp_ms = HAL_GetTick(),
      .adc_batt     = (uint16_t)adc_buf[0],
      .adc_temp_bd  = (uint16_t)adc_buf[1],
      .adc_temp_air = (uint16_t)adc_buf[2],
      .adc_current  = (uint16_t)adc_buf[3],
      .ir_temp_c    = g_thermal.ir_temp_c,
      .pwm_heater   = g_thermal.pwm_heater,
    };

    /* Drop if queue full — never block the producer. */
    (void)xQueueSend(q_log, &msg, 0);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
  }
}

/* ===========================================================================
 *  logger_task  —  q_log → SD card + UART (slow path)
 * ---------------------------------------------------------------------------
 *  Allowed to block freely on f_write() because nobody upstream is waiting
 *  for it. SD card writes can stall for tens of ms — that is exactly why
 *  this lives in its own low-priority task.
 *
 *  TODO: open log file with f_open() once mount path is confirmed.
 * ===========================================================================*/
static void logger_task(void *argument)
{
  (void)argument;
  log_msg_t msg;

  for (;;)
  {
    if (xQueueReceive(q_log, &msg, portMAX_DELAY) == pdTRUE)
    {
      printf("ADC batt=%4u temp_bd=%4u temp_air=%4u curr=%4u  "
             "IR=%5.1fC  pwm_h=%lu\r\n",
             msg.adc_batt, msg.adc_temp_bd, msg.adc_temp_air, msg.adc_current,
             (double)msg.ir_temp_c, (unsigned long)msg.pwm_heater);
      /* TODO: f_write(&log_file, ...) once SD card path is wired up. */
    }
  }
}

/* ===========================================================================
 *  comm_task  —  USART3 command parser
 * ---------------------------------------------------------------------------
 *  Stub: blocks 1s and prints a placeholder. Replace with HAL_UART_Receive_IT
 *  + ring buffer + parser once the host protocol is defined. New setpoints
 *  go to thermal_ctrl via q_setpoint.
 * ===========================================================================*/
static void comm_task(void *argument)
{
  (void)argument;
  for (;;)
  {
    /* TODO: HAL_UART_Receive_IT(&huart3, ...) and parse incoming bytes. */
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/* USER CODE END Application */


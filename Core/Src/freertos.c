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
#include "SEGGER_RTT.h"
#include "queue.h"
#include "semphr.h"
#include "app/config.h"
#include "app/ipc.h"
#include "app/fsm.h"
#include "app/rtt_log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Application task handles. defaultTaskHandle (CMSIS-OS v1) is declared
 * separately below by the CubeMX-generated code; the four application tasks
 * use the raw FreeRTOS API (TaskHandle_t) instead. */
TaskHandle_t h_t_state;
TaskHandle_t h_t_pid;
TaskHandle_t h_t_ml;
TaskHandle_t h_t_logger;

/* PI mutex for the I2C1 bus shared by T_PID (IR sensors) and T_ML
 * (ICM42670P + ADS1115). Created with xSemaphoreCreateMutex() so priority
 * inheritance is automatic — see plan A1 for the blocking analysis. */
SemaphoreHandle_t mtx_i2c1;

/* Mutex for the dual RTT+UART transport used by __io_putchar in main.c.
 * Required because both the logger task and any defaultTask printf would
 * otherwise interleave bytes on the wire. */
SemaphoreHandle_t mtx_uart1;

/* Inter-task queues. Depths come from config.h so we can resize them in
 * one place if a particular path turns out to be a bottleneck. q_btn_evt
 * was removed when buttons were dropped from the design (D13). */
QueueHandle_t q_log;
QueueHandle_t q_ctrl_to_pid;
QueueHandle_t q_trigger_to_state;
QueueHandle_t q_fault_req;

/* Current FSM state, written only by T_STATE, read by T_PID for safety
 * invariants and by defaultTask for the heartbeat dump. uint32_t alignment
 * makes single-word reads atomic on Cortex-M3 — see plan A8. */
volatile uint32_t g_fsm_state = 0;  /* FSM_FORCE_DOWN */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* Application task entry points. Each lives in Core/Src/app/t_*.c. Forward
 * declared here rather than in a separate header so adding a new task is a
 * one-line change in this file. */
void t_state_run (void *arg);
void t_pid_run   (void *arg);
void t_ml_run    (void *arg);
void t_logger_run(void *arg);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  /* Stack overflow is unrecoverable: the offending task may have corrupted
   * the TCB or adjacent task stacks before this hook is called. We must NOT
   * try to schedule any further work on this kernel state.
   *
   * Strategy: emit one synchronous RTT line so the failing task is visible
   * in the debug console, then trip a hardware reset. NEVER use printf() or
   * any FreeRTOS API here — interrupts may be in an unknown state and the
   * heap may be corrupted. SEGGER_RTT_Write is safe because it touches a
   * fixed memory region with no locks.
   *
   * Plan reference: A6.
   */
  (void)xTask;
  SEGGER_RTT_Write(0, "\r\n!!! STACK OVERFLOW: ", 22);
  if (pcTaskName != NULL) {
    /* pcTaskName is a TCB-owned NUL-terminated string up to
     * configMAX_TASK_NAME_LEN bytes. Walk it manually to avoid strlen. */
    const char *p = (const char *)pcTaskName;
    int n = 0;
    while (n < 16 && p[n] != '\0') n++;
    SEGGER_RTT_Write(0, p, n);
  }
  SEGGER_RTT_Write(0, " !!!\r\n", 6);

  /* Give the debugger a chance to drain RTT before reset. Busy-wait because
   * vTaskDelay would be unsafe at this point. */
  for (volatile uint32_t i = 0; i < 2000000; i++) { __asm__ volatile ("nop"); }

  NVIC_SystemReset();
  for (;;) { /* unreachable */ }
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
  /* PI mutex — priority inheritance is automatic with xSemaphoreCreateMutex.
   * NEVER replace with xSemaphoreCreateBinary() — see plan A1. */
  mtx_i2c1  = xSemaphoreCreateMutex();
  mtx_uart1 = xSemaphoreCreateMutex();
  configASSERT(mtx_i2c1  != NULL);
  configASSERT(mtx_uart1 != NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  q_log              = xQueueCreate(Q_LOG_DEPTH,              sizeof(log_msg_t));
  q_ctrl_to_pid      = xQueueCreate(Q_CTRL_TO_PID_DEPTH,      sizeof(ctrl_cmd_t));
  q_trigger_to_state = xQueueCreate(Q_TRIGGER_TO_STATE_DEPTH, sizeof(trig_msg_t));
  q_fault_req        = xQueueCreate(Q_FAULT_REQ_DEPTH,        sizeof(fault_req_t));
  configASSERT(q_log              != NULL);
  configASSERT(q_ctrl_to_pid      != NULL);
  configASSERT(q_trigger_to_state != NULL);
  configASSERT(q_fault_req        != NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Application tasks (raw FreeRTOS API).
   *
   * Phase 1: stub bodies — each task just delays and prints heartbeat to RTT.
   * We create them now so the IPC primitives above are exercised at boot
   * and we can verify the scheduler handles 5 tasks (4 app + defaultTask)
   * before piling on real driver code in later phases.
   *
   * Stack sizes are deliberately generous during bring-up; we shrink in
   * Phase 6 after measuring uxTaskGetStackHighWaterMark from each task. */
  BaseType_t ok;
  ok = xTaskCreate(t_state_run,  "T_STATE",  STK_T_STATE_WORDS,
                   NULL, PRIO_T_STATE,  &h_t_state);
  configASSERT(ok == pdPASS);
  ok = xTaskCreate(t_pid_run,    "T_PID",    STK_T_PID_WORDS,
                   NULL, PRIO_T_PID,    &h_t_pid);
  configASSERT(ok == pdPASS);
  ok = xTaskCreate(t_ml_run,     "T_ML",     STK_T_ML_WORDS,
                   NULL, PRIO_T_ML,     &h_t_ml);
  configASSERT(ok == pdPASS);
  ok = xTaskCreate(t_logger_run, "T_LOGGER", STK_T_LOGGER_WORDS,
                   NULL, PRIO_T_LOGGER, &h_t_logger);
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
  /* 1 Hz health beacon. Reports:
   *   - tick counter (proves scheduler is alive)
   *   - free heap (xPortGetFreeHeapSize, drops if any task leaks)
   *   - lowest free heap watermark (xPortGetMinimumEverFreeHeapSize)
   *   - current FSM state from g_fsm_state (atomic 32-bit read)
   *
   * In Phase 2, this task also doubles as the auto-trigger emulator
   * (PHASE2_AUTO_TRIGGER) — every PHASE2_AUTO_TRIGGER_PERIOD_MS we publish
   * an alternating UP/DOWN trigger to q_trigger_to_state so the FSM
   * cycles even though T_ML still has no real trigger source. Removed
   * by setting PHASE2_AUTO_TRIGGER to 0 in Phase 4.
   *
   * Uses rtt_log_hb (printf-free) which is non-blocking and safe from any
   * task. Avoid newlib printf here so this loop never blocks on UART
   * transmit or grabs mtx_uart1 — defaultTask is the canary that proves
   * the rest of the system is healthy, so it must not depend on the rest
   * of the system being healthy. */
  uint32_t tick = 0;
#if PHASE2_AUTO_TRIGGER
  /* Period in heartbeat ticks. heartbeat is 1 Hz and PHASE2_AUTO_TRIGGER_PERIOD_MS
   * is 10000, so we fire on every 10th tick. */
  const uint32_t auto_trig_every = PHASE2_AUTO_TRIGGER_PERIOD_MS / PERIOD_HEARTBEAT_MS;
  bool next_trig_is_up = true;  /* first trigger lifts FORCE_DOWN → FORCE_UP */
#endif

  for (;;)
  {
    tick++;
    rtt_log_hb("[hb]",
        " t=",    tick,
        " free=", (uint32_t)xPortGetFreeHeapSize(),
        " lo=",   (uint32_t)xPortGetMinimumEverFreeHeapSize(),
        " fsm=",  g_fsm_state);

    /* Stack high-water mark: minimum free stack in WORDS for each task.
     * Logged every 10 seconds to avoid RTT noise. A value approaching 0
     * means the task is close to stack overflow. Use these numbers to
     * right-size STK_*_WORDS in config.h after bring-up stabilizes. */
    if ((tick % 10) == 0) {
      rtt_log_hb("[hb:stk]",
          " st=",  (uint32_t)uxTaskGetStackHighWaterMark(h_t_state),
          " pid=", (uint32_t)uxTaskGetStackHighWaterMark(h_t_pid),
          " ml=",  (uint32_t)uxTaskGetStackHighWaterMark(h_t_ml),
          " log=", (uint32_t)uxTaskGetStackHighWaterMark(h_t_logger));
    }

#if PHASE2_AUTO_TRIGGER
    if (auto_trig_every > 0 && (tick % auto_trig_every) == 0) {
      trig_msg_t tmsg = {
        .event = (uint8_t)(next_trig_is_up
                           ? FSM_EVT_TRIGGER_FORCE_UP
                           : FSM_EVT_TRIGGER_FORCE_DOWN),
        .pad   = {0, 0, 0},
      };
      (void)xQueueSendToBack(q_trigger_to_state, &tmsg, 0);
      rtt_log_str(next_trig_is_up
                  ? "[hb] auto-trig: TRIG_UP"
                  : "[hb] auto-trig: TRIG_DOWN");
      next_trig_is_up = !next_trig_is_up;
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(PERIOD_HEARTBEAT_MS));
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


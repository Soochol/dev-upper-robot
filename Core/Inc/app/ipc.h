/**
 * @file    ipc.h
 * @brief   FreeRTOS IPC primitives shared between application tasks.
 *
 * All extern declarations for queues, mutexes, and task handles created in
 * MX_FREERTOS_Init() (Core/Src/freertos.c). Including this header gives a
 * task access to every queue/mutex it might need; the actual handles are
 * defined exactly once in freertos.c.
 *
 * Message struct definitions live here too because they describe the
 * contract between producer and consumer tasks.
 */

#ifndef APP_IPC_H
#define APP_IPC_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "app/fsm.h"

/* ========================================================================
 * Task handles
 * ======================================================================== */

extern TaskHandle_t h_t_state;
extern TaskHandle_t h_t_pid;
extern TaskHandle_t h_t_ml;
extern TaskHandle_t h_t_logger;
extern TaskHandle_t h_t_wdg;

/* ========================================================================
 * Mutexes
 * ======================================================================== */

/* PI mutex protecting all I2C1 access. Created with xSemaphoreCreateMutex()
 * so priority inheritance is automatic — see plan A1 for blocking analysis.
 * Held by T_PID for IR reads (~2 ms) and by T_ML for IMU+FSR reads (~2 ms),
 * with offset interleave making contention near-zero in normal operation. */
extern SemaphoreHandle_t mtx_i2c1;

/* Standard mutex for T_LOGGER's dual RTT+UART transport. Required because
 * defaultTask and T_LOGGER both call __io_putchar (eventually). */
extern SemaphoreHandle_t mtx_uart1;

/* ========================================================================
 * Queue message structs
 * ======================================================================== */

/* T_PID/T_ML → T_LOGGER. Kept compact (24 B) so 16 entries fit comfortably
 * in heap. Severity is encoded as a small integer to avoid string copies. */
typedef enum {
    LOG_LVL_DEBUG = 0,
    LOG_LVL_INFO,
    LOG_LVL_WARN,
    LOG_LVL_ERROR,
} log_level_t;

typedef struct {
    uint32_t    timestamp_ms;
    log_level_t level;
    uint8_t     source;       /* task ID, see LOG_SRC_* below */
    uint8_t     code;         /* application-defined event code */
    int32_t     value;        /* primary numeric payload */
    int32_t     extra;        /* secondary numeric payload */
} log_msg_t;

#define LOG_SRC_DEFAULT 0
#define LOG_SRC_STATE   1
#define LOG_SRC_PID     2
#define LOG_SRC_ML      3
#define LOG_SRC_LOGGER  4

/* T_STATE → T_PID. New setpoint and fan command issued whenever the FSM
 * transitions. T_PID polls this queue non-blocking each tick. */
typedef struct {
    int16_t  setpoint_c;
    uint8_t  fan_duty_pct;
    uint8_t  led_pattern;
    uint8_t  pid_enabled;     /* 0 = freeze heater at 0, 1 = active */
    uint8_t  pad[3];
} ctrl_cmd_t;

/* T_ML → T_STATE. Trigger event from the active provider (rule or ml). */
typedef struct {
    uint8_t  event;           /* fsm_event_t value */
    uint8_t  pad[3];
} trig_msg_t;

/* T_PID / T_ML → T_STATE. Raised when over-temperature, sensor error, or
 * I2C bus failure is detected. Tiny payload — just the reason for logging. */
typedef struct {
    uint8_t  reason;          /* fault_reason_t */
    uint8_t  pad[3];
} fault_req_t;

typedef enum {
    FAULT_REASON_OVERTEMP = 0,
    FAULT_REASON_SENSOR_TIMEOUT,    /* IR (T_PID) */
    FAULT_REASON_PID_WATCHDOG,
    FAULT_REASON_FSR_TIMEOUT,       /* FSR/ADS1115 (T_ML) */
    FAULT_REASON_IMU_TIMEOUT,       /* IMU/ICM42670P (T_ML) */
    FAULT_REASON_WATCHDOG_RECOVERY, /* Boot following an IWDG reset */
} fault_reason_t;

/* ========================================================================
 * Queue handles
 * ======================================================================== */

extern QueueHandle_t q_log;              /* log_msg_t,    depth Q_LOG_DEPTH */
extern QueueHandle_t q_ctrl_to_pid;      /* ctrl_cmd_t,   depth Q_CTRL_TO_PID_DEPTH */
extern QueueHandle_t q_trigger_to_state; /* trig_msg_t,   depth Q_TRIGGER_TO_STATE_DEPTH */
extern QueueHandle_t q_fault_req;        /* fault_req_t,  depth Q_FAULT_REQ_DEPTH */

/* ========================================================================
 * Shared atomic state
 * ======================================================================== */

/* Current FSM state, written only by T_STATE, read by T_PID for safety
 * invariants. uint32_t alignment guarantees atomic read on Cortex-M3. */
extern volatile uint32_t g_fsm_state;

/* ========================================================================
 * Canary counters (IWDG watchdog)
 * ========================================================================
 * Each task increments its counter at the end of its main loop. T_WDG
 * reads all three each tick. volatile uint32_t at file scope is atomic
 * on Cortex-M3 (aligned 32-bit load/store). */

extern volatile uint32_t g_canary_state;
extern volatile uint32_t g_canary_pid;
extern volatile uint32_t g_canary_ml;

/* Failsafe latch — set to 1 by T_WDG when a canary stall is detected,
 * read by T_PID to skip its actuator writes so they cannot overwrite
 * the emergency-off applied by T_WDG. Single-writer (T_WDG), single-
 * reader (T_PID), uint8_t volatile is atomic on Cortex-M3. Once set,
 * stays set until IWDG reset clears it (terminal state). */
extern volatile uint8_t  g_failsafe_active;

/* ========================================================================
 * Stall diagnostics — last phase each task entered
 * ========================================================================
 * Each task updates its phase byte at major boundaries within its main
 * loop. T_WDG includes these in its STALL log so we know *which line of
 * code* a task was executing when it stopped incrementing its canary.
 * Pure observability — no behavior change, single-byte volatile writes
 * are atomic on Cortex-M3. Phase values defined per-task in their .c file. */

extern volatile uint8_t  g_phase_state;
extern volatile uint8_t  g_phase_pid;
extern volatile uint8_t  g_phase_ml;

/* Cycle duration of the last completed iteration, in FreeRTOS ticks
 * (1 tick = 1 ms with portTICK_PERIOD_MS=1). 0 means "no cycle completed
 * yet". Useful for catching gradual slowdowns before they become stalls. */
extern volatile uint32_t g_cycle_ms_state;
extern volatile uint32_t g_cycle_ms_pid;
extern volatile uint32_t g_cycle_ms_ml;

/* Per-phase wall-clock max duration (ms) for T_PID, indexed by phase value.
 * Updated by t_pid.c on every phase transition; reset by T_WDG when
 * dumped at 1 Hz. Lets us pinpoint which step inside T_PID's loop is
 * eating the cycle budget. Includes preemption time from higher-priority
 * tasks — phase whose value spikes is where the wall-clock is being lost. */
#define PID_PHASE_COUNT 10
extern volatile uint32_t g_pid_phase_ms[PID_PHASE_COUNT];

/* ========================================================================
 * Reset cause (snapshot of RCC_CSR taken before clearing)
 * ========================================================================
 * Written exactly once by main.c before FreeRTOS starts; read by tasks at
 * init time. Single-writer/multi-reader with strict happens-before via
 * scheduler start, so volatile uint32_t is sufficient (no mutex). Bits
 * follow stm32f1xx HAL: RCC_CSR_IWDGRSTF, RCC_CSR_SFTRSTF, etc. */

extern volatile uint32_t g_reset_cause;

#endif /* APP_IPC_H */

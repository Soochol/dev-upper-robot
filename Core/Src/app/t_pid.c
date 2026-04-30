/**
 * @file    t_pid.c
 * @brief   T_PID task — full control loop (Phase 3b).
 *
 * 20 Hz periodic loop:
 *
 *   1. Read both IR sensors via the I2C1 mutex.
 *   2. Drain q_ctrl_to_pid (coalesce any backlog) → current_cmd.
 *   3. Pick PID input = max(IR1, IR2) for safety: always track the
 *      hotter sensor so over-temperature is detected fastest.
 *   4. Over-temp check: if either IR > OVERTEMP_HARD_C, send a fault
 *      request to T_STATE and force heater_duty = 0 for this cycle.
 *   5. FSM state check: if g_fsm_state == FAULT, force heater_duty = 0
 *      and fan_duty = 0 regardless of any other input. T_STATE shuts
 *      down PC5 right after entering FAULT, so this last cycle must
 *      leave actuators in their safe state before VDD drops.
 *   6. PID compute (only if pid_enabled in current_cmd).
 *   7. Write heater PWM, fan PWM, LED pattern via actuators.
 *   8. Heartbeat log every second.
 *
 * The IR read can fail (HAL_ERROR / TIMEOUT) without crashing the loop.
 * On failure we keep the previous valid measurement and increment a
 * fault counter — three consecutive failures → q_fault_req with
 * FAULT_REASON_SENSOR_TIMEOUT.
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "i2c.h"
#include "app/config.h"
#include "app/ipc.h"
#include "app/fsm.h"
#include "app/sensors_i2c.h"
#include "app/actuators.h"
#include "app/pid.h"
#include "app/rtt_log.h"
#if DATA_COLLECT_MODE
#include "app/sd_logger.h"
#endif

/* Consecutive read failures before escalating to FAULT. Set high enough
 * to survive the boot-time init window where T_ML holds mtx_i2c1 for
 * ~300 ms (ICM42670P soft reset + startup). At 20 Hz that's ~6 missed
 * reads; we use 40 (~2 seconds) as a comfortable margin so only real
 * sustained sensor failures trigger FAULT. */
#define IR_READ_FAIL_LIMIT   40u

/* Phase markers written to g_phase_pid (see t_ml.c for rationale). */
enum {
    PHASE_PID_IDLE       = 0,
    PHASE_PID_MUTEX_TAKE = 1,
    PHASE_PID_IR_READ    = 2,   /* HAL_I2C busy-wait for both IR sensors */
    PHASE_PID_DRAIN_CMD  = 3,   /* drain q_ctrl_to_pid */
    PHASE_PID_FAULT_CHK  = 4,   /* fail count + recovery decision */
    PHASE_PID_RECOVERY   = 5,   /* I2C bus recovery loop */
    PHASE_PID_OVERTEMP   = 6,   /* over-temp evaluation */
    PHASE_PID_COMPUTE    = 7,   /* PID compute */
    PHASE_PID_ACTUATE    = 8,   /* heater PWM, fan PWM, LED */
    PHASE_PID_HEARTBEAT  = 9,   /* RTT log */
};

/* Per-phase wall-clock duration (ms) accumulator. On each phase transition,
 * record how long was spent in the previous phase; keep the running max
 * per phase in the global array. T_WDG dumps and resets these at 1 Hz. */
static TickType_t s_pid_phase_start = 0;

static inline void pid_phase_enter(uint8_t new_phase)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t elapsed = (uint32_t)((now - s_pid_phase_start)
                                  * portTICK_PERIOD_MS);
    uint8_t prev = g_phase_pid;
    if (prev < PID_PHASE_COUNT && elapsed > g_pid_phase_ms[prev]) {
        g_pid_phase_ms[prev] = elapsed;
    }
    g_phase_pid = new_phase;
    s_pid_phase_start = now;
}

void t_pid_run(void *arg)
{
    (void)arg;

    rtt_log_str("[t_pid] start (Phase 3b: full control loop)");

    /* Bring up actuators FIRST so the timer is generating zero-duty PWM
     * before any code path can ask for a non-zero output. */
    actuators_init();

    /* PID instance — gains from config.h, output range 0..1000 to match
     * TIM1 ARR. PLACEHOLDER gains; tune in Phase 6. */
    pid_ctrl_t heater_pid;
    pid_init(&heater_pid, PID_KP, PID_KI, PID_KD,
             (float)PID_OUTPUT_MIN, (float)PID_OUTPUT_MAX);
    pid_set_tuning(&heater_pid, PID_SETPOINT_WEIGHT_B, PID_DERIV_FILTER_N);

    /* Sensor subsystem bring-up — 200 ms wait + bus scan + IR ping. */
    if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(500)) == pdTRUE) {
        (void)sensors_i2c_init(&hi2c1);
        xSemaphoreGive(mtx_i2c1);
    } else {
        rtt_log_str("[t_pid] init: mtx_i2c1 timeout, continuing anyway");
    }

    /* Default command matches the boot FSM state (FORCE_DOWN). T_STATE
     * will publish a fresh ctrl_cmd_t shortly after we start, but having
     * a sensible default avoids a brief window where PWM = 0 because
     * pid_enabled defaulted to false. */
    ctrl_cmd_t current_cmd = {
        .setpoint_c   = TEMP_COOL_C,
        .fan_duty_pct = FAN_DUTY_FORCE_DOWN,
        .led_pattern  = LED_FADE_YELLOW,
        .pid_enabled  = 1u,
        .pad          = {0, 0, 0},
    };

    float    last_valid_meas = (float)TEMP_COOL_C;
    uint32_t ir_fail_count   = 0u;
    bool     reached_target  = false;
    uint8_t  prev_led_pat    = current_cmd.led_pattern;

    /* FU 2-phase control state (APPROACH → SETTLE).
     * Reset to APPROACH on every FU entry (detected via prev_state). */
    fu_phase_t  fu_phase   = FU_PHASE_APPROACH;
    fsm_state_t prev_state = FSM_FORCE_DOWN;

    TickType_t next_wake = xTaskGetTickCount();
    uint32_t   tick = 0;

    for (;;) {
        pid_phase_enter(PHASE_PID_IDLE);
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(PERIOD_T_PID_MS));
        TickType_t cycle_start = xTaskGetTickCount();
        tick++;

        /* ---- 0. Failsafe gate ----
         * If T_WDG has detected a canary stall, stop touching actuators.
         * T_WDG already wrote heater=0 / fan_pwr=0 via actuators_emergency_off()
         * and is no longer kicking IWDG, so reset is imminent (≤400 ms).
         * Without this gate we would race T_WDG and re-arm the heater on
         * every 50 ms tick — defeating the failsafe.
         *
         * Canary still increments so the stall log identifies the actually
         * frozen task (T_STATE or T_ML), not us. T_WDG only kicks when ALL
         * canaries are below CANARY_MAX_MISS, and the truly-stalled task's
         * canary stays frozen — so resuming our canary cannot accidentally
         * cancel the pending reset. */
        if (g_failsafe_active) {
            g_canary_pid++;
            continue;
        }

        /* ---- 1. Read both IR sensors ---- */
        float ir1_c = 0.0f, ir2_c = 0.0f;
        HAL_StatusTypeDef st1 = HAL_ERROR, st2 = HAL_ERROR;

        pid_phase_enter(PHASE_PID_MUTEX_TAKE);
        if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(10)) == pdTRUE) {
            pid_phase_enter(PHASE_PID_IR_READ);
            st1 = tbp_h70_read_target_c(&hi2c1, IR_SENSOR_1_I2C_ADDR_7B, &ir1_c);
            st2 = tbp_h70_read_target_c(&hi2c1, IR_SENSOR_2_I2C_ADDR_7B, &ir2_c);
            xSemaphoreGive(mtx_i2c1);
        }

        /* ---- 2. Drain ctrl_cmd queue (keep last) ---- */
        pid_phase_enter(PHASE_PID_DRAIN_CMD);
        ctrl_cmd_t cmd_in;
        while (xQueueReceive(q_ctrl_to_pid, &cmd_in, 0) == pdTRUE) {
            current_cmd = cmd_in;
        }

        /* ---- 3. Pick PID input = max(IR1, IR2), with fault tracking ---- */
        float measurement;
        bool  ir_ok = false;
        if (st1 == HAL_OK && st2 == HAL_OK) {
            measurement = (ir1_c > ir2_c) ? ir1_c : ir2_c;
            ir_ok = true;
        } else if (st1 == HAL_OK) {
            measurement = ir1_c;
            ir_ok = true;
        } else if (st2 == HAL_OK) {
            measurement = ir2_c;
            ir_ok = true;
        } else {
            measurement = last_valid_meas;  /* keep stale value, count failure */
        }

        pid_phase_enter(PHASE_PID_FAULT_CHK);
        if (ir_ok) {
            last_valid_meas = measurement;
            ir_fail_count = 0u;
        } else {
            ir_fail_count++;
            if (ir_fail_count >= IR_READ_FAIL_LIMIT) {
                pid_phase_enter(PHASE_PID_RECOVERY);
                /* Attempt I2C bus recovery before declaring FAULT.
                 * 3 immediate retries under mutex [C2], then FAULT. */
                bool recovered = false;
                if (xSemaphoreTake(mtx_i2c1, pdMS_TO_TICKS(100)) == pdTRUE) {
                    for (uint8_t att = 0; att < I2C_MAX_RECOVERY; att++) {
                        g_canary_pid++;  /* [C2] keep canary alive during recovery */
                        i2c1_bus_recover(&hi2c1);
                        float test1 = 0.0f, test2 = 0.0f;
                        HAL_StatusTypeDef r1 = tbp_h70_read_target_c(
                            &hi2c1, IR_SENSOR_1_I2C_ADDR_7B, &test1);
                        HAL_StatusTypeDef r2 = tbp_h70_read_target_c(
                            &hi2c1, IR_SENSOR_2_I2C_ADDR_7B, &test2);
                        if (r1 == HAL_OK && r2 == HAL_OK) {
                            recovered = true;
                            ir_fail_count = 0u;
                            break;
                        }
                    }
                    xSemaphoreGive(mtx_i2c1);
                    if (!recovered) {
                        fault_req_t freq = { .reason = FAULT_REASON_SENSOR_TIMEOUT };
                        (void)xQueueSendToBack(q_fault_req, &freq, 0);
                        ir_fail_count = 0u;
                    }
                } else {
                    /* Mutex timeout — recovery impossible. Send FAULT. */
                    fault_req_t freq = { .reason = FAULT_REASON_SENSOR_TIMEOUT };
                    (void)xQueueSendToBack(q_fault_req, &freq, 0);
                    ir_fail_count = 0u;
                }
            }
        }

        /* ---- 4. Over-temperature: raw value check ----
         *
         * [W7] The `overtemp` boolean still kills heater_duty immediately
         * on the first sample above OVERTEMP_HARD_C (existing behavior).
         * The FAULT transition is now gated by OVERTEMP_SUSTAIN_COUNT
         * consecutive exceeding samples (1 second) to reject noise spikes.
         * [W4] Counter is NOT reset after sending FAULT — if the queue
         * was full, the next tick retries immediately. */
        pid_phase_enter(PHASE_PID_OVERTEMP);
        bool overtemp = false;
        if (st1 == HAL_OK && ir1_c > (float)OVERTEMP_HARD_C) overtemp = true;
        if (st2 == HAL_OK && ir2_c > (float)OVERTEMP_HARD_C) overtemp = true;
        {
            static uint32_t overtemp_count = 0;
            if (overtemp) {
                overtemp_count++;
                if (overtemp_count >= OVERTEMP_SUSTAIN_COUNT) {
                    if ((fsm_state_t)g_fsm_state != FSM_FAULT) {
                        fault_req_t freq = { .reason = FAULT_REASON_OVERTEMP };
                        (void)xQueueSendToBack(q_fault_req, &freq, 0);
                    }
                    overtemp_count = OVERTEMP_SUSTAIN_COUNT; /* [C4] clamp, prevent uint32 wrap */
                }
            } else {
                overtemp_count = 0;
            }
        }

        /* ---- 5. FSM-aware control: compute PID only in FORCE_UP ----
         *
         * Bug fix: previously pid_compute ran before FSM state checks,
         * so the integrator accumulated error even when the output was
         * unconditionally overridden to 0 (FORCE_DOWN / deadband).
         * Now we branch on FSM state first and only call pid_compute
         * when the PID actually drives the heater. */
        pid_phase_enter(PHASE_PID_COMPUTE);
        fsm_state_t snap_state = (fsm_state_t)g_fsm_state;
        uint8_t fan_pct = current_cmd.fan_duty_pct;
        led_pattern_t led = (led_pattern_t)current_cmd.led_pattern;
        uint16_t heater_duty = 0;

        if (snap_state == FSM_FAULT) {
            fan_pct = 0;
            led = LED_FLASH_RED;
            pid_reset(&heater_pid);

        } else if (snap_state == FSM_FORCE_UP) {
            /* FU 진입 감지 → phase 강제 reset (clean state from any path:
             * FD→FU, FAULT→FU 등). prev_state는 분기 끝에서 갱신. */
            if (prev_state != FSM_FORCE_UP) {
                fu_phase = FU_PHASE_APPROACH;
            }

            if (current_cmd.pid_enabled && ir_ok && !overtemp) {
                float err = (float)current_cmd.setpoint_c - measurement;

                /* Phase 전이 (sticky, hysteresis) */
                if (fu_phase == FU_PHASE_APPROACH) {
                    if (err <= (float)FU_APPROACH_BAND_C) {
                        fu_phase = FU_PHASE_SETTLE;
                        pid_reset(&heater_pid);
                        pid_set_gains(&heater_pid,
                                      PID_KP_SETTLE,
                                      PID_KI_SETTLE,
                                      PID_KD_SETTLE);
                        rtt_log_str("[t_pid] FU phase: APPROACH->SETTLE");
                    }
                } else { /* SETTLE */
                    if (err > (float)FU_SETTLE_HYSTERESIS_C) {
                        fu_phase = FU_PHASE_APPROACH;
                        rtt_log_str("[t_pid] FU phase: SETTLE->APPROACH (disturbance)");
                    }
                }

                /* Phase별 출력 계산 */
                if (fu_phase == FU_PHASE_APPROACH) {
                    /* Bang-bang: 풀듀티. PID integrator는 호출 안 해서 동결.
                     * Slew limiter(5a)가 0→MAX 1초 ramp로 thermal shock 보호. */
                    heater_duty = (uint16_t)PID_OUTPUT_MAX;
                } else { /* SETTLE */
                    if (measurement > (float)current_cmd.setpoint_c) {
                        /* Setpoint 초과 → 명시적 OFF (FF baseline의 안전판).
                         * pid_compute는 출력 0 클램프라 음수 보정 불가하므로
                         * FF=250이 항상 남아 오버슛 폭주 가능 — 이를 차단. */
                        heater_duty = 0;
                    } else {
                        float pid_out = pid_compute(&heater_pid,
                                                    (float)current_cmd.setpoint_c,
                                                    measurement,
                                                    PID_DT_S);
                        float total = (float)FF_HOLD_DUTY + pid_out;
                        if (total < 0.0f) total = 0.0f;
                        if (total > (float)SETTLE_DUTY_MAX)
                            total = (float)SETTLE_DUTY_MAX;
                        heater_duty = (uint16_t)total;
                    }
                }
            }

        } else if (snap_state == FSM_FORCE_DOWN) {
            float err = measurement - (float)current_cmd.setpoint_c;
            fan_pct = (ir_ok && err > (float)TEMP_DEADBAND_C) ? FAN_DUTY_COOLDOWN_PCT : 0;
        }

        /* prev_state 갱신 — 다음 사이클의 FU 진입 검출용. 모든 분기가
         * 끝난 시점에서 한 번만. */
        prev_state = snap_state;

        /* ---- 5a. Heater output slew-rate limit ---- */
        /* Bypass slew limiter when heater must shut off immediately:
         * FAULT (safety) and FORCE_DOWN (no heating needed).
         * Only ramp UP during FORCE_UP — 하강은 즉시(끄는 건 안전).
         * 단방향 ramp는 APPROACH→SETTLE 전환 시 듀티 급락을 즉시 반영해
         * 오버슛을 줄인다. */
        {
            static uint16_t prev_heater_duty = 0;
            if (snap_state == FSM_FORCE_UP && heater_duty > prev_heater_duty) {
                int16_t delta = (int16_t)heater_duty - (int16_t)prev_heater_duty;
                if (delta > PID_SLEW_LIMIT_PER_CYCLE)
                    heater_duty = prev_heater_duty + PID_SLEW_LIMIT_PER_CYCLE;
            }
            prev_heater_duty = heater_duty;
        }

        /* ---- 6. LED blink while approaching target ----
         *
         * Bug fix: removed pid_reset() that was called on every LED
         * pattern change. This was zeroing the integrator on every
         * FORCE_UP/FORCE_DOWN transition, defeating bumpless transfer.
         * pid_reset is now only called on FAULT entry (above). */
        if (current_cmd.led_pattern != prev_led_pat) {
            reached_target = false;
            prev_led_pat = current_cmd.led_pattern;
        }
        if (ir_ok && !reached_target) {
            if (led == LED_RAMP_YELLOW &&
                measurement >= (float)(current_cmd.setpoint_c - TEMP_DEADBAND_C))
                reached_target = true;
            if (led == LED_FADE_YELLOW &&
                measurement <= (float)(current_cmd.setpoint_c + TEMP_DEADBAND_C))
                reached_target = true;
        }
        if (!reached_target) {
            if (led == LED_RAMP_YELLOW)  led = LED_RAMP_YELLOW_BLINK;
            if (led == LED_FADE_YELLOW)  led = LED_FADE_YELLOW_BLINK;
        }

        /* ---- 7. Write actuators ---- */
        pid_phase_enter(PHASE_PID_ACTUATE);
        actuators_set_heater_duty(heater_duty);
        actuators_set_fan_duty_pct(fan_pct);
#if DATA_COLLECT_MODE
        /* Recording overlay — FAULT always wins (safety). When recording,
         * fast magenta blink signals SD write failures so the user notices
         * silent corruption immediately instead of post-hoc. */
        if (led != LED_FLASH_RED && sd_logger_is_recording()) {
            led = sd_logger_has_write_error() ? LED_RECORDING_ERROR_BLINK
                                              : LED_RECORDING_BLINK;
        }
#endif
        actuators_set_led_pattern(led);

        /* ---- 8. Heartbeat log (1 Hz) ---- */
        pid_phase_enter(PHASE_PID_HEARTBEAT);
        if ((tick % 20) == 0) {
            rtt_log_hb_s("[t_pid]",
                         " ir_cC=", (int32_t)(measurement * 100.0f),
                         " sp_cC=", (int32_t)current_cmd.setpoint_c * 100,
                         " heat=",  (int32_t)heater_duty,
                         " fan=",   (int32_t)fan_pct);

            /* Structured log to q_log for T_LOGGER (Phase 5). Non-blocking;
             * drop silently if the queue is full — the direct RTT heartbeat
             * above is the primary bring-up output, q_log is the future
             * SD/UART path. */
            log_msg_t lm = {
                .timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                .level        = LOG_LVL_INFO,
                .source       = LOG_SRC_PID,
                .code         = 0,  /* 0 = periodic heartbeat */
                .value        = (int32_t)(measurement * 100.0f),
                .extra        = (int32_t)heater_duty,
            };
            (void)xQueueSendToBack(q_log, &lm, 0);
        }

        g_cycle_ms_pid = (uint32_t)((xTaskGetTickCount() - cycle_start)
                                    * portTICK_PERIOD_MS);

        /* Canary: prove T_PID completed a full loop iteration. */
        g_canary_pid++;
    }
}

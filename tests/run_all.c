#include "unity.h"

/* test_fsm.c */
void test_initial_state_is_force_down(void);
void test_force_down_trig_up_goes_force_up(void);
void test_force_up_trig_down_goes_force_down(void);
void test_force_up_safety_timeout_goes_force_down(void);
void test_force_down_fault_req_goes_fault(void);
void test_force_up_fault_req_goes_fault(void);
void test_fault_is_terminal_for_all_events(void);
void test_none_event_keeps_state(void);
void test_force_down_ignores_trig_down(void);
void test_force_up_ignores_trig_up(void);
void test_state_names_not_null(void);

/* test_pid.c */
void test_pid_zero_error_zero_output(void);
void test_pid_positive_error_positive_output(void);
void test_pid_output_clamped_to_max(void);
void test_pid_output_clamped_to_min(void);
void test_pid_reset_clears_integrator(void);
void test_pid_integral_accumulates(void);
void test_pid_no_derivative_kick_on_first_call(void);

/* test_features.c */
void test_tilt_init_zeros(void);
void test_tilt_stationary_near_zero(void);
void test_tilt_tilted_nonzero(void);
void test_ml_window_init_empty(void);
void test_ml_features_invalid_before_fill(void);
void test_ml_features_valid_after_fill(void);
void test_ml_features_invalid_extreme_accel(void);

/* test_state_table.c */
void test_force_down_output(void);
void test_force_up_output(void);
void test_fault_output(void);
void test_invalid_state_returns_force_down(void);

void setUp(void) {}
void tearDown(void) {}

int main(void)
{
    UNITY_BEGIN();

    /* FSM transitions */
    RUN_TEST(test_initial_state_is_force_down);
    RUN_TEST(test_force_down_trig_up_goes_force_up);
    RUN_TEST(test_force_up_trig_down_goes_force_down);
    RUN_TEST(test_force_up_safety_timeout_goes_force_down);
    RUN_TEST(test_force_down_fault_req_goes_fault);
    RUN_TEST(test_force_up_fault_req_goes_fault);
    RUN_TEST(test_fault_is_terminal_for_all_events);
    RUN_TEST(test_none_event_keeps_state);
    RUN_TEST(test_force_down_ignores_trig_down);
    RUN_TEST(test_force_up_ignores_trig_up);
    RUN_TEST(test_state_names_not_null);

    /* PID controller */
    RUN_TEST(test_pid_zero_error_zero_output);
    RUN_TEST(test_pid_positive_error_positive_output);
    RUN_TEST(test_pid_output_clamped_to_max);
    RUN_TEST(test_pid_output_clamped_to_min);
    RUN_TEST(test_pid_reset_clears_integrator);
    RUN_TEST(test_pid_integral_accumulates);
    RUN_TEST(test_pid_no_derivative_kick_on_first_call);

    /* Features (tilt + ML) */
    RUN_TEST(test_tilt_init_zeros);
    RUN_TEST(test_tilt_stationary_near_zero);
    RUN_TEST(test_tilt_tilted_nonzero);
    RUN_TEST(test_ml_window_init_empty);
    RUN_TEST(test_ml_features_invalid_before_fill);
    RUN_TEST(test_ml_features_valid_after_fill);
    RUN_TEST(test_ml_features_invalid_extreme_accel);

    /* State table */
    RUN_TEST(test_force_down_output);
    RUN_TEST(test_force_up_output);
    RUN_TEST(test_fault_output);
    RUN_TEST(test_invalid_state_returns_force_down);

    return UNITY_END();
}

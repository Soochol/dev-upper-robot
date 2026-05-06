CC      = gcc
CFLAGS  = -Wall -Wextra -I../Core/Inc -Istubs -lm
APP_SRC = ../Core/Src/app

.PHONY: all clean

all: test_pid test_fsm test_features test_trigger test_trigger_ml
	./test_pid
	./test_fsm
	./test_features
	./test_trigger
	./test_trigger_ml

# --- PID ---
test_pid: test_pid.c $(APP_SRC)/pid.c
	$(CC) -o $@ $^ $(CFLAGS)

# --- FSM + state table ---
test_fsm: test_fsm.c $(APP_SRC)/fsm.c $(APP_SRC)/state_table.c
	$(CC) -o $@ $^ $(CFLAGS)

# --- Features (tilt + ML) ---
test_features: test_features.c $(APP_SRC)/features.c
	$(CC) -o $@ $^ $(CFLAGS)

# --- Trigger rule (FreeRTOS stubs in stubs/) ---
test_trigger: test_trigger.c $(APP_SRC)/trigger_rule.c
	$(CC) -o $@ $^ $(CFLAGS)

# --- Trigger ML (mocks score() and rtt_log_hb_s) ---
test_trigger_ml: test_trigger_ml.c $(APP_SRC)/trigger_ml.c
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -f test_pid test_fsm test_features test_trigger test_trigger_ml

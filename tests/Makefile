CC     = gcc
CFLAGS = -Wall -Wextra -I../Core/Inc -lm
SRC    = test_pid.c ../Core/Src/app/pid.c

.PHONY: all clean

all: test_pid
	./test_pid

test_pid: $(SRC)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -f test_pid

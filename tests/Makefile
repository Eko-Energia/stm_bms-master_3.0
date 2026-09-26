CC      := gcc
CFLAGS  := -std=gnu11 -Wall -Wextra -Werror -O1 -g
INC     := -Ifake -I. -I../App/Inc -I../Core/Inc \
           -I../EKO_Drivers/CAN/Inc -I../EKO_Drivers/PWM/Inc \
           -I../EKO_Drivers/LED/Inc -I../EKO_Drivers/Error_Corrutines/Inc
FAKE    := fake/fake_hal.c
APP     := $(filter-out ../App/Src/app.c,$(wildcard ../App/Src/*.c))
DRV     := ../EKO_Drivers/CAN/Src/can_driver.c ../EKO_Drivers/Error_Corrutines/Src/error_handler.c \
           ../EKO_Drivers/PWM/Src/pwm_driver.c
# CAN_DB.c is cantools-generated: only the pack/init helpers this app actually
# calls get used, so the unused 32-bit shift helpers trip -Wunused-function.
# Built into its own object with that one warning relaxed, rather than edited.
CAN_DB_OBJ := can_db.o
LIBS    := -lm
TESTS   := $(patsubst %.c,%,$(wildcard test_*.c))

.PHONY: all clean oracle $(TESTS)
all: $(TESTS) oracle

oracle: test_pack_vectors
	@../.claude/tmp/.venv/bin/python oracle/check_packing.py

$(CAN_DB_OBJ): ../EKO_Drivers/CAN/Src/CAN_DB.c
	@$(CC) $(CFLAGS) -Wno-unused-function $(INC) -c -o $@ $<

# app.c is the only application file with HAL_*Callback definitions and an
# infinite main loop, so it is linked only by the test that exercises it.
test_app_fatal: EXTRA_SRC := ../App/Src/app.c ../EKO_Drivers/LED/Src/led_driver.c

$(TESTS): %: %.c $(FAKE) $(APP) $(DRV) $(CAN_DB_OBJ)
	@$(CC) $(CFLAGS) $(INC) -o $@.bin $^ $(EXTRA_SRC) $(LIBS) && ./$@.bin

clean:
	@rm -f *.bin *.o

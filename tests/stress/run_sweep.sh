#!/bin/sh
# Build and run the exhaustive numeric sweeps.
#
#   ./run_sweep.sh           ASan + UBSan, trapping (the default pass)
#   ./run_sweep.sh strict    adds -fsanitize=implicit-conversion, non-trapping,
#                            to surface silent narrowing conversions
#   ./run_sweep.sh hazard    runs the opt-in divide-by-zero probe
#
# Does not touch tests/Makefile, tests/fake/* or App/*: the modules under test
# are #included by sweep_pure.c, so they are NOT linked in again here.
set -e
cd "$(dirname "$0")"

CC=${CC:-clang}
INC="-I../fake -I../../App/Inc -I../../App/Src -I../../Core/Inc \
     -I../../EKO_Drivers/CAN/Inc -I../../EKO_Drivers/PWM/Inc \
     -I../../EKO_Drivers/LED/Inc -I../../EKO_Drivers/Error_Corrutines/Inc"
WARN="-std=gnu11 -Wall -Wextra -Werror"
# Linked deps only: the fake HAL, the NTC table, and the error/CAN plumbing the
# ADC module reports through. app_adc.c / app_therm.c / jk_protocol.c are
# included as source by sweep_pure.c.
SRC="sweep_pure.c ../fake/fake_hal.c ../../App/Src/bms_calib.c \
     ../../EKO_Drivers/Error_Corrutines/Src/error_handler.c \
     ../../EKO_Drivers/CAN/Src/can_driver.c"
DB=can_db_sweep.o
SAN="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"

# cantools output has unused static shift helpers; same relaxation the
# tests/Makefile uses.
$CC $WARN -Wno-unused-function -O1 -g $SAN $INC -c -o "$DB" \
    ../../EKO_Drivers/CAN/Src/CAN_DB.c

case "${1:-}" in
strict)
    echo "--- strict pass: implicit-conversion checks, non-trapping ---"
    $CC $WARN -O1 -g $INC \
        -fsanitize=address,undefined,implicit-conversion \
        -fno-omit-frame-pointer -o sweep_strict.out $SRC "$DB" -lm
    UBSAN_OPTIONS=print_stacktrace=0:halt_on_error=0 ./sweep_strict.out 2>&1 |
        awk '/runtime error/ { if (!seen[$0]++) print "UBSAN " $0; next } { print }'
    ;;
hazard)
    $CC $WARN -O1 -g $SAN $INC -o sweep_pure.out $SRC "$DB" -lm
    echo "--- opt-in hazard probe (expected to trap) ---"
    ./sweep_pure.out --hazard-zero-fill || echo "exit status $?"
    ;;
*)
    $CC $WARN -O1 -g $SAN $INC -o sweep_pure.out $SRC "$DB" -lm
    ./sweep_pure.out
    ;;
esac

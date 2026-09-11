#!/usr/bin/env bash
#
# Build and run the whole-system soak. Everything is deterministic: the seed is
# printed at the top of the output and can be replayed with --seed=.
#
# usage: tests/stress/run_soak.sh [--quick] [-- <extra soak args>]
#
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TESTS="$(cd "$HERE/.." && pwd)"
ROOT="$(cd "$TESTS/.." && pwd)"
OUT="$HERE/soak_system.bin"

CC="${CC:-gcc}"
SAN="-fsanitize=address,undefined -fno-sanitize-recover=all"
CFLAGS="-std=gnu11 -Wall -Wextra -O1 -g $SAN"
INC="-I$TESTS/fake -I$TESTS -I$ROOT/App/Inc -I$ROOT/Core/Inc \
     -I$ROOT/EKO_Drivers/CAN/Inc -I$ROOT/EKO_Drivers/PWM/Inc \
     -I$ROOT/EKO_Drivers/LED/Inc -I$ROOT/EKO_Drivers/Error_Corrutines/Inc"

# App/Src/app.c is #included by soak_system.c (it is the only way to reach the
# static step() and initAll()), so it must NOT also be compiled separately.
# tests/fake/fake_hal.c is likewise excluded: soak_system.c provides its own
# backend for the same header. Neither shared file is modified.
SRC="$HERE/soak_system.c"
for f in "$ROOT"/App/Src/*.c; do
    [ "$(basename "$f")" = "app.c" ] && continue
    SRC="$SRC $f"
done
SRC="$SRC $ROOT/EKO_Drivers/CAN/Src/can_driver.c"
SRC="$SRC $ROOT/EKO_Drivers/Error_Corrutines/Src/error_handler.c"
SRC="$SRC $ROOT/EKO_Drivers/PWM/Src/pwm_driver.c"
SRC="$SRC $ROOT/EKO_Drivers/LED/Src/led_driver.c"

# CAN_DB.c is cantools-generated; its unused 32-bit shift helpers trip
# -Wunused-function, so it gets its own object with that warning relaxed.
CAN_DB_OBJ="$HERE/can_db_soak.o"

ARGS=()
QUICK=0
for a in "$@"; do
    case "$a" in
        --quick) QUICK=1 ;;
        --) ;;
        *) ARGS+=("$a") ;;
    esac
done
if [ "$QUICK" = "1" ]; then
    ARGS=("--days=0.2" "--daysB=0.2" "--coarse=5" "--c2=200000" "--c3=60000" "--c4=1" ${ARGS[@]+"${ARGS[@]}"})
fi

echo "== building $OUT"
$CC $CFLAGS -Wno-unused-function $INC -c -o "$CAN_DB_OBJ" "$ROOT/EKO_Drivers/CAN/Src/CAN_DB.c" || exit 1
# shellcheck disable=SC2086
$CC $CFLAGS $INC -o "$OUT" $SRC "$CAN_DB_OBJ" -lm || exit 1

export ASAN_OPTIONS="${ASAN_OPTIONS:-abort_on_error=0:detect_stack_use_after_return=1:strict_string_checks=1}"
export UBSAN_OPTIONS="${UBSAN_OPTIONS:-print_stacktrace=1}"

echo "== running the soak"
"$OUT" ${ARGS[@]+"${ARGS[@]}"}
MAIN_RC=$?

echo
echo "== isolated probe: JK_OnRxEvent(513) (runs separately; a sanitizer abort here"
echo "   would otherwise end the soak)"
"$OUT" --probe-jk-oob
PROBE_RC=$?
echo "== probe exit code: $PROBE_RC"

echo "== soak exit code: $MAIN_RC"
exit $MAIN_RC

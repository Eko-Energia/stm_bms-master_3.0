#!/bin/sh
# Build and run the jk_protocol stress harness.
#
#   tests/stress/run_fuzz.sh [iterations_per_population] [seed]
#
# Pass 1: ASan + UBSan, non-recoverable. This is the one that must stay clean.
# Pass 2: the same harness built with -fsanitize=integer in *recoverable* mode.
#         That set includes implicit-integer-truncation and -sign-change, which
#         jk_protocol hits deliberately (uint16 -> int8 temperature narrowing,
#         intentional unsigned wrap in the checksum accumulator), so its output
#         is a diagnostic listing, not a pass/fail gate.
# note: a nonzero exit from the harness means "invariant violations found",
# not a build failure, so this script must not use set -e around the runs.

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
root=$(CDPATH= cd -- "$here/../.." && pwd)
out="$here/build"
mkdir -p "$out"

iters=${1:-2000000}
seed=${2:-0xC0FFEE}

CC=${CC:-clang}
INC="-I$root/App/Inc"
WARN="-std=c11 -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow"
SRC="$here/fuzz_jk.c $root/App/Src/jk_protocol.c"

echo "=== building pass 1: address,undefined (-fno-sanitize-recover=all) ==="
$CC $WARN $INC -O1 -g -fno-omit-frame-pointer \
    -fsanitize=address,undefined -fno-sanitize-recover=all \
    -o "$out/fuzz_jk" $SRC

echo
echo "=== pass 1: ASan + UBSan, $iters iterations per population ==="
ASAN_OPTIONS=abort_on_error=1:allocator_may_return_null=0 \
UBSAN_OPTIONS=print_stacktrace=1 \
    "$out/fuzz_jk" "$iters" "$seed" || rc1=$?
rc1=${rc1:-0}

echo
echo "=== building pass 2: integer sanitizer (diagnostic, recoverable) ==="
$CC $WARN $INC -O1 -g -fno-omit-frame-pointer \
    -fsanitize=integer,implicit-conversion -fsanitize-recover=all \
    -o "$out/fuzz_jk_int" $SRC 2>/dev/null

echo
echo "=== pass 2: integer-sanitizer findings inside jk_protocol.c (unique) ==="
UBSAN_OPTIONS=print_stacktrace=0 \
    "$out/fuzz_jk_int" "$(expr "$iters" / 40)" "$seed" 2>&1 \
    | grep 'jk_protocol\.c' | sed 's/^.*jk_protocol\.c/jk_protocol.c/' \
    | sort | uniq -c | sort -rn || true

exit $rc1

#include "bms_calib.h"

const uint16_t calibNtcCount[101] = {
    1092, 1127, 1162, 1198, 1235, 1272, 1309, 1347, 1385, 1423,
    1462, 1500, 1539, 1578, 1617, 1656, 1696, 1735, 1774, 1814,
    1853, 1892, 1931, 1970, 2009, 2048, 2086, 2124, 2162, 2199,
    2237, 2273, 2310, 2346, 2382, 2417, 2452, 2487, 2521, 2555,
    2588, 2620, 2653, 2684, 2715, 2746, 2776, 2806, 2835, 2863,
    2890, 2919, 2946, 2973, 2998, 3024, 3049, 3073, 3097, 3121,
    3143, 3166, 3188, 3209, 3230, 3251, 3271, 3290, 3310, 3328,
    3346, 3364, 3382, 3399, 3415, 3431, 3447, 3463, 3478, 3493,
    3507, 3520, 3534, 3548, 3561, 3573, 3586, 3597, 3609, 3621,
    3632, 3643, 3653, 3663, 3674, 3683, 3693, 3702, 3711, 3720,
    3728
};

/*
 * countToCenti divides by the gap between adjacent entries, so the table must
 * be strictly increasing. The values are slated for recalibration at bring-up
 * by hand, which is exactly when a flat or inverted pair gets introduced, so
 * app.c checks this at init rather than trusting the table.
 *
 * Not a _Static_assert: reading a const array is not an integer constant
 * expression in C, and the workarounds all duplicate the 101 values.
 */
bool CALIB_NtcCountIsMonotonic(void)
{
    for (uint8_t i = 0u; i < 100u; i++) {
        if (calibNtcCount[i + 1u] <= calibNtcCount[i]) { return false; }
    }
    return true;
}

#include "test_runner.h"
#include "app_timing.h"

TEST(due_fires_once_per_period)
{
    uint32_t last = 0;
    CHECK(!Timing_Due(999u, &last, 1000u));
    CHECK(Timing_Due(1000u, &last, 1000u));
    CHECK(!Timing_Due(1001u, &last, 1000u));
    CHECK(Timing_Due(2000u, &last, 1000u));
}

TEST(due_advances_by_whole_periods_so_cadence_does_not_drift)
{
    uint32_t last = 0;
    CHECK(Timing_Due(1007u, &last, 1000u));
    CHECK_EQ(last, 1000u);              /* not 1007 */
}

TEST(due_resynchronises_after_a_long_stall)
{
    uint32_t last = 0;
    CHECK(Timing_Due(5000u, &last, 1000u));
    CHECK_EQ(last, 5000u);              /* no catch-up burst */
}

TEST(due_survives_the_tick_wrap)
{
    uint32_t last = 0xFFFFFC00u;
    CHECK(Timing_Due(0x00000100u, &last, 1000u));   /* 1280 ms elapsed across the wrap */
}

int main(void)
{
    RUN(due_fires_once_per_period);
    RUN(due_advances_by_whole_periods_so_cadence_does_not_drift);
    RUN(due_resynchronises_after_a_long_stall);
    RUN(due_survives_the_tick_wrap);
    return TEST_SUMMARY();
}

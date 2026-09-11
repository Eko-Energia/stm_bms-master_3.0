#include "test_runner.h"
#include "app_therm.h"
#include "bms_errors.h"
#include "error_handler.h"
#include "CAN_DB.h"

static CAN_InstanceTypeDef inst;
static CAN_HandleTypeDef hcan = { &inst, { DISABLE } };
static struct CAN_scheduledMsgList sched;
static EH_HandleTypeDef eh;

static void setup(void)
{
    Fake_Reset();
    inst.MCR = 0;
    memset(&sched, 0, sizeof sched);
    memset(&eh, 0, sizeof eh);
    EH_init(&eh, &hcan, BMSMASTER_NODE_FRAME_ID, &sched);
    THERM_Init(&eh);
}

/* Deliver one value to every thermistor of every module, then advance a period. */
static void feedAll(uint8_t raw, int periods)
{
    for (int p = 0; p < periods; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            THERM_OnFrame(id, raw);
        }
        THERM_Task();
    }
}

TEST(odd_packs_number_upward)
{
    setup();
    THERM_OnFrame(211u, 40u);       /* PCBCells1_Therm1 */
    THERM_OnFrame(219u, 41u);       /* PCBCells1_Therm9 */
    THERM_Task();
    CHECK_EQ(THERM_Filtered(1u, 1u), 40u);
    CHECK_EQ(THERM_Filtered(1u, 9u), 41u);
}

TEST(even_packs_number_downward)
{
    setup();
    THERM_OnFrame(221u, 50u);       /* PCBCells2_Therm9, not Therm1 */
    THERM_OnFrame(229u, 51u);       /* PCBCells2_Therm1 */
    THERM_Task();
    CHECK_EQ(THERM_Filtered(2u, 9u), 50u);
    CHECK_EQ(THERM_Filtered(2u, 1u), 51u);
}

TEST(the_last_pack_maps_at_both_ends)
{
    setup();
    THERM_OnFrame(271u, 60u);       /* PCBCells7_Therm1 */
    THERM_OnFrame(279u, 61u);       /* PCBCells7_Therm9 */
    THERM_Task();
    CHECK_EQ(THERM_Filtered(7u, 1u), 60u);
    CHECK_EQ(THERM_Filtered(7u, 9u), 61u);
}

TEST(node_frames_and_out_of_range_ids_are_ignored)
{
    setup();
    feedAll(30u, 1);
    const uint8_t before = THERM_Filtered(1u, 1u);
    THERM_OnFrame(210u, 200u);      /* PCBCells1_NODE */
    THERM_OnFrame(280u, 200u);      /* past the last module */
    THERM_OnFrame(1u, 200u);        /* SafeState, wrong bus entirely */
    THERM_Task();
    CHECK_EQ(THERM_Filtered(1u, 1u), before);
}

TEST(a_single_spike_is_rejected_by_the_trimmed_mean)
{
    setup();
    feedAll(64u, 10);
    CHECK_EQ(THERM_Filtered(3u, 5u), 64u);

    /* One corrupt sample: becomes the window max and is discarded. */
    for (uint32_t id = 211u; id <= 279u; id++) {
        if ((id % 10u) == 0u) { continue; }
        THERM_OnFrame(id, (id == 235u) ? 255u : 64u);
    }
    THERM_Task();
    CHECK_EQ(THERM_Filtered(3u, 5u), 64u);
}

TEST(a_real_step_settles_within_ten_periods)
{
    setup();
    feedAll(20u, 10);
    CHECK_EQ(THERM_Filtered(1u, 1u), 20u);
    feedAll(80u, 10);
    CHECK_EQ(THERM_Filtered(1u, 1u), 80u);
}

TEST(three_missed_periods_raise_pack_silent)
{
    setup();
    feedAll(40u, 10);
    CHECK_EQ(eh.activeErrorCount, 0u);

    /* Starve module 4 only; everyone else keeps reporting. */
    for (int p = 0; p < 3; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            if (id >= 241u && id <= 249u) { continue; }
            THERM_OnFrame(id, 40u);
        }
        THERM_Task();
    }
    CHECK(eh.activeErrorCount > 0u);
    CHECK_EQ(THERM_Filtered(4u, 1u), 40u);      /* value held, not zeroed */
}

TEST(two_misses_are_silent_but_the_third_trips_the_fault)
{
    setup();
    feedAll(40u, 10);
    CHECK_EQ(eh.activeErrorCount, 0u);

    /* Starve module 4 for two periods only: below the 3-miss threshold. */
    for (int p = 0; p < 2; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            if (id >= 241u && id <= 249u) { continue; }
            THERM_OnFrame(id, 40u);
        }
        THERM_Task();
    }
    CHECK_EQ(eh.activeErrorCount, 0u);

    /* Third consecutive miss: threshold reached, fault trips. */
    for (uint32_t id = 211u; id <= 279u; id++) {
        if ((id % 10u) == 0u) { continue; }
        if (id >= 241u && id <= 249u) { continue; }
        THERM_OnFrame(id, 40u);
    }
    THERM_Task();
    CHECK(eh.activeErrorCount > 0u);
}

TEST(a_returning_pack_clears_the_fault)
{
    setup();
    feedAll(40u, 10);
    for (int p = 0; p < 3; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u || (id >= 241u && id <= 249u)) { continue; }
            THERM_OnFrame(id, 40u);
        }
        THERM_Task();
    }
    CHECK(eh.activeErrorCount > 0u);
    feedAll(40u, 1);
    CHECK_EQ(eh.activeErrorCount, 0u);
}

TEST(max_raw_tracks_the_hottest_thermistor)
{
    setup();
    feedAll(30u, 10);
    CHECK_EQ(THERM_MaxRaw(), 30u);
    for (int p = 0; p < 10; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            THERM_OnFrame(id, (id == 265u) ? 90u : 30u);
        }
        THERM_Task();
    }
    CHECK_EQ(THERM_MaxRaw(), 90u);
}

TEST(the_hottest_thermistor_is_located_not_just_measured)
{
    setup();
    for (int p = 0; p < 10; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            THERM_OnFrame(id, (id == 265u) ? 90u : 30u);
        }
        THERM_Task();
    }
    CHECK_EQ(THERM_MaxRaw(), 90u);
    /* 265 is module 6, which counts down: offset 5 -> thermistor 5. */
    CHECK_EQ(THERM_MaxModule(), 6u);
    CHECK_EQ(THERM_MaxTherm(), 5u);
    CHECK_EQ(THERM_Filtered(THERM_MaxModule(), THERM_MaxTherm()), THERM_MaxRaw());
}

int main(void)
{
    RUN(odd_packs_number_upward);
    RUN(even_packs_number_downward);
    RUN(the_last_pack_maps_at_both_ends);
    RUN(node_frames_and_out_of_range_ids_are_ignored);
    RUN(a_single_spike_is_rejected_by_the_trimmed_mean);
    RUN(a_real_step_settles_within_ten_periods);
    RUN(three_missed_periods_raise_pack_silent);
    RUN(two_misses_are_silent_but_the_third_trips_the_fault);
    RUN(a_returning_pack_clears_the_fault);
    RUN(max_raw_tracks_the_hottest_thermistor);
    RUN(the_hottest_thermistor_is_located_not_just_measured);
    return TEST_SUMMARY();
}

#include "test_runner.h"
#include "app_therm.h"
#include "bms_errors.h"
#include "error_handler.h"
#include "CAN_DB.h"
#include "CAN2_DB.h"

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

/* The DBC's own frame names, indexed [module - 1][thermistor - 1]. This is the
   only independent oracle for the odd-up / even-down numbering. */
static const uint32_t dbThermIds[THERM_MODULES][THERM_PER_MODULE] = {
    { PCBCELLS1_THERM1_FRAME_ID, PCBCELLS1_THERM2_FRAME_ID, PCBCELLS1_THERM3_FRAME_ID,
      PCBCELLS1_THERM4_FRAME_ID, PCBCELLS1_THERM5_FRAME_ID, PCBCELLS1_THERM6_FRAME_ID,
      PCBCELLS1_THERM7_FRAME_ID, PCBCELLS1_THERM8_FRAME_ID, PCBCELLS1_THERM9_FRAME_ID },
    { PCBCELLS2_THERM1_FRAME_ID, PCBCELLS2_THERM2_FRAME_ID, PCBCELLS2_THERM3_FRAME_ID,
      PCBCELLS2_THERM4_FRAME_ID, PCBCELLS2_THERM5_FRAME_ID, PCBCELLS2_THERM6_FRAME_ID,
      PCBCELLS2_THERM7_FRAME_ID, PCBCELLS2_THERM8_FRAME_ID, PCBCELLS2_THERM9_FRAME_ID },
    { PCBCELLS3_THERM1_FRAME_ID, PCBCELLS3_THERM2_FRAME_ID, PCBCELLS3_THERM3_FRAME_ID,
      PCBCELLS3_THERM4_FRAME_ID, PCBCELLS3_THERM5_FRAME_ID, PCBCELLS3_THERM6_FRAME_ID,
      PCBCELLS3_THERM7_FRAME_ID, PCBCELLS3_THERM8_FRAME_ID, PCBCELLS3_THERM9_FRAME_ID },
    { PCBCELLS4_THERM1_FRAME_ID, PCBCELLS4_THERM2_FRAME_ID, PCBCELLS4_THERM3_FRAME_ID,
      PCBCELLS4_THERM4_FRAME_ID, PCBCELLS4_THERM5_FRAME_ID, PCBCELLS4_THERM6_FRAME_ID,
      PCBCELLS4_THERM7_FRAME_ID, PCBCELLS4_THERM8_FRAME_ID, PCBCELLS4_THERM9_FRAME_ID },
    { PCBCELLS5_THERM1_FRAME_ID, PCBCELLS5_THERM2_FRAME_ID, PCBCELLS5_THERM3_FRAME_ID,
      PCBCELLS5_THERM4_FRAME_ID, PCBCELLS5_THERM5_FRAME_ID, PCBCELLS5_THERM6_FRAME_ID,
      PCBCELLS5_THERM7_FRAME_ID, PCBCELLS5_THERM8_FRAME_ID, PCBCELLS5_THERM9_FRAME_ID },
    { PCBCELLS6_THERM1_FRAME_ID, PCBCELLS6_THERM2_FRAME_ID, PCBCELLS6_THERM3_FRAME_ID,
      PCBCELLS6_THERM4_FRAME_ID, PCBCELLS6_THERM5_FRAME_ID, PCBCELLS6_THERM6_FRAME_ID,
      PCBCELLS6_THERM7_FRAME_ID, PCBCELLS6_THERM8_FRAME_ID, PCBCELLS6_THERM9_FRAME_ID },
    { PCBCELLS7_THERM1_FRAME_ID, PCBCELLS7_THERM2_FRAME_ID, PCBCELLS7_THERM3_FRAME_ID,
      PCBCELLS7_THERM4_FRAME_ID, PCBCELLS7_THERM5_FRAME_ID, PCBCELLS7_THERM6_FRAME_ID,
      PCBCELLS7_THERM7_FRAME_ID, PCBCELLS7_THERM8_FRAME_ID, PCBCELLS7_THERM9_FRAME_ID }
};

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

TEST(the_filter_is_correct_from_the_first_period_onward)
{
    setup();
    /* fill 1 and 2 divide by fill itself, and trimmedMean now returns 0 at
       fill 0 rather than dividing by it. THERM_Task increments the fill before
       every call, so fill 0 is not reachable from here - this pins the
       shortest window that is. */
    feedAll(100u, 1);
    CHECK_EQ(THERM_Filtered(1u, 1u), 100u);
    feedAll(110u, 1);
    CHECK_EQ(THERM_Filtered(1u, 1u), 105u);     /* fill 2: (100 + 110) / 2 */
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

TEST(all_63_database_ids_map_to_their_own_module_and_thermistor)
{
    setup();
    /* The _Static_asserts sample four ids; THERM_OnFrame must agree with the
       database on all 63, or a renumber cross-maps thermistors silently. */
    for (uint8_t m = 0u; m < THERM_MODULES; m++) {
        for (uint8_t t = 0u; t < THERM_PER_MODULE; t++) {
            THERM_OnFrame(dbThermIds[m][t], (uint8_t)(10u + (m * THERM_PER_MODULE) + t));
        }
    }
    THERM_Task();
    for (uint8_t m = 0u; m < THERM_MODULES; m++) {
        for (uint8_t t = 0u; t < THERM_PER_MODULE; t++) {
            CHECK_EQ(THERM_Filtered((uint8_t)(m + 1u), (uint8_t)(t + 1u)),
                     (uint8_t)(10u + (m * THERM_PER_MODULE) + t));
        }
    }
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
    CHECK_EQ(EH_getActiveCount(&eh), 0u);

    /* Starve module 4 only; everyone else keeps reporting. */
    for (int p = 0; p < 3; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            if (id >= 241u && id <= 249u) { continue; }
            THERM_OnFrame(id, 40u);
        }
        THERM_Task();
    }
    CHECK(EH_getActiveCount(&eh) > 0u);
    CHECK_EQ(THERM_Filtered(4u, 1u), 40u);      /* value held, not zeroed */
}

TEST(two_misses_are_silent_but_the_third_trips_the_fault)
{
    setup();
    feedAll(40u, 10);
    CHECK_EQ(EH_getActiveCount(&eh), 0u);

    /* Starve module 4 for two periods only: below the 3-miss threshold. */
    for (int p = 0; p < 2; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            if (id >= 241u && id <= 249u) { continue; }
            THERM_OnFrame(id, 40u);
        }
        THERM_Task();
    }
    CHECK_EQ(EH_getActiveCount(&eh), 0u);

    /* Third consecutive miss: threshold reached, fault trips. */
    for (uint32_t id = 211u; id <= 279u; id++) {
        if ((id % 10u) == 0u) { continue; }
        if (id >= 241u && id <= 249u) { continue; }
        THERM_OnFrame(id, 40u);
    }
    THERM_Task();
    CHECK(EH_getActiveCount(&eh) > 0u);
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
    CHECK(EH_getActiveCount(&eh) > 0u);
    feedAll(40u, 1);
    CHECK_EQ(EH_getActiveCount(&eh), 0u);
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

/* Everyone healthy except one thermistor held at `odd`. */
static void feedAllExcept(uint32_t oddId, uint8_t normal, uint8_t odd, int periods)
{
    for (int p = 0; p < periods; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            THERM_OnFrame(id, (id == oddId) ? odd : normal);
        }
        THERM_Task();
    }
}

static const EH_ActiveError *findError(uint16_t code)
{
    for (uint8_t i = 0u; i < eh.activeErrorCount; i++) {
        if (eh.activeErrors[i].pendingClear) { continue; }
        if (eh.activeErrors[i].errorCode == code) { return &eh.activeErrors[i]; }
    }
    return NULL;
}

TEST(a_floored_thermistor_is_reported_instead_of_silently_reading_cold)
{
    setup();
    feedAll(40u, 10);
    CHECK_EQ(THERM_SaturatedCount(), 0u);

    /* 229 is PCBCells2_Therm1. An open sensor floors the PCBCells lookup at
       0 degC, which reaches us as 0 after the de-bias. */
    feedAllExcept(229u, 40u, 0u, 10);
    CHECK_EQ(THERM_SaturatedCount(), 1u);
    CHECK_EQ(THERM_Filtered(2u, 1u), 0u);          /* the value still reads cold */

    const EH_ActiveError *e = findError(BMS_ERR_CAN2_THERM_SATURATED);
    CHECK(e != NULL);
    CHECK_EQ(e->severity, ERROR_SEVERITY_WARNING);
    CHECK_EQ(e->specificData[0], 2u);              /* module */
    CHECK_EQ(e->specificData[1], 1u);              /* thermistor */
    CHECK_EQ(e->specificData[2], 0u);              /* 0 = floor */
    CHECK_EQ(e->specificData[3], 1u);              /* how many are saturated */
}

TEST(a_saturated_thermistor_is_reported_alongside_the_over_temperature)
{
    setup();
    /* 254 is what wire byte 123 de-biases to: the lookup's 100 degC ceiling. */
    feedAllExcept(211u, 40u, 254u, 10);
    CHECK_EQ(THERM_SaturatedCount(), 1u);
    CHECK_EQ(THERM_MaxRaw(), 254u);                /* still the hottest: may be real */
    CHECK_EQ(THERM_MaxModule(), 1u);
    CHECK_EQ(THERM_MaxTherm(), 1u);

    const EH_ActiveError *e = findError(BMS_ERR_CAN2_THERM_SATURATED);
    CHECK(e != NULL);
    CHECK_EQ(e->specificData[2], 1u);              /* 1 = ceiling */
}

TEST(a_module_that_never_transmitted_is_not_a_floored_sensor)
{
    setup();
    /* Module 4 never speaks: its windows are all zero, which must read as
       silent (code 3), never as nine floored thermistors. */
    for (int p = 0; p < 5; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u || (id >= 241u && id <= 249u)) { continue; }
            THERM_OnFrame(id, 40u);
        }
        THERM_Task();
    }
    CHECK(findError(BMS_ERR_CAN2_MODULE_SILENT) != NULL);
    CHECK_EQ(THERM_SaturatedCount(), 0u);
    CHECK(findError(BMS_ERR_CAN2_THERM_SATURATED) == NULL);
}

TEST(the_ceiling_is_reported_in_preference_to_a_floor)
{
    setup();
    /* A floor on module 1 and a ceiling on module 7: the ceiling is the end
       that can also be a genuine thermal event, so it must be the one named. */
    for (int p = 0; p < 10; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            uint8_t v = 40u;
            if (id == 211u) { v = 0u; }
            if (id == 279u) { v = 254u; }
            THERM_OnFrame(id, v);
        }
        THERM_Task();
    }
    CHECK_EQ(THERM_SaturatedCount(), 2u);
    const EH_ActiveError *e = findError(BMS_ERR_CAN2_THERM_SATURATED);
    CHECK(e != NULL);
    CHECK_EQ(e->specificData[2], 1u);              /* ceiling */
    CHECK_EQ(e->specificData[0], 7u);              /* module 7 */
    CHECK_EQ(e->specificData[3], 2u);              /* both counted */
}

TEST(saturation_clears_when_the_sensor_recovers)
{
    setup();
    feedAllExcept(229u, 40u, 0u, 10);
    CHECK(findError(BMS_ERR_CAN2_THERM_SATURATED) != NULL);
    feedAll(40u, 10);
    CHECK_EQ(THERM_SaturatedCount(), 0u);
    CHECK(findError(BMS_ERR_CAN2_THERM_SATURATED) == NULL);
}

int main(void)
{
    RUN(the_filter_is_correct_from_the_first_period_onward);
    RUN(odd_packs_number_upward);
    RUN(even_packs_number_downward);
    RUN(the_last_pack_maps_at_both_ends);
    RUN(all_63_database_ids_map_to_their_own_module_and_thermistor);
    RUN(node_frames_and_out_of_range_ids_are_ignored);
    RUN(a_single_spike_is_rejected_by_the_trimmed_mean);
    RUN(a_real_step_settles_within_ten_periods);
    RUN(three_missed_periods_raise_pack_silent);
    RUN(two_misses_are_silent_but_the_third_trips_the_fault);
    RUN(a_returning_pack_clears_the_fault);
    RUN(max_raw_tracks_the_hottest_thermistor);
    RUN(the_hottest_thermistor_is_located_not_just_measured);
    RUN(a_floored_thermistor_is_reported_instead_of_silently_reading_cold);
    RUN(a_saturated_thermistor_is_reported_alongside_the_over_temperature);
    RUN(a_module_that_never_transmitted_is_not_a_floored_sensor);
    RUN(the_ceiling_is_reported_in_preference_to_a_floor);
    RUN(saturation_clears_when_the_sensor_recovers);
    return TEST_SUMMARY();
}

#include "test_runner.h"
#include "app_thermal.h"
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
    THERMAL_Init(&eh);
}

TEST(healthy_temperatures_raise_nothing)
{
    setup();
    THERMAL_Evaluate(true, 2500u, 100u, 1u, 1u);
    CHECK_EQ(EH_getActiveCount(&eh), 0u);
}

TEST(board_overtemperature_raises_and_clears_with_hysteresis)
{
    setup();
    THERMAL_Evaluate(true, 6500u, 100u, 1u, 1u);        /* 65 degC */
    CHECK_EQ(EH_getActiveCount(&eh), 1u);

    THERMAL_Evaluate(true, 5800u, 100u, 1u, 1u);        /* 58 degC: inside the hysteresis band */
    CHECK_EQ(EH_getActiveCount(&eh), 1u);

    THERMAL_Evaluate(true, 5000u, 100u, 1u, 1u);        /* 50 degC: clearly clear */
    CHECK_EQ(EH_getActiveCount(&eh), 0u);
}

TEST(an_invalid_board_reading_is_not_judged)
{
    setup();
    THERMAL_Evaluate(false, 9000u, 100u, 1u, 1u);       /* filter has not filled yet */
    CHECK_EQ(EH_getActiveCount(&eh), 0u);
}

TEST(pack_overtemperature_uses_the_hottest_thermistor)
{
    setup();
    /* 0.39216 degC per count, so 60 degC is raw 153. */
    THERMAL_Evaluate(true, 2500u, 153u, 1u, 1u);
    CHECK_EQ(EH_getActiveCount(&eh), 0u);          /* at the limit, not above it */

    THERMAL_Evaluate(true, 2500u, 160u, 1u, 1u);
    CHECK_EQ(EH_getActiveCount(&eh), 1u);

    THERMAL_Evaluate(true, 2500u, 148u, 1u, 1u);        /* inside the hysteresis band */
    CHECK_EQ(EH_getActiveCount(&eh), 1u);

    THERMAL_Evaluate(true, 2500u, 140u, 1u, 1u);
    CHECK_EQ(EH_getActiveCount(&eh), 0u);
}

TEST(the_two_sensors_are_reported_independently)
{
    setup();
    THERMAL_Evaluate(true, 6500u, 160u, 1u, 1u);
    CHECK_EQ(EH_getActiveCount(&eh), 2u);          /* TEMP_HIGH and CAN2_TEMP_HIGH */

    THERMAL_Evaluate(true, 5000u, 160u, 1u, 1u);
    CHECK_EQ(EH_getActiveCount(&eh), 1u);          /* only the pack remains */
}

TEST(pack_overtemperature_names_the_thermistor)
{
    setup();
    THERMAL_Evaluate(true, 2500u, 160u, 5u, 7u);
    CHECK_EQ(EH_getActiveCount(&eh), 1u);
    CHECK_EQ(eh.activeErrors[0].errorCode, BMS_ERR_CAN2_TEMP_HIGH);
    /* Spec 6.4: pack, thermistor, raw count - not the raw count alone. */
    CHECK_EQ(eh.activeErrors[0].specificDataLen, 3u);
    CHECK_EQ(eh.activeErrors[0].specificData[0], 5u);
    CHECK_EQ(eh.activeErrors[0].specificData[1], 7u);
    CHECK_EQ(eh.activeErrors[0].specificData[2], 160u);
}

TEST(the_bench_capture_raises_can2_temp_high)
{
    Fake_Reset();
    memset(&eh, 0, sizeof eh); memset(&sched, 0, sizeof sched);
    EH_init(&eh, &hcan, BMSMASTER_NODE_FRAME_ID, &sched);
    THERM_Init(&eh); THERMAL_Init(&eh);

    /* His readings: most sensors 171 (67.06 C), some unfitted at 0. */
    for (int pass = 0; pass < 12; pass++) {
        for (uint8_t m = 1; m <= 7; m++)
            for (uint8_t t = 1; t <= 9; t++) {
                const uint8_t raw = ((m + t) % 3 == 0) ? 0u : 171u;  /* scattered zeros */
                if (raw != 0u) THERM_OnFrame(210u + (m - 1u) * 10u + ((m & 1u) ? t : (10u - t)), raw);
            }
        THERM_Task();
    }
    printf("    THERM_MaxRaw() = %u  (threshold is 153)\n", THERM_MaxRaw());
    CHECK(THERM_MaxRaw() > 153u);

    THERMAL_Evaluate(true, 2521u, THERM_MaxRaw(), THERM_MaxModule(), THERM_MaxTherm());
    /* The scattered unfitted sensors raise CAN2_MODULE_SILENT as well, so assert
       membership: the bench capture has both faults active at once. */
    bool tempHigh = false, moduleSilent = false;
    for (uint8_t i = 0u; i < eh.activeErrorCount; i++) {
        if (eh.activeErrors[i].errorCode == BMS_ERR_CAN2_TEMP_HIGH)     { tempHigh = true; }
        if (eh.activeErrors[i].errorCode == BMS_ERR_CAN2_MODULE_SILENT) { moduleSilent = true; }
    }
    CHECK(tempHigh);
    CHECK(moduleSilent);
}

int main(void)
{
    RUN(healthy_temperatures_raise_nothing);
    RUN(board_overtemperature_raises_and_clears_with_hysteresis);
    RUN(an_invalid_board_reading_is_not_judged);
    RUN(pack_overtemperature_uses_the_hottest_thermistor);
    RUN(the_two_sensors_are_reported_independently);
    RUN(pack_overtemperature_names_the_thermistor);
    RUN(the_bench_capture_raises_can2_temp_high);
    return TEST_SUMMARY();
}

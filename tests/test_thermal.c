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

/* A cleared fault stays listed until it has reached the bus once, so standing
   means present and not already flagged for drop. */
static const EH_ActiveError *standing(uint16_t code)
{
    for (uint8_t i = 0u; i < eh.activeErrorCount; i++) {
        if (eh.activeErrors[i].errorCode == code
            && eh.activeErrors[i].pendingClear == 0u) { return &eh.activeErrors[i]; }
    }
    return NULL;
}

/* 0.39216 degC per count: raw 122 is 47.84 degC, 123 is 48.24, 132 is 51.77
   and 133 is 52.16. The limits round up, so they trip at or above themselves. */
TEST(the_warning_trips_at_its_limit_and_not_a_count_below)
{
    setup();
    THERMAL_Evaluate(122u, 1u, 1u);
    CHECK_EQ(EH_getActiveCount(&eh), 0u);

    THERMAL_Evaluate(123u, 1u, 1u);
    CHECK_EQ(EH_getActiveCount(&eh), 1u);
    CHECK(standing(BMS_ERR_CAN2_TEMP_HIGH) != NULL);
    CHECK_EQ(standing(BMS_ERR_CAN2_TEMP_HIGH)->severity, ERROR_SEVERITY_WARNING);
}

TEST(the_error_trips_at_its_limit_and_not_a_count_below)
{
    setup();
    THERMAL_Evaluate(132u, 1u, 1u);
    CHECK(standing(BMS_ERR_CAN2_TEMP_HIGH) != NULL);     /* still only the warning */
    CHECK(standing(BMS_ERR_CAN2_TEMP_EXTREME) == NULL);

    THERMAL_Evaluate(133u, 1u, 1u);
    CHECK(standing(BMS_ERR_CAN2_TEMP_EXTREME) != NULL);
    CHECK_EQ(standing(BMS_ERR_CAN2_TEMP_EXTREME)->severity, ERROR_SEVERITY_ERROR);
}

/* Escalating, not cumulative: above the error limit the warning would be
   redundant and would cost a heartbeat slot of its own. */
TEST(exactly_one_limit_stands_at_a_time)
{
    setup();
    THERMAL_Evaluate(200u, 1u, 1u);                 /* 78.4 degC */
    CHECK_EQ(EH_getActiveCount(&eh), 1u);
    CHECK(standing(BMS_ERR_CAN2_TEMP_EXTREME) != NULL);
    CHECK(standing(BMS_ERR_CAN2_TEMP_HIGH) == NULL);

    THERMAL_Evaluate(125u, 1u, 1u);                 /* 49.0 degC: back to warning */
    CHECK_EQ(EH_getActiveCount(&eh), 1u);
    CHECK(standing(BMS_ERR_CAN2_TEMP_HIGH) != NULL);
    CHECK(standing(BMS_ERR_CAN2_TEMP_EXTREME) == NULL);
}

TEST(neither_limit_uses_hysteresis)
{
    setup();
    THERMAL_Evaluate(200u, 1u, 1u);
    CHECK_EQ(EH_getActiveCount(&eh), 1u);

    /* One count under the warning clears both on the same call. */
    THERMAL_Evaluate(122u, 1u, 1u);
    CHECK_EQ(EH_getActiveCount(&eh), 0u);
}

TEST(both_limits_name_the_thermistor)
{
    setup();
    THERMAL_Evaluate(130u, 5u, 7u);                 /* warning band */
    const EH_ActiveError *w = standing(BMS_ERR_CAN2_TEMP_HIGH);
    CHECK(w != NULL);
    /* Spec 6.4: pack, thermistor, raw count - not the raw count alone. */
    CHECK_EQ(w->specificDataLen, 3u);
    CHECK_EQ(w->specificData[0], 5u);
    CHECK_EQ(w->specificData[1], 7u);
    CHECK_EQ(w->specificData[2], 130u);

    setup();
    THERMAL_Evaluate(160u, 3u, 2u);                 /* error band */
    const EH_ActiveError *e = standing(BMS_ERR_CAN2_TEMP_EXTREME);
    CHECK(e != NULL);
    CHECK_EQ(e->specificDataLen, 3u);
    CHECK_EQ(e->specificData[0], 3u);
    CHECK_EQ(e->specificData[1], 2u);
    CHECK_EQ(e->specificData[2], 160u);
}

/* A disabled position reports 0 and never reaches the maximum, so it cannot
   raise either limit however hot its frame claims to be. */
TEST(a_disabled_thermistor_cannot_raise_either_limit)
{
    Fake_Reset();
    memset(&eh, 0, sizeof eh); memset(&sched, 0, sizeof sched);
    EH_init(&eh, &hcan, BMSMASTER_NODE_FRAME_ID, &sched);
    THERM_Init(&eh); THERMAL_Init(&eh);

    for (int pass = 0; pass < 12; pass++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            THERM_OnFrame(id, 100u);                /* 39.2 degC, comfortably cool */
        }
        THERM_OnFrame(261u, 255u);                  /* PCB6Therm9: disabled */
        THERM_OnFrame(236u, 255u);                  /* PCB3Therm6: disabled */
        THERM_Task();
    }
    CHECK_EQ(THERM_MaxRaw(), 100u);
    THERMAL_Evaluate(THERM_MaxRaw(), THERM_MaxModule(), THERM_MaxTherm());
    CHECK(standing(BMS_ERR_CAN2_TEMP_HIGH) == NULL);
    CHECK(standing(BMS_ERR_CAN2_TEMP_EXTREME) == NULL);
}

TEST(the_bench_capture_raises_the_error_limit)
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
    CHECK(THERM_MaxRaw() > 133u);

    THERMAL_Evaluate(THERM_MaxRaw(), THERM_MaxModule(), THERM_MaxTherm());
    /* The scattered unfitted sensors raise CAN2_MODULE_SILENT as well, so assert
       membership: the bench capture has both faults active at once. */
    CHECK(standing(BMS_ERR_CAN2_TEMP_EXTREME) != NULL);
    CHECK(standing(BMS_ERR_CAN2_MODULE_SILENT) != NULL);
}

int main(void)
{
    RUN(the_warning_trips_at_its_limit_and_not_a_count_below);
    RUN(the_error_trips_at_its_limit_and_not_a_count_below);
    RUN(exactly_one_limit_stands_at_a_time);
    RUN(neither_limit_uses_hysteresis);
    RUN(both_limits_name_the_thermistor);
    RUN(a_disabled_thermistor_cannot_raise_either_limit);
    RUN(the_bench_capture_raises_the_error_limit);
    return TEST_SUMMARY();
}

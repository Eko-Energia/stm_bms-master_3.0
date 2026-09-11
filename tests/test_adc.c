#include "test_runner.h"
#include "app_adc.h"
#include "bms_calib.h"
#include "bms_errors.h"
#include "error_handler.h"
#include "CAN_DB.h"

static CAN_InstanceTypeDef inst;
static CAN_HandleTypeDef hcan = { &inst, { DISABLE } };
static struct CAN_scheduledMsgList sched;
static EH_HandleTypeDef eh;
static volatile uint16_t buf[3];

#define CH_TEMP 0
#define CH_CURR 1
#define CH_VOLT 2

static void setup(void)
{
    Fake_Reset();
    inst.MCR = 0;
    memset(&sched, 0, sizeof sched);
    memset(&eh, 0, sizeof eh);
    EH_init(&eh, &hcan, BMSMASTER_NODE_FRAME_ID, &sched);
    ADC_Init(buf, &eh);
}

/* Push one identical sample set through the filter n times, at tick nowMs. */
static void feedAt(uint32_t nowMs, uint16_t temp, uint16_t curr, uint16_t volt, int n)
{
    for (int i = 0; i < n; i++) {
        buf[CH_TEMP] = temp; buf[CH_CURR] = curr; buf[CH_VOLT] = volt;
        ADC_OnConvComplete();
        ADC_Task(nowMs);
    }
}

static void feed(uint16_t temp, uint16_t curr, uint16_t volt, int n)
{
    feedAt(0u, temp, curr, volt, n);
}

static int activeCount(void) { return (int)eh.activeErrorCount; }

/* Index of an active error by code, or -1. Codes 8 and 11 are separate faults
   with separate causes, so the tests must be able to tell them apart. */
static int errorIndex(uint16_t code)
{
    for (uint8_t i = 0u; i < eh.activeErrorCount; i++) {
        if (eh.activeErrors[i].errorCode == code) { return (int)i; }
    }
    return -1;
}

TEST(the_filter_is_correct_from_the_first_sample_onward)
{
    setup();
    /* fill 1 and 2 divide by fill itself, and trimmedMean now returns 0 at
       fill 0 rather than dividing by it. ADC_Task increments the fill before
       every call, so fill 0 is not reachable from here - this pins the
       shortest window that is. */
    feed(calibNtcCount[25], 2108u, 3000u, 1);
    CHECK_EQ(ADC_PackDecivolts(), 686u);
    CHECK_EQ(ADC_TempCenti(), 2500u);
    CHECK_EQ(ADC_PackDeciamps(), 0);

    feed(calibNtcCount[25], 2108u, 3010u, 1);   /* fill 2: plain mean of both */
    CHECK_EQ(ADC_PackDecivolts(), 687u);        /* mean count 3005 */
}

TEST(not_ready_until_the_window_has_filled)
{
    setup();
    feed(calibNtcCount[25], 2108u, 3000u, 9);
    CHECK(!ADC_Ready());
    feed(calibNtcCount[25], 2108u, 3000u, 1);
    CHECK(ADC_Ready());
}

TEST(trimmed_mean_discards_a_single_outlier_entirely)
{
    setup();
    feed(calibNtcCount[25], 2108u, 3000u, 10);
    const uint16_t clean = ADC_PackDecivolts();

    /* One wild sample in a full window must not move the output at all:
       it becomes the max and is subtracted before dividing by 8. */
    buf[CH_TEMP] = calibNtcCount[25]; buf[CH_CURR] = 2108u; buf[CH_VOLT] = 4095u;
    ADC_OnConvComplete();
    ADC_Task(0u);
    CHECK_EQ(ADC_PackDecivolts(), clean);
}

TEST(pack_voltage_converts_with_rounding)
{
    setup();
    feed(calibNtcCount[25], 2108u, 3000u, 10);
    /* (3000 * 228554 + 500000) / 1000000 = 686 decivolts = 68.6 V */
    CHECK_EQ(ADC_PackDecivolts(), 686u);
}

TEST(current_is_signed_around_the_zero_offset)
{
    setup();
    feed(calibNtcCount[25], 2108u, 3000u, 10);
    CHECK_EQ(ADC_PackDeciamps(), 0);

    feed(calibNtcCount[25], 2112u, 3000u, 10);      /* +4 counts = +1.0 A */
    CHECK_EQ(ADC_PackDeciamps(), 10);

    feed(calibNtcCount[25], 2104u, 3000u, 10);      /* -4 counts = -1.0 A */
    CHECK_EQ(ADC_PackDeciamps(), -10);
}

TEST(current_clamps_and_faults_both_directions)
{
    setup();
    /* count 4095: (4095-2108)*5 = 9935, rounded/2 = 4968 da, well past +3000 */
    feed(calibNtcCount[25], 4095u, 3000u, 10);
    CHECK_EQ(ADC_PackDeciamps(), 3000);
    CHECK(activeCount() > 0);

    feed(calibNtcCount[25], 2108u, 3000u, 10);      /* back in range */
    CHECK_EQ(activeCount(), 0);

    /* count 0: (0-2108)*5 = -10540, rounded/2 = -5270 da, well past -3000 */
    feed(calibNtcCount[25], 0u, 3000u, 10);
    CHECK_EQ(ADC_PackDeciamps(), -3000);
    CHECK(activeCount() > 0);

    feed(calibNtcCount[25], 2108u, 3000u, 10);      /* back in range */
    CHECK_EQ(activeCount(), 0);
}

TEST(trimmed_mean_drops_exactly_one_min_and_one_max)
{
    setup();
    /* 3 samples tied at 1000 (min), 4 at 2108, 3 tied at 4000 (max).
       Trim one min and one max: (3*1000+4*2108+3*4000-1000-4000)/8 = 2304.
       delta = 2304-2108 = 196; da = 196*5 = 980; (980+1)/2 = 490. */
    static const uint16_t curr[10] = { 1000, 1000, 1000, 2108, 2108, 2108, 2108, 4000, 4000, 4000 };
    for (int i = 0; i < 10; i++) {
        buf[CH_TEMP] = calibNtcCount[25];
        buf[CH_CURR] = curr[i];
        buf[CH_VOLT] = 3000u;
        ADC_OnConvComplete();
        ADC_Task(0u);
    }
    CHECK_EQ(ADC_PackDeciamps(), 490);
}

TEST(ntc_guard_band_boundaries_do_not_fault)
{
    setup();
    feed(200u, 2108u, 3000u, 10);                   /* exactly the open-band edge */
    CHECK_EQ(activeCount(), 0);
    CHECK_EQ(ADC_TempCenti(), 0u);

    setup();
    feed(4000u, 2108u, 3000u, 10);                  /* exactly the short-band edge */
    CHECK_EQ(activeCount(), 0);
    CHECK_EQ(ADC_TempCenti(), 10000u);
}

TEST(temperature_matches_the_table_at_its_anchor_points)
{
    setup();
    feed(calibNtcCount[0], 2108u, 3000u, 10);
    CHECK_EQ(ADC_TempCenti(), 0u);

    feed(calibNtcCount[25], 2108u, 3000u, 10);
    CHECK_EQ(ADC_TempCenti(), 2500u);

    feed(calibNtcCount[60], 2108u, 3000u, 10);
    CHECK_EQ(ADC_TempCenti(), 6000u);

    feed(calibNtcCount[100], 2108u, 3000u, 10);
    CHECK_EQ(ADC_TempCenti(), 10000u);
}

TEST(temperature_interpolates_between_anchors)
{
    setup();
    const uint16_t mid = (uint16_t)((calibNtcCount[30] + calibNtcCount[31]) / 2u);
    feed(mid, 2108u, 3000u, 10);
    /* Halfway between 30 and 31 degC, within one interpolation step. */
    CHECK(ADC_TempCenti() >= 3040u && ADC_TempCenti() <= 3060u);
}

TEST(an_open_or_shorted_ntc_raises_a_sensor_fault)
{
    setup();
    feed(150u, 2108u, 3000u, 10);                   /* far below table start */
    CHECK(activeCount() > 0);
    CHECK(errorIndex(BMS_ERR_TEMP_SENSOR_FAULT) >= 0);
    CHECK(errorIndex(BMS_ERR_ADC_STALLED) < 0);     /* the stream is fine; the sensor is not */
    CHECK_EQ(ADC_TempCenti(), 0u);

    setup();
    feed(4050u, 2108u, 3000u, 10);                  /* far above table end */
    CHECK(activeCount() > 0);
    CHECK(errorIndex(BMS_ERR_TEMP_SENSOR_FAULT) >= 0);
    CHECK(errorIndex(BMS_ERR_ADC_STALLED) < 0);
    CHECK_EQ(ADC_TempCenti(), 10000u);
}

TEST(genuinely_cold_is_not_a_sensor_fault)
{
    setup();
    feed(500u, 2108u, 3000u, 10);                   /* below 0 degC, above the open band */
    CHECK_EQ(activeCount(), 0);
    CHECK_EQ(ADC_TempCenti(), 0u);
}

TEST(out_of_range_voltage_clamps_and_faults)
{
    setup();
    feed(calibNtcCount[25], 2108u, 2000u, 10);      /* about 45.7 V, below the 63 V floor */
    CHECK_EQ(ADC_PackDecivolts(), 630u);            /* clamped to the DBC minimum */
    CHECK(activeCount() > 0);

    feed(calibNtcCount[25], 2108u, 3000u, 10);      /* back in range */
    CHECK_EQ(activeCount(), 0);

    /* count 4095: (4095*228554+500000)/1000000 = 936 dV, well past the 87 V ceiling */
    feed(calibNtcCount[25], 2108u, 4095u, 10);
    CHECK_EQ(ADC_PackDecivolts(), 870u);            /* clamped to the DBC maximum */
    CHECK(activeCount() > 0);

    feed(calibNtcCount[25], 2108u, 3000u, 10);      /* back in range */
    CHECK_EQ(activeCount(), 0);
}

TEST(in_range_values_raise_nothing)
{
    setup();
    feed(calibNtcCount[25], 2108u, 3000u, 10);
    CHECK_EQ(activeCount(), 0);
}

TEST(a_stalled_conversion_stream_is_reported_and_stops_being_ready)
{
    setup();
    feed(calibNtcCount[25], 2108u, 3000u, 10);
    CHECK(ADC_Ready());
    CHECK_EQ(activeCount(), 0);

    ADC_Task(50u);                          /* no scan yet, inside the window */
    CHECK(ADC_Ready());
    CHECK_EQ(activeCount(), 0);

    /* The DMA has stopped: the last values must stop being trusted. */
    ADC_Task(100u);
    CHECK(!ADC_Ready());
    CHECK_EQ(activeCount(), 1);
    /* A dead conversion stream is not a wiring fault: the sensor is probably
       fine and the DMA is not, so it gets its own code. */
    const int idx = errorIndex(BMS_ERR_ADC_STALLED);
    CHECK(idx >= 0);
    CHECK(errorIndex(BMS_ERR_TEMP_SENSOR_FAULT) < 0);
    if (idx >= 0) {
        CHECK_EQ(eh.activeErrors[idx].specificDataLen, 2u);
        /* Idle time in ms, u16 little-endian: 100 ms at the moment of detection. */
        CHECK_EQ((uint16_t)(eh.activeErrors[idx].specificData[0] |
                            ((uint16_t)eh.activeErrors[idx].specificData[1] << 8)), 100u);
    }

    feedAt(120u, calibNtcCount[25], 2108u, 3000u, 1);      /* the stream restarts */
    CHECK(ADC_Ready());
    CHECK_EQ(activeCount(), 0);
    CHECK(errorIndex(BMS_ERR_ADC_STALLED) < 0);
}

TEST(the_liveness_check_survives_the_tick_wrap)
{
    setup();
    const uint32_t nearWrap = 0xFFFFFFFFu - 20u;
    feedAt(nearWrap, calibNtcCount[25], 2108u, 3000u, 10);
    CHECK(ADC_Ready());

    ADC_Task(nearWrap + 50u);               /* wraps past 0xFFFFFFFF to 29 */
    CHECK(ADC_Ready());
    CHECK_EQ(activeCount(), 0);

    ADC_Task(nearWrap + 120u);
    CHECK(!ADC_Ready());
    CHECK(errorIndex(BMS_ERR_ADC_STALLED) >= 0);
}

int main(void)
{
    RUN(the_filter_is_correct_from_the_first_sample_onward);
RUN(not_ready_until_the_window_has_filled);
    RUN(trimmed_mean_discards_a_single_outlier_entirely);
    RUN(pack_voltage_converts_with_rounding);
    RUN(current_is_signed_around_the_zero_offset);
    RUN(current_clamps_and_faults_both_directions);
    RUN(trimmed_mean_drops_exactly_one_min_and_one_max);
    RUN(temperature_matches_the_table_at_its_anchor_points);
    RUN(temperature_interpolates_between_anchors);
    RUN(an_open_or_shorted_ntc_raises_a_sensor_fault);
    RUN(genuinely_cold_is_not_a_sensor_fault);
    RUN(ntc_guard_band_boundaries_do_not_fault);
    RUN(out_of_range_voltage_clamps_and_faults);
    RUN(in_range_values_raise_nothing);
    RUN(a_stalled_conversion_stream_is_reported_and_stops_being_ready);
    RUN(the_liveness_check_survives_the_tick_wrap);
    return TEST_SUMMARY();
}

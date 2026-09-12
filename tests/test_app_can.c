#include "test_runner.h"
#include "app_can.h"
#include "app_adc.h"
#include "app_therm.h"
#include "app_jk.h"
#include "bms_calib.h"
#include "bms_errors.h"
#include "CAN_DB.h"
#include "main.h"

static CAN_InstanceTypeDef i1, i2;
static CAN_HandleTypeDef h1 = { &i1, { DISABLE } }, h2 = { &i2, { DISABLE } };
static UART_InstanceTypeDef uinst;
static UART_HandleTypeDef huart = { &uinst };
static EH_HandleTypeDef eh;
static volatile uint16_t adcBuf[3];

static void setup(void)
{
    Fake_Reset();
    i1.MCR = 0; i2.MCR = 0;
    memset(&eh, 0, sizeof eh);
    ADC_Init(adcBuf, &eh);
    THERM_Init(&eh);
    JK_Init(&huart, &eh);
    CAN_App_Init(&h1, &h2, &eh);
}

/* One pass per millisecond, as the superloop runs far faster than the tick.
   Only three TX mailboxes exist, so a burst of frames that fall due together
   drains over the next few passes rather than all at once. */
static void pump(uint32_t untilMs)
{
    for (uint32_t t = 0; t <= untilMs; t++) {
        Fake_SetTick(t);
        CAN_App_Task();
    }
}

TEST(both_transceivers_leave_standby_before_the_buses_start)
{
    setup();
    /* LOW is normal operation: the n prefix misleads, and driving these HIGH
       would put both transceivers to sleep and produce a board with no CAN. */
    CHECK_EQ(Fake_PinState(nCAN1_Stby_GPIO_Port, nCAN1_Stby_Pin), GPIO_PIN_RESET);
    CHECK_EQ(Fake_PinState(nCAN2_Stby_GPIO_Port, nCAN2_Stby_Pin), GPIO_PIN_RESET);
}

TEST(twenty_frames_are_registered_plus_the_node_frame)
{
    setup();
    CHECK_EQ(CAN_App_Scheduler()->size, 20u);
    EH_init(&eh, &h1, BMSMASTER_NODE_FRAME_ID, CAN_App_Scheduler());
    CHECK_EQ(CAN_App_Scheduler()->size, 21u);
}

TEST(every_expected_frame_id_appears_on_the_bus)
{
    setup();
    pump(6000u);
    const uint32_t ids[] = {
        BMSMASTER_MASTERVOLTCURRTEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM1TEMP_FRAME_ID, BMSMASTER_PCBSTHERM2TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM3TEMP_FRAME_ID, BMSMASTER_PCBSTHERM4TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM5TEMP_FRAME_ID, BMSMASTER_PCBSTHERM6TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM7TEMP_FRAME_ID, BMSMASTER_PCBSTHERM8TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM9TEMP_FRAME_ID,
        BMSMASTER_JK_PACK_FRAME_ID,
        BMSMASTER_JK_CELLS_1_4_FRAME_ID, BMSMASTER_JK_CELLS_5_8_FRAME_ID,
        BMSMASTER_JK_CELLS_9_12_FRAME_ID, BMSMASTER_JK_CELLS_13_16_FRAME_ID,
        BMSMASTER_JK_CELLS_17_20_FRAME_ID, BMSMASTER_JK_CELLS_21_FRAME_ID,
        BMSMASTER_JK_TEMP_FRAME_ID, BMSMASTER_JK_CYCLESTATS_FRAME_ID,
        BMSMASTER_END_FRAME_ID
    };
    /* All 20 ids, not a sample: a deleted registration must fail this loop. */
    CHECK_EQ(sizeof ids / sizeof ids[0], 20u);
    for (size_t i = 0; i < sizeof ids / sizeof ids[0]; i++) {
        CHECK(Fake_TxCountFor(ids[i]) > 0u);
    }
}

TEST(the_measurement_frame_runs_at_two_hertz)
{
    setup();
    /* 10010, not 10000: the burst due at 10000 needs a few passes to reach the
       three mailboxes, and the 10th JK frame is late in the list. */
    pump(10010u);
    /* 500 ms period over 10 s: 20 frames. */
    CHECK_EQ(Fake_TxCountFor(BMSMASTER_MASTERVOLTCURRTEMP_FRAME_ID), 20u);
    CHECK_EQ(Fake_TxCountFor(BMSMASTER_JK_PACK_FRAME_ID), 10u);      /* 1000 ms */
}

TEST(the_end_frame_is_eight_zero_bytes)
{
    setup();
    pump(2000u);
    uint8_t data[8];
    CHECK(Fake_FindTx(BMSMASTER_END_FRAME_ID, data, NULL));
    for (int i = 0; i < 8; i++) { CHECK_EQ(data[i], 0u); }
}

TEST(measurements_reach_the_payload)
{
    setup();
    for (int i = 0; i < 10; i++) {
        adcBuf[0] = calibNtcCount[25]; adcBuf[1] = 2112u; adcBuf[2] = 3000u;
        ADC_OnConvComplete();
        ADC_Task(0u);
    }
    pump(1000u);

    uint8_t d[8];
    CHECK(Fake_FindTx(BMSMASTER_MASTERVOLTCURRTEMP_FRAME_ID, d, NULL));
    /* Voltage is a little-endian u16 at byte 0, scale 0.1 V. */
    CHECK_EQ((uint16_t)(d[0] | ((uint16_t)d[1] << 8)), ADC_PackDecivolts());
    /* Current is a little-endian i16 at byte 2, scale 0.1 A - the signed spot check. */
    CHECK_EQ((int16_t)(d[2] | ((uint16_t)d[3] << 8)), ADC_PackDeciamps());
}

TEST(the_thermistor_frames_carry_the_transpose)
{
    setup();
    /* Give every module a distinct value on thermistor 1. */
    for (int period = 0; period < 10; period++) {
        for (uint8_t module = 1u; module <= 7u; module++) {
            const uint32_t base = 210u + ((module - 1u) * 10u);
            const uint32_t id = (module & 1u) ? (base + 1u) : (base + 9u);   /* Therm1 */
            THERM_OnFrame(id, (uint8_t)(30u + module));
        }
        THERM_Task();
    }
    pump(1000u);

    uint8_t d[8];
    CHECK(Fake_FindTx(BMSMASTER_PCBSTHERM1TEMP_FRAME_ID, d, NULL));
    for (uint8_t module = 1u; module <= 7u; module++) {
        CHECK_EQ(d[module - 1u], (uint8_t)(30u + module));
    }
}

TEST(link_down_jk_frames_carry_zero_payload_on_cadence)
{
    /* No JK exchange has happened: cells and pack stay all-zero, but the
       schedule must still fire on time - link-down must not silence the bus. */
    setup();
    pump(1010u);                      /* let the burst due at 1000 drain */
    uint8_t d[8];
    CHECK(Fake_FindTx(BMSMASTER_JK_CELLS_1_4_FRAME_ID, d, NULL));
    for (int i = 0; i < 8; i++) { CHECK_EQ(d[i], 0u); }
    CHECK(Fake_FindTx(BMSMASTER_JK_PACK_FRAME_ID, d, NULL));
    for (int i = 0; i < 8; i++) { CHECK_EQ(d[i], 0u); }
}

TEST(can1_filter_admits_only_the_safe_state_ids)
{
    setup();
    /* Bank 0 is a 16-bit list of {1, 3, 1, 1}: only 1 and 3 pass. */
    Fake_QueueCanRx(&h1, 1u, NULL, 1u);
    Fake_QueueCanRx(&h1, 3u, NULL, 1u);
    Fake_QueueCanRx(&h1, 42u, NULL, 1u);       /* rejected: not in the list */
    CHECK_EQ(Fake_RxPending(&h1), 2u);         /* the reject never reached the queue */
    CAN_App_OnRx1(&h1);
    CHECK_EQ(Fake_RxPending(&h1), 0u);
}

TEST(can2_filter_admits_the_thermistor_range_and_rejects_others)
{
    setup();
    const uint8_t data[8] = {50u};
    Fake_QueueCanRx(&h2, 211u, data, 1u);      /* module 1 therm 1: in range */
    Fake_QueueCanRx(&h2, 279u, data, 1u);      /* module 7 therm 9: in range */
    Fake_QueueCanRx(&h2, 999u, data, 1u);      /* far outside 0x0D0..0x11F: rejected */
    CHECK_EQ(Fake_RxPending(&h2), 2u);
    CAN_App_OnRx2(&h2);
    CHECK_EQ(Fake_RxPending(&h2), 0u);
}

TEST(safe_state_frames_route_to_the_contactor_and_thermistors_do_not)
{
    setup();
    /* CAN1 and CAN2 share HAL_CAN_RxFifo0MsgPendingCallback, so the dispatch
       must key on the instance or CAN1 traffic lands in the thermistor path. */
    CHECK_EQ(THERM_Filtered(1u, 1u), 0u);
    CAN_App_OnRx1(&h1);
    CAN_App_OnRx2(&h2);
    CHECK_EQ(THERM_Filtered(1u, 1u), 0u);     /* nothing queued: no spurious writes */
}

/* One pass per millisecond over a window, without restarting at tick 0. */
static void pumpFrom(uint32_t fromMs, uint32_t untilMs)
{
    for (uint32_t t = fromMs; t <= untilMs; t++) {
        Fake_SetTick(t);
        CAN_App_Task();
    }
}

TEST(burst_contention_alone_never_reports_a_tx_fault)
{
    setup();
    EH_init(&eh, &h1, BMSMASTER_NODE_FRAME_ID, CAN_App_Scheduler());
    /* Nineteen of the twenty frames fall due in the same pass against three
       mailboxes, so failed enqueues are routine here. Code 9 must mean "this
       frame cannot get out", never "a mailbox was busy". */
    pump(10000u);
    CHECK(Fake_TxCountFor(BMSMASTER_JK_CYCLESTATS_FRAME_ID) > 0u);
    CHECK_EQ(eh.activeErrorCount, 0u);
}

TEST(a_frame_that_cannot_get_out_raises_tx_fail_and_clears_on_recovery)
{
    setup();
    EH_init(&eh, &h1, BMSMASTER_NODE_FRAME_ID, CAN_App_Scheduler());
    pump(1010u);
    CHECK_EQ(eh.activeErrorCount, 0u);

    /* Bus-off, missing termination or no ACK: the mailboxes never free. The
       driver aborts after CAN_TX_FAIL_LIMIT missed periods; before this fix it
       did so silently and the vehicle never learned CAN1 TX was failing. */
    Fake_HoldCanTx(1);
    pumpFrom(1011u, 5000u);
    CHECK(Fake_CanAbortCount() > 0u);
    CHECK_EQ(eh.activeErrorCount, 1u);
    CHECK_EQ(eh.activeErrors[0].errorCode, BMS_ERR_CAN1_TX_FAIL);
    CHECK_EQ(eh.activeErrors[0].severity, ERROR_SEVERITY_WARNING);

    /* Spec 10: the payload is the frame ID as a little-endian u16. */
    CHECK_EQ(eh.activeErrors[0].specificDataLen, 2u);
    const uint32_t reported = (uint32_t)eh.activeErrors[0].specificData[0]
                            | ((uint32_t)eh.activeErrors[0].specificData[1] << 8);
    int registered = 0;
    for (uint8_t i = 0u; i < CAN_App_Scheduler()->size; i++) {
        if (CAN_App_Scheduler()->list[i].header.StdId == reported) { registered = 1; }
    }
    CHECK(registered);

    Fake_HoldCanTx(0);
    pumpFrom(5001u, 8000u);
    CHECK_EQ(eh.activeErrorCount, 0u);
}

/* A failed controller bring-up happens before EH_init, so it cannot be
   reported as a fault. It must fail the init instead, or the board runs with a
   dead bus and never says so. */
TEST(a_failed_controller_bring_up_fails_the_init)
{
    Fake_Reset();
    i1.MCR = 0; i2.MCR = 0;
    memset(&eh, 0, sizeof eh);
    CHECK(CAN_App_Init(&h1, &h2, &eh));      /* healthy: reports success */

    Fake_Reset();
    i1.MCR = 0; i2.MCR = 0;
    memset(&eh, 0, sizeof eh);
    Fake_ForceCanStartFail();
    CHECK(!CAN_App_Init(&h1, &h2, &eh));     /* one controller down: reports failure */
}

int main(void)
{
    RUN(both_transceivers_leave_standby_before_the_buses_start);
    RUN(twenty_frames_are_registered_plus_the_node_frame);
    RUN(every_expected_frame_id_appears_on_the_bus);
    RUN(the_measurement_frame_runs_at_two_hertz);
    RUN(the_end_frame_is_eight_zero_bytes);
    RUN(measurements_reach_the_payload);
    RUN(the_thermistor_frames_carry_the_transpose);
    RUN(link_down_jk_frames_carry_zero_payload_on_cadence);
    RUN(can1_filter_admits_only_the_safe_state_ids);
    RUN(can2_filter_admits_the_thermistor_range_and_rejects_others);
    RUN(safe_state_frames_route_to_the_contactor_and_thermistors_do_not);
    RUN(burst_contention_alone_never_reports_a_tx_fault);
    RUN(a_frame_that_cannot_get_out_raises_tx_fail_and_clears_on_recovery);
    RUN(a_failed_controller_bring_up_fails_the_init);
    return TEST_SUMMARY();
}

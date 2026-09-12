#include "test_runner.h"
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
}

/* Drive the scheduler far enough that every registered frame is due at least once. */
static void pump(uint32_t untilMs)
{
    for (uint32_t t = 0; t <= untilMs; t += 100u) {
        Fake_SetTick(t);
        CAN_HandleScheduled(&hcan, &sched);
    }
}

TEST(error_codes_match_the_csv_registry)
{
    CHECK_EQ(BMS_ERR_TEMP_HIGH, 1);
    CHECK_EQ(BMS_ERR_CAN2_TEMP_HIGH, 2);
    CHECK_EQ(BMS_ERR_CAN2_MODULE_SILENT, 3);
    CHECK_EQ(BMS_ERR_JK_COMMS_TIMEOUT, 4);
    CHECK_EQ(BMS_ERR_JK_FRAME_INVALID, 5);
    CHECK_EQ(BMS_ERR_PACK_VOLT_RANGE, 6);
    CHECK_EQ(BMS_ERR_PACK_CURRENT_HIGH, 7);
    CHECK_EQ(BMS_ERR_TEMP_SENSOR_FAULT, 8);
    CHECK_EQ(BMS_ERR_CAN1_TX_FAIL, 9);
    CHECK_EQ(BMS_ERR_FATAL_INIT, 10);
    CHECK_EQ(BMS_ERR_ADC_STALLED, 11);
    CHECK_EQ(BMS_ERR_CAN2_THERM_SATURATED, 12);
}

TEST(heartbeat_uses_the_database_cycle_time_not_the_driver_default)
{
    CHECK_EQ(HEARTBEAT_INTERVAL, 5000);
}

TEST(node_frame_is_registered_on_the_scheduler)
{
    setup();
    CHECK(sched.size > 0u);
    pump(6000u);
    CHECK(Fake_TxCountFor(BMSMASTER_NODE_FRAME_ID) > 0u);
}

TEST(a_reported_fault_reaches_the_node_frame)
{
    setup();
    const uint8_t blob[5] = { 0x12u, 0x34u, 0u, 0u, 0u };
    EH_reportEx(&eh, BMS_ERR_PACK_VOLT_RANGE, ERROR_SEVERITY_ERROR, blob, 2u);
    CHECK_EQ(eh.activeErrorCount, 1u);

    pump(6000u);
    uint8_t data[8];
    CHECK(Fake_FindTx(BMSMASTER_NODE_FRAME_ID, data, NULL));

    /* Error_Code is a 16-bit little-endian field at byte 0 of BMSMaster_NODE. */
    uint16_t code = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
    CHECK_EQ(code, BMS_ERR_PACK_VOLT_RANGE);
}

/* ADC_Task re-reports the same fault on every scan while the condition holds.
   A repeat report used to re-install the scheduled message, resetting lastTick,
   so BMSMaster_NODE never aged up to its period and stopped transmitting. */
TEST(a_repeatedly_reported_single_fault_keeps_the_node_frame_on_cadence)
{
    setup();
    EH_reportEx(&eh, BMS_ERR_PACK_VOLT_RANGE, ERROR_SEVERITY_ERROR, NULL, 0u);
    CHECK_EQ(eh.activeErrorCount, 1u);

    const uint32_t window = 4u * (uint32_t)ERROR_INTERVAL;
    uint32_t sent = 0u, lastSendTick = 0u, worstGapMs = 0u;
    for (uint32_t t = 1u; t <= window; t++) {
        Fake_SetTick(t);
        /* Re-report at 1 ms, far faster than the frame period. */
        EH_reportEx(&eh, BMS_ERR_PACK_VOLT_RANGE, ERROR_SEVERITY_ERROR, NULL, 0u);
        CAN_HandleScheduled(&hcan, &sched);

        uint32_t now = Fake_TxCountFor(BMSMASTER_NODE_FRAME_ID);
        if (now != sent) {
            uint32_t gapMs = t - lastSendTick;
            if (gapMs > worstGapMs) worstGapMs = gapMs;
            lastSendTick = t;
            sent = now;
        }
    }

    CHECK(sent >= (window / (uint32_t)ERROR_INTERVAL) - 1u);
    CHECK(worstGapMs <= (uint32_t)ERROR_INTERVAL + 1u);
}

/* The re-install fires once per insertion, so a second fault must not restart it either. */
TEST(a_second_fault_does_not_restart_the_node_frame)
{
    setup();
    EH_reportEx(&eh, BMS_ERR_PACK_VOLT_RANGE, ERROR_SEVERITY_ERROR, NULL, 0u);
    Fake_SetTick((uint32_t)ERROR_INTERVAL - 1u);
    CAN_HandleScheduled(&hcan, &sched);
    CHECK_EQ(Fake_TxCountFor(BMSMASTER_NODE_FRAME_ID), 0u);

    EH_reportEx(&eh, BMS_ERR_PACK_CURRENT_HIGH, ERROR_SEVERITY_ERROR, NULL, 0u);
    Fake_SetTick((uint32_t)ERROR_INTERVAL);
    CAN_HandleScheduled(&hcan, &sched);
    CHECK_EQ(Fake_TxCountFor(BMSMASTER_NODE_FRAME_ID), 1u);
}

TEST(three_faults_cycle_at_the_heartbeat_cadence_not_faster)
{
    setup();
    EH_reportEx(&eh, BMS_ERR_PACK_VOLT_RANGE,   ERROR_SEVERITY_ERROR, NULL, 0u);
    EH_reportEx(&eh, BMS_ERR_PACK_CURRENT_HIGH, ERROR_SEVERITY_ERROR, NULL, 0u);
    EH_reportEx(&eh, BMS_ERR_JK_COMMS_TIMEOUT,  ERROR_SEVERITY_ERROR, NULL, 0u);
    CHECK_EQ(eh.activeErrorCount, 3u);

    /* One full round trip is 3 * ERROR_INTERVAL: one error per transmission. */
    const uint32_t cycle = 3u * (uint32_t)ERROR_INTERVAL;
    uint32_t seenVolt = 0u, seenCurr = 0u, seenJk = 0u, sent = 0u;

    for (uint32_t t = 1u; t <= cycle; t++) {
        Fake_SetTick(t);
        CAN_HandleScheduled(&hcan, &sched);
        const uint32_t now = Fake_TxCountFor(BMSMASTER_NODE_FRAME_ID);
        if (now == sent) { continue; }
        sent = now;
        uint8_t d[8];
        CHECK(Fake_FindTx(BMSMASTER_NODE_FRAME_ID, d, NULL));
        const uint16_t code = (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
        if (code == BMS_ERR_PACK_VOLT_RANGE)   { seenVolt++; }
        if (code == BMS_ERR_PACK_CURRENT_HIGH) { seenCurr++; }
        if (code == BMS_ERR_JK_COMMS_TIMEOUT)  { seenJk++; }
    }

    CHECK_EQ(sent, 3u);              /* three sends in 15 s, not fifty */
    CHECK_EQ(seenVolt, 1u);          /* and each fault got exactly one slot */
    CHECK_EQ(seenCurr, 1u);
    CHECK_EQ(seenJk, 1u);
}

TEST(clearing_the_last_fault_returns_to_healthy)
{
    setup();
    EH_reportEx(&eh, BMS_ERR_JK_COMMS_TIMEOUT, ERROR_SEVERITY_ERROR, NULL, 0u);
    CHECK_EQ(EH_getActiveCount(&eh), 1u);
    EH_clear(&eh, BMS_ERR_JK_COMMS_TIMEOUT);
    CHECK_EQ(EH_getActiveCount(&eh), 0u);
}

TEST(a_fault_that_clears_before_its_slot_still_reaches_the_bus_once)
{
    setup();
    /* Raised and gone well inside one period: at 5000 ms cadence this would
       otherwise never be transmitted at all. */
    EH_reportEx(&eh, BMS_ERR_JK_FRAME_INVALID, ERROR_SEVERITY_WARNING, NULL, 0u);
    EH_clear(&eh, BMS_ERR_JK_FRAME_INVALID);
    CHECK_EQ(EH_getActiveCount(&eh), 0u);          /* the node is healthy again */

    uint16_t codes[4] = {0u, 0u, 0u, 0u};
    uint32_t sent = 0u;
    for (uint32_t t = 1u; t <= 2u * (uint32_t)HEARTBEAT_INTERVAL; t++) {
        Fake_SetTick(t);
        CAN_HandleScheduled(&hcan, &sched);
        const uint32_t now = Fake_TxCountFor(BMSMASTER_NODE_FRAME_ID);
        if (now == sent) { continue; }
        sent = now;
        uint8_t d[8];
        CHECK(Fake_FindTx(BMSMASTER_NODE_FRAME_ID, d, NULL));
        if (sent <= 4u) { codes[sent - 1u] = (uint16_t)(d[0] | ((uint16_t)d[1] << 8)); }
    }

    CHECK_EQ(sent, 2u);
    CHECK_EQ(codes[0], (uint16_t)BMS_ERR_JK_FRAME_INVALID);  /* sent once ... */
    CHECK_EQ(codes[1], (uint16_t)HEARTBEAT_ERROR_CODE);      /* ... then back to healthy */
    CHECK_EQ(eh.activeErrorCount, 0u);                       /* and dropped from the queue */
}

int main(void)
{
    RUN(error_codes_match_the_csv_registry);
    RUN(heartbeat_uses_the_database_cycle_time_not_the_driver_default);
    RUN(node_frame_is_registered_on_the_scheduler);
    RUN(a_reported_fault_reaches_the_node_frame);
    RUN(a_repeatedly_reported_single_fault_keeps_the_node_frame_on_cadence);
    RUN(a_second_fault_does_not_restart_the_node_frame);
    RUN(three_faults_cycle_at_the_heartbeat_cadence_not_faster);
    RUN(clearing_the_last_fault_returns_to_healthy);
    RUN(a_fault_that_clears_before_its_slot_still_reaches_the_bus_once);
    return TEST_SUMMARY();
}

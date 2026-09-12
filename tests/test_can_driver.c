#include "test_runner.h"
#include "can_driver.h"

static CAN_InstanceTypeDef inst;
static CAN_HandleTypeDef hcan = { &inst, { DISABLE } };

static void fillZero(uint8_t *d, void *ctx) { (void)ctx; memset(d, 0, 8); }

static void setup(void)
{
    Fake_Reset();
    inst.MCR = 0;
    hcan.Instance = &inst;
}

TEST(init_sets_nart_when_retransmission_is_disabled)
{
    setup();
    CHECK_EQ(CAN_Init(&hcan), HAL_OK);
    /* CAN_AUTO_RETRANSMISSION is 0, so NART must be set and mirrored. */
    CHECK((inst.MCR & CAN_MCR_NART) != 0u);
    CHECK_EQ(hcan.Init.AutoRetransmission, DISABLE);
}

TEST(init_rejects_a_null_handle)
{
    setup();
    CHECK_EQ(CAN_Init(NULL), HAL_ERROR);
}

TEST(filter_rejects_a_bank_beyond_the_hardware)
{
    setup();
    CHECK_EQ(CAN_ConfigFilterMask32(&hcan, CAN_FILTER_BANK_COUNT, 0x0D0u, 0x7F0u), HAL_ERROR);
    CHECK_EQ(CAN_ConfigFilterMask32(&hcan, 0u, 0x0D0u, 0x7F0u), HAL_OK);
    uint16_t ids[4] = { 1u, 3u, 1u, 1u };
    CHECK_EQ(CAN_ConfigFilterList16(&hcan, CAN_FILTER_BANK_COUNT, ids), HAL_ERROR);
    CHECK_EQ(CAN_ConfigFilterList16(&hcan, 0u, ids), HAL_OK);
}

TEST(scheduler_holds_28_messages_and_rejects_bad_input)
{
    setup();
    struct CAN_scheduledMsgList list = {0};
    CHECK_EQ(CAN_AddScheduledMsg(NULL, &list), HAL_ERROR);

    struct CAN_scheduledMsg m = {0};
    m.header.IDE = CAN_ID_STD; m.header.RTR = CAN_RTR_DATA; m.header.DLC = 8;
    m.getData = fillZero;
    m.periodMs = 0u;
    m.header.StdId = 100u;
    CHECK_EQ(CAN_AddScheduledMsg(&m, &list), HAL_ERROR);      /* zero period */

    m.periodMs = 1000u;
    CHECK_EQ(CAN_AddScheduledMsg(&m, &list), HAL_OK);
    CHECK_EQ(CAN_AddScheduledMsg(&m, &list), HAL_ERROR);      /* duplicate id */

    for (uint32_t i = 1; i < CAN_MAX_MSG; i++) {
        m.header.StdId = 200u + i;
        CHECK_EQ(CAN_AddScheduledMsg(&m, &list), HAL_OK);
    }
    CHECK_EQ(list.size, CAN_MAX_MSG);
    m.header.StdId = 999u;
    CHECK_EQ(CAN_AddScheduledMsg(&m, &list), HAL_ERROR);      /* full */
}

TEST(scheduler_keeps_cadence_without_drift)
{
    setup();
    struct CAN_scheduledMsgList list = {0};
    struct CAN_scheduledMsg m = {0};
    m.header.StdId = 130u; m.header.IDE = CAN_ID_STD; m.header.RTR = CAN_RTR_DATA;
    m.header.DLC = 6; m.periodMs = 500u; m.getData = fillZero;
    CHECK_EQ(CAN_AddScheduledMsg(&m, &list), HAL_OK);

    /* Service every 7 ms for 10 s: expect exactly 20 frames, not 19 or 21.
       Upper bound is 10006, not 10000: 10000 itself is never a multiple of
       the 7 ms service step, so stopping at 10000 would miss the 20th
       period (due at tick 10000) by one service call. */
    for (uint32_t t = 0; t <= 10006u; t += 7u) {
        Fake_SetTick(t);
        CAN_HandleScheduled(&hcan, &list);
    }
    CHECK_EQ(Fake_TxCountFor(130u), 20u);
}

/* The real BMS Master registration: nine PCB thermistor frames, nine JK frames
   and END all on the DBC's 1000 ms cycle, plus the 500 ms measurement frame.
   Nineteen of the twenty fall due in the same pass. */
static const struct { uint32_t id; uint32_t periodMs; } burst[] = {
    { 0x82u, 500u },
    { 0x83u, 1000u }, { 0x84u, 1000u }, { 0x85u, 1000u }, { 0x86u, 1000u },
    { 0x87u, 1000u }, { 0x88u, 1000u }, { 0x89u, 1000u }, { 0x8Au, 1000u },
    { 0x8Bu, 1000u }, { 0x8Cu, 1000u }, { 0x8Du, 1000u }, { 0x8Eu, 1000u },
    { 0x8Fu, 1000u }, { 0x90u, 1000u }, { 0x91u, 1000u }, { 0x92u, 1000u },
    { 0x93u, 1000u }, { 0x94u, 1000u }, { 0x9Fu, 1000u }
};
#define BURST_COUNT (sizeof burst / sizeof burst[0])

static void addBurst(struct CAN_scheduledMsgList *list)
{
    for (uint32_t i = 0; i < BURST_COUNT; i++) {
        struct CAN_scheduledMsg m = {0};
        m.header.StdId = burst[i].id; m.header.IDE = CAN_ID_STD;
        m.header.RTR = CAN_RTR_DATA; m.header.DLC = 8;
        m.periodMs = burst[i].periodMs; m.getData = fillZero;
        CHECK_EQ(CAN_AddScheduledMsg(&m, list), HAL_OK);
    }
}

TEST(a_full_burst_exhausts_the_three_mailboxes_and_drains_over_later_passes)
{
    setup();
    struct CAN_scheduledMsgList list = {0};
    addBurst(&list);
    CHECK_EQ(list.size, BURST_COUNT);

    Fake_SetTick(1000u);
    CAN_HandleScheduled(&hcan, &list);
    CHECK_EQ(Fake_TxCount(), 3u);                 /* bxCAN holds three frames, no more */
    CHECK_EQ(Fake_CanTxMailboxesFree(&hcan), 0u);

    Fake_SetTick(1001u);
    CAN_HandleScheduled(&hcan, &list);
    CHECK_EQ(Fake_TxCount(), 6u);                 /* the rest wait for a pass, not a period */
}

TEST(every_frame_of_the_burst_keeps_its_cadence)
{
    setup();
    struct CAN_scheduledMsgList list = {0};
    addBurst(&list);

    /* 6 s of 1 ms superloop passes. 6010, not 6000: twenty frames need seven
       passes to clear three mailboxes, so the last burst drains just after. */
    for (uint32_t t = 0; t <= 6010u; t++) {
        Fake_SetTick(t);
        CAN_HandleScheduled(&hcan, &list);
    }

    /* Every frame, not a sample: under the re-arming scheduler the tail of the
       list transmitted zero times while the head kept its slot. */
    for (uint32_t i = 0; i < BURST_COUNT; i++) {
        CHECK_EQ(Fake_TxCountFor(burst[i].id), 6000u / burst[i].periodMs);
    }
    CHECK_EQ(Fake_CanAbortCount(), 0u);           /* contention alone must not escalate */
}

TEST(a_permanently_stuck_mailbox_is_aborted_but_contention_is_not)
{
    setup();
    struct CAN_scheduledMsgList list = {0};
    addBurst(&list);
    Fake_HoldCanTx(1);              /* unacknowledged frames never release a mailbox */

    /* CAN_TX_FAIL_LIMIT counts each frame's own periods, so the 500 ms frame
       escalates first, three of its periods after its last send at 1000. */
    for (uint32_t t = 0; t <= 2499u; t++) {
        Fake_SetTick(t);
        CAN_HandleScheduled(&hcan, &list);
    }
    CHECK_EQ(Fake_CanAbortCount(), 0u);

    for (uint32_t t = 2500u; t <= 4999u; t++) {
        Fake_SetTick(t);
        CAN_HandleScheduled(&hcan, &list);
    }
    /* Recovery runs, but once per period: 2500 passes happened in that window. */
    CHECK(Fake_CanAbortCount() >= 1u);
    CHECK(Fake_CanAbortCount() <= 2u * BURST_COUNT);
}

TEST(incoming_ring_is_fifo_and_bounded)
{
    setup();
    struct CAN_IncomingMsgList rx = {0};
    CAN_RxHeaderTypeDef h = {0};
    uint8_t d[8] = {0};
    for (uint32_t i = 0; i < CAN_MAX_MSG; i++) {
        h.StdId = 10u + i; d[0] = (uint8_t)i;
        CHECK_EQ(CAN_AddIncomingMsg(&rx, &h, d), HAL_OK);
    }
    h.StdId = 999u;
    CHECK_EQ(CAN_AddIncomingMsg(&rx, &h, d), HAL_ERROR);      /* full */

    struct CAN_IncomingMsg out;
    CHECK_EQ(CAN_GetLatestMessage(&rx, &out), HAL_OK);
    CHECK_EQ(out.header.StdId, 10u);                          /* FIFO, not lowest id */
}

int main(void)
{
    RUN(init_sets_nart_when_retransmission_is_disabled);
    RUN(init_rejects_a_null_handle);
    RUN(filter_rejects_a_bank_beyond_the_hardware);
    RUN(scheduler_holds_28_messages_and_rejects_bad_input);
    RUN(scheduler_keeps_cadence_without_drift);
    RUN(a_full_burst_exhausts_the_three_mailboxes_and_drains_over_later_passes);
    RUN(every_frame_of_the_burst_keeps_its_cadence);
    RUN(a_permanently_stuck_mailbox_is_aborted_but_contention_is_not);
    RUN(incoming_ring_is_fifo_and_bounded);
    return TEST_SUMMARY();
}

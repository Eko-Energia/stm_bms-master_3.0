#include "test_runner.h"
#include "app_jk.h"
#include "bms_errors.h"
#include "error_handler.h"
#include "CAN_DB.h"
#include "main.h"

static CAN_InstanceTypeDef inst;
static CAN_HandleTypeDef hcan = { &inst, { DISABLE } };
static struct CAN_scheduledMsgList sched;
static EH_HandleTypeDef eh;
static UART_InstanceTypeDef uinst;
static UART_HandleTypeDef huart = { &uinst };

static void setup(void)
{
    Fake_Reset();
    inst.MCR = 0;
    memset(&sched, 0, sizeof sched);
    memset(&eh, 0, sizeof eh);
    EH_init(&eh, &hcan, BMSMASTER_NODE_FRAME_ID, &sched);
    JK_Init(&huart, &eh);
}

static uint16_t makeSocResponse(uint8_t *buf, uint8_t soc)
{
    uint16_t n = 0;
    buf[n++] = 0x4Eu; buf[n++] = 0x57u;
    const uint16_t total = 22u;
    buf[n++] = (uint8_t)((total - 2u) >> 8); buf[n++] = (uint8_t)((total - 2u) & 0xFFu);
    buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u;
    buf[n++] = 0x06u; buf[n++] = 0x00u; buf[n++] = 0x01u;
    buf[n++] = 0x85u; buf[n++] = soc;
    buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u;
    buf[n++] = 0x68u;
    uint16_t sum = 0u;
    for (uint16_t i = 0; i < n; i++) { sum = (uint16_t)(sum + buf[i]); }
    buf[n++] = 0u; buf[n++] = 0u;
    buf[n++] = (uint8_t)(sum >> 8); buf[n++] = (uint8_t)(sum & 0xFFu);
    return n;
}

/* The reply to JKP_CMD_ACTIVATE: a well-formed, checksum-valid frame with an
   empty TLV payload, so JKP_Decode returns true with an all-zero JK_Data_t. */
static uint16_t makeActivateAck(uint8_t *buf)
{
    const uint16_t total = 20u;
    memset(buf, 0, total);
    buf[0] = 0x4Eu; buf[1] = 0x57u;
    buf[2] = (uint8_t)((total - 2u) >> 8); buf[3] = (uint8_t)((total - 2u) & 0xFFu);
    buf[8] = JKP_CMD_ACTIVATE;
    buf[10] = 0x01u;                        /* transmission type: response */
    buf[15] = 0x68u;                        /* end flag */
    uint16_t sum = 0u;
    for (uint16_t i = 0; i <= 15u; i++) { sum = (uint16_t)(sum + buf[i]); }
    buf[18] = (uint8_t)(sum >> 8); buf[19] = (uint8_t)(sum & 0xFFu);
    return total;
}

/* One full exchange: request goes out, TC fires, the reply is queued and its
   arrival signalled the way HAL_UARTEx_RxEventCallback would - by calling
   JK_OnRxEvent directly. That callback is dispatched from app.c (Task 11),
   not from this module, so the test drives the module's real interface. */
static void exchange(uint32_t now, const uint8_t *reply, uint16_t replyLen)
{
    Fake_SetTick(now);
    JK_Task(now);
    if (replyLen > 0u) { Fake_QueueUartRx(reply, replyLen); }
    JK_OnTxComplete();
    if (replyLen > 0u) { JK_OnRxEvent(replyLen); }
    JK_Task(now);
}

TEST(the_first_exchange_sends_the_activation_command)
{
    setup();
    Fake_SetTick(0u);
    JK_Task(0u);
    uint8_t sent[JKP_REQUEST_LEN];
    CHECK_EQ(Fake_LastUartTx(sent, sizeof sent), JKP_REQUEST_LEN);
    CHECK_EQ(sent[8], JKP_CMD_ACTIVATE);
}

TEST(transmit_asserts_the_driver_and_mutes_the_receiver)
{
    setup();
    Fake_SetTick(0u);
    JK_Task(0u);
    CHECK_EQ(Fake_PinState(RS_DIR_GPIO_Port, RS_DIR_Pin), GPIO_PIN_SET);
    CHECK_EQ(Fake_PinState(RE_DIR_GPIO_Port, RE_DIR_Pin), GPIO_PIN_SET);
}

TEST(transmission_complete_turns_the_transceiver_around)
{
    setup();
    Fake_SetTick(0u);
    JK_Task(0u);
    JK_OnTxComplete();
    CHECK_EQ(Fake_PinState(RS_DIR_GPIO_Port, RS_DIR_Pin), GPIO_PIN_RESET);
    CHECK_EQ(Fake_PinState(RE_DIR_GPIO_Port, RE_DIR_Pin), GPIO_PIN_RESET);
}

TEST(a_valid_response_is_decoded_and_marks_the_link_up)
{
    setup();
    uint8_t reply[64];
    const uint16_t n = makeSocResponse(reply, 77u);

    exchange(0u, reply, n);                       /* activation reply */
    exchange(1000u, reply, n);                    /* first read-all */
    CHECK(JK_Valid());
    CHECK_EQ(JK_Data()->soc, 77u);
}

TEST(polling_settles_to_read_all_at_one_hertz)
{
    setup();
    uint8_t reply[64];
    const uint16_t n = makeSocResponse(reply, 50u);
    exchange(0u, reply, n);
    exchange(1000u, reply, n);

    uint8_t sent[JKP_REQUEST_LEN];
    Fake_LastUartTx(sent, sizeof sent);
    CHECK_EQ(sent[8], JKP_CMD_READ_ALL);

    /* Nothing goes out between periods. */
    Fake_Reset();
    JK_Task(1500u);
    CHECK_EQ(Fake_LastUartTx(sent, sizeof sent), 0u);
}

TEST(three_timeouts_raise_the_comms_fault_and_zero_the_data)
{
    setup();
    uint8_t reply[64];
    const uint16_t n = makeSocResponse(reply, 60u);
    exchange(0u, reply, n);
    exchange(1000u, reply, n);
    CHECK_EQ(JK_Data()->soc, 60u);

    /* Three polls with no reply: each times out 100 ms after its request. */
    for (uint32_t p = 1u; p <= 3u; p++) {
        const uint32_t t = 1000u + (p * 1000u);
        Fake_SetTick(t);
        JK_Task(t);
        JK_OnTxComplete();
        JK_Task(t + 150u);                        /* past the 100 ms timeout */
    }
    CHECK(!JK_Valid());
    CHECK_EQ(JK_Data()->soc, 0u);
    CHECK(eh.activeErrorCount > 0u);
}

TEST(a_timeout_retries_with_activation_first)
{
    setup();
    uint8_t reply[64];
    const uint16_t n = makeSocResponse(reply, 60u);
    exchange(0u, reply, n);
    exchange(1000u, reply, n);

    Fake_SetTick(2000u);
    JK_Task(2000u);
    JK_OnTxComplete();
    JK_Task(2150u);                               /* timeout */

    Fake_Reset();
    JK_Task(3000u);
    uint8_t sent[JKP_REQUEST_LEN];
    Fake_LastUartTx(sent, sizeof sent);
    CHECK_EQ(sent[8], JKP_CMD_ACTIVATE);          /* the BMS may have gone to sleep */
}

TEST(a_corrupt_response_raises_frame_invalid_not_timeout)
{
    setup();
    uint8_t reply[64];
    uint16_t n = makeSocResponse(reply, 60u);
    exchange(0u, reply, n);

    reply[n - 1u] ^= 0xFFu;                       /* break the checksum */
    exchange(1000u, reply, n);
    CHECK(!JK_Valid());
    CHECK(eh.activeErrorCount > 0u);
}

TEST(a_response_arriving_just_after_the_deadline_is_ignored)
{
    setup();
    uint8_t reply[64];
    const uint16_t n = makeSocResponse(reply, 42u);
    exchange(0u, reply, n);
    exchange(1000u, reply, n);
    CHECK_EQ(JK_Data()->soc, 42u);

    Fake_SetTick(2000u);
    JK_Task(2000u);                 /* sends the next poll */
    JK_OnTxComplete();              /* arms the receiver, no reply queued yet */
    JK_Task(2101u);                 /* 101 ms later: past the 100 ms timeout */
    CHECK(!JK_Valid());

    /* The BMS's answer shows up after the module already gave up on it. */
    Fake_QueueUartRx(reply, n);
    JK_OnRxEvent(n);
    JK_Task(2102u);
    CHECK_EQ(JK_Data()->soc, 42u);  /* unchanged: the stale byte count is not decoded */
}

TEST(an_unsolicited_frame_with_no_request_outstanding_does_not_corrupt_state)
{
    setup();
    /* Right after Init nothing has been sent yet: the BMS pushes uninvited. */
    JK_OnRxEvent(21u);
    CHECK(!JK_Valid());

    uint8_t reply[64];
    const uint16_t n = makeSocResponse(reply, 88u);
    exchange(0u, reply, n);
    exchange(1000u, reply, n);
    CHECK(JK_Valid());
    CHECK_EQ(JK_Data()->soc, 88u);
}

TEST(polling_continues_across_the_tick_wrap)
{
    setup();
    uint8_t reply[64];
    const uint16_t n = makeSocResponse(reply, 33u);

    exchange(0u, reply, n);
    exchange(1000u, reply, n);

    /* Jump the poll clock right up to the wrap, then step across it. */
    const uint32_t nearWrap = 0xFFFFFFFFu - 500u;
    exchange(nearWrap, reply, n);
    CHECK(JK_Valid());

    const uint32_t afterWrap = nearWrap + 1000u;   /* wraps past 0xFFFFFFFF to 499 */
    exchange(afterWrap, reply, n);
    CHECK(JK_Valid());
    CHECK_EQ(JK_Data()->soc, 33u);
}

TEST(the_100ms_response_timeout_survives_the_tick_wrap)
{
    setup();
    uint8_t reply[64];
    const uint16_t n = makeSocResponse(reply, 15u);
    exchange(0u, reply, n);
    exchange(1000u, reply, n);
    CHECK(JK_Valid());

    /* Send the next poll right at the wrap, then let it go unanswered. */
    const uint32_t nearWrap = 0xFFFFFFFFu - 50u;
    Fake_SetTick(nearWrap);
    JK_Task(nearWrap);              /* sends the next read-all; requestMs = nearWrap */
    JK_OnTxComplete();              /* arms the receiver; no reply queued */

    const uint32_t afterWrap = nearWrap + 150u;   /* wraps past 0xFFFFFFFF to 99 */
    Fake_SetTick(afterWrap);
    JK_Task(afterWrap);             /* 150 ms elapsed: past the 100 ms timeout */
    CHECK(!JK_Valid());
    CHECK(eh.activeErrorCount > 0u);
}

TEST(a_failed_transmit_start_is_reported_and_counted)
{
    setup();
    uint8_t reply[64];
    const uint16_t n = makeSocResponse(reply, 70u);
    exchange(0u, reply, n);
    exchange(1000u, reply, n);
    CHECK_EQ(JK_Data()->soc, 70u);

    /* Three transmit starts that fail to launch, back to back. If a refused
       DMA start weren't counted as a failure, this would never zero the
       data - the link would look healthy while stuck sending nothing. */
    Fake_ForceUartTxFail();
    Fake_SetTick(2000u);
    JK_Task(2000u);
    CHECK(!JK_Valid());
    CHECK_EQ(JK_Data()->soc, 70u);      /* one strike: data still held */
    CHECK(eh.activeErrorCount > 0u);

    Fake_ForceUartTxFail();
    Fake_SetTick(3000u);
    JK_Task(3000u);
    CHECK_EQ(JK_Data()->soc, 70u);      /* two strikes: still held */

    Fake_ForceUartTxFail();
    Fake_SetTick(4000u);
    JK_Task(4000u);
    CHECK(!JK_Valid());
    CHECK_EQ(JK_Data()->soc, 0u);       /* three strikes: zeroed */
}

TEST(an_activation_reply_is_never_published_as_data)
{
    setup();
    uint8_t ack[32];   const uint16_t a = makeActivateAck(ack);
    uint8_t reply[64]; const uint16_t n = makeSocResponse(reply, 77u);

    /* Boot: the activation acknowledgement is not a reading. */
    exchange(0u, ack, a);
    CHECK(!JK_Valid());
    CHECK_EQ(JK_Data()->soc, 0u);

    exchange(1000u, reply, n);              /* the first real read-all */
    CHECK(JK_Valid());
    CHECK_EQ(JK_Data()->soc, 77u);

    /* The link drops, which arms an activation on the next poll. */
    Fake_SetTick(2000u);
    JK_Task(2000u);
    JK_OnTxComplete();
    JK_Task(2150u);                         /* past the 100 ms timeout */
    CHECK(!JK_Valid());
    CHECK(eh.activeErrorCount > 0u);

    /* Reconnect. The ack must not put 0 % SOC and 0 mV cells on the bus as
       healthy, and must not clear the comms fault. */
    exchange(3000u, ack, a);
    CHECK(!JK_Valid());
    CHECK_EQ(JK_Data()->soc, 77u);          /* held, not overwritten with zeros */
    CHECK(eh.activeErrorCount > 0u);

    /* The read-all that follows is what actually restores the link. */
    exchange(4000u, reply, n);
    CHECK(JK_Valid());
    CHECK_EQ(eh.activeErrorCount, 0u);
}

TEST(frame_invalid_is_graded_warning_and_a_timeout_error)
{
    setup();
    uint8_t ack[32];   const uint16_t a = makeActivateAck(ack);
    uint8_t reply[64]; uint16_t n = makeSocResponse(reply, 60u);
    exchange(0u, ack, a);

    reply[n - 1u] ^= 0xFFu;                 /* break the checksum */
    exchange(1000u, reply, n);
    CHECK_EQ(eh.activeErrorCount, 1u);
    CHECK_EQ(eh.activeErrors[0].errorCode, BMS_ERR_JK_FRAME_INVALID);
    /* Spec 10 grades code 5 a warning; severity drives eviction priority. */
    CHECK_EQ(eh.activeErrors[0].severity, ERROR_SEVERITY_WARNING);

    Fake_SetTick(2000u);
    JK_Task(2000u);
    JK_OnTxComplete();
    JK_Task(2150u);
    CHECK_EQ(eh.activeErrorCount, 2u);
    for (uint8_t i = 0u; i < eh.activeErrorCount; i++) {
        if (eh.activeErrors[i].errorCode == BMS_ERR_JK_COMMS_TIMEOUT) {
            CHECK_EQ(eh.activeErrors[i].severity, ERROR_SEVERITY_ERROR);
        }
    }
}

int main(void)
{
    RUN(the_first_exchange_sends_the_activation_command);
    RUN(transmit_asserts_the_driver_and_mutes_the_receiver);
    RUN(transmission_complete_turns_the_transceiver_around);
    RUN(a_valid_response_is_decoded_and_marks_the_link_up);
    RUN(polling_settles_to_read_all_at_one_hertz);
    RUN(three_timeouts_raise_the_comms_fault_and_zero_the_data);
    RUN(a_timeout_retries_with_activation_first);
    RUN(a_corrupt_response_raises_frame_invalid_not_timeout);
    RUN(a_response_arriving_just_after_the_deadline_is_ignored);
    RUN(an_unsolicited_frame_with_no_request_outstanding_does_not_corrupt_state);
    RUN(polling_continues_across_the_tick_wrap);
    RUN(the_100ms_response_timeout_survives_the_tick_wrap);
    RUN(a_failed_transmit_start_is_reported_and_counted);
    RUN(an_activation_reply_is_never_published_as_data);
    RUN(frame_invalid_is_graded_warning_and_a_timeout_error);
    return TEST_SUMMARY();
}

#include "app_jk.h"
#include "app_timing.h"
#include "bms_errors.h"
#include <string.h>

#define JK_POLL_MS        (1000u)  /* matches the 1000 ms cycle of frames 140-148 */
#define JK_TIMEOUT_MS     (100u)   /* worst-case response is 29.4 ms at 115200 */
#define JK_FAIL_LIMIT     (3u)

/* SN65HVD72: RS_DIR drives DE (active high), RE_DIR drives /RE (active low);
   both LOW at boot (MX_GPIO_Init) means listening. Confirmed by Bartek, 2026-09-11. */
#define DE_ACTIVE         GPIO_PIN_SET
#define DE_IDLE           GPIO_PIN_RESET
#define RE_MUTED          GPIO_PIN_SET
#define RE_LISTENING      GPIO_PIN_RESET

typedef enum { JK_IDLE, JK_SENDING, JK_RECEIVING } JK_State_e;

static UART_HandleTypeDef *uart;
static EH_HandleTypeDef   *ehandler;
/* Written by JK_Task (superloop) and by JK_OnTxComplete (ISR, SENDING ->
   RECEIVING). volatile so the task always re-reads it; see the CAS in the
   SENDING-timeout branch of JK_Task for how the write race is closed. */
static volatile JK_State_e state;
static uint32_t   lastPollMs;
static uint32_t   requestMs;
static uint8_t    failCount;
static bool       needActivation;
static bool       linkValid;
static uint8_t    pendingCmd;          /* command of the request in flight */
static JK_Data_t  data;

static uint8_t  request[JKP_REQUEST_LEN];
static uint8_t  rxBuf[JKP_RX_BUF_LEN];
/* Set by JK_OnRxEvent (ISR), read-then-cleared by JK_Task (superloop). The ISR
   only ever writes rxLen then rxReady, and the task clears rxReady before
   reading rxLen - so no store from either side is ever half-read. */
static volatile uint16_t rxLen;
static volatile uint8_t  rxReady;

static void setDirection(GPIO_PinState de, GPIO_PinState re)
{
    HAL_GPIO_WritePin(RS_DIR_GPIO_Port, RS_DIR_Pin, de);
    HAL_GPIO_WritePin(RE_DIR_GPIO_Port, RE_DIR_Pin, re);
}

static void report(uint16_t code, uint8_t detail)
{
    if (ehandler != NULL) {
        /* Spec 10: code 5 is a warning, code 4 an error. Severity drives
           eviction priority, so a framing glitch must not outrank a fault. */
        const errorSeverity_e severity = (code == BMS_ERR_JK_FRAME_INVALID)
                                         ? ERROR_SEVERITY_WARNING : ERROR_SEVERITY_ERROR;
        const uint8_t blob[5] = { detail, 0u, 0u, 0u, 0u };
        EH_reportEx(ehandler, code, severity, blob, 1u);
    }
}

/* Common tail of both failure paths; returns the new consecutive-failure count. */
static uint8_t endFailedPoll(void)
{
    if (failCount < JK_FAIL_LIMIT) { failCount++; }
    linkValid = false;                     /* this poll did not decode: link is down now */
    if (failCount >= JK_FAIL_LIMIT) {
        /* Three strikes: publish zeroed payloads rather than stale cell voltages. */
        memset(&data, 0, sizeof data);
    }
    setDirection(DE_IDLE, RE_LISTENING);
    state = JK_IDLE;
    return failCount;
}

/* Spec 7.3: only three consecutive failures raise code 4, and only a timeout
   re-arms 0x01 - activation exists for a BMS that has gone to sleep. */
static void onTimeout(void)
{
    const uint8_t failures = endFailedPoll();
    needActivation = true;
    if (failures >= JK_FAIL_LIMIT) { report(BMS_ERR_JK_COMMS_TIMEOUT, failures); }
}

/* Spec 7.3: a per-frame warning, reported at once. A framing glitch is not a
   sleeping BMS, so it must not re-arm activation and halve the poll rate. */
static void onFrameInvalid(uint8_t reason)
{
    (void)endFailedPoll();
    report(BMS_ERR_JK_FRAME_INVALID, reason);
}

static void sendRequest(uint32_t nowMs, uint8_t cmd)
{
    (void)JKP_BuildRequest(request, cmd);
    pendingCmd = cmd;
    setDirection(DE_ACTIVE, RE_MUTED);
    rxReady = 0u;
    rxLen = 0u;
    state = JK_SENDING;
    requestMs = nowMs;
    if (HAL_UART_Transmit_DMA(uart, request, JKP_REQUEST_LEN) != HAL_OK) {
        onTimeout();                       /* the exchange never started */
    }
}

void JK_Init(UART_HandleTypeDef *huart, EH_HandleTypeDef *eh)
{
    uart = huart;
    ehandler = eh;
    state = JK_IDLE;
    lastPollMs = 0u - JK_POLL_MS;   /* first call is immediately due; wrap-safe */
    requestMs = 0u;
    failCount = 0u;
    needActivation = true;                 /* activate once at startup */
    linkValid = false;
    pendingCmd = JKP_CMD_READ_ALL;
    rxReady = 0u;
    rxLen = 0u;
    memset(&data, 0, sizeof data);
    setDirection(DE_IDLE, RE_LISTENING);
}

void JK_OnTxComplete(void)
{
    if (state != JK_SENDING) { return; }
    setDirection(DE_IDLE, RE_LISTENING);
    state = JK_RECEIVING;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(uart, rxBuf, JKP_RX_BUF_LEN);
}

void JK_OnRxEvent(uint16_t size)
{
    /* Clamp rather than trust: a size above the armed buffer is a HAL anomaly,
       and JKP_Validate rejects the frame anyway. */
    rxLen = (size > JKP_RX_BUF_LEN) ? JKP_RX_BUF_LEN : size;
    rxReady = 1u;
}

void JK_Task(uint32_t nowMs)
{
    if (uart == NULL) { return; }

    if (state == JK_RECEIVING) {
        if (rxReady != 0u) {
            rxReady = 0u;
            const uint16_t len = rxLen;
            JK_Data_t decoded;
            if (JKP_Decode(rxBuf, len, &decoded)) {
                if (pendingCmd == JKP_CMD_ACTIVATE) {
                    /* An activation reply is a well-formed frame with no data
                       TLVs: it decodes all-zero. Publishing it would put 0 %
                       SOC and 0 mV cells on the bus as valid readings, and
                       clear the timeout fault, at every boot and reconnect. */
                    needActivation = false;
                } else {
                    data = decoded;
                    linkValid = true;
                    failCount = 0u;
                    needActivation = false;
                    if (ehandler != NULL) {
                        EH_clear(ehandler, BMS_ERR_JK_COMMS_TIMEOUT);
                        EH_clear(ehandler, BMS_ERR_JK_FRAME_INVALID);
                    }
                }
                setDirection(DE_IDLE, RE_LISTENING);
                state = JK_IDLE;
            } else {
                onFrameInvalid((uint8_t)(len & 0xFFu));
            }
        } else if ((uint32_t)(nowMs - requestMs) >= JK_TIMEOUT_MS) {
            onTimeout();
        }
        return;
    }

    if (state == JK_SENDING) {
        if ((uint32_t)(nowMs - requestMs) >= JK_TIMEOUT_MS) {
            /* TC may fire mid-check and commit SENDING->RECEIVING with the RX
               DMA armed. The CAS makes this task the sole writer of the
               SENDING->IDLE edge; if the ISR won, the timeout is dropped and
               JK_RECEIVING picks the exchange up. Spec 7.3. */
            JK_State_e expected = JK_SENDING;
            if (__atomic_compare_exchange_n(&state, &expected, JK_IDLE, false,
                                             __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {
                onTimeout();
            }
        }
        return;
    }

    if (Timing_Due(nowMs, &lastPollMs, JK_POLL_MS)) {
        sendRequest(nowMs, needActivation ? JKP_CMD_ACTIVATE : JKP_CMD_READ_ALL);
    }
}

bool JK_Valid(void) { return linkValid; }
const JK_Data_t *JK_Data(void) { return &data; }

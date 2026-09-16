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
static volatile uint8_t  lineErrBits;
static volatile uint8_t  lineErrPending;
static volatile uint8_t  lineErrCount;   /* line errors within the current poll */

static void setDirection(GPIO_PinState de, GPIO_PinState re)
{
    HAL_GPIO_WritePin(RS_DIR_GPIO_Port, RS_DIR_Pin, de);
    HAL_GPIO_WritePin(RE_DIR_GPIO_Port, RE_DIR_Pin, re);
}

/* Code 5's second payload byte: a short frame and a wiring fault are both
   "invalid frame", and the reason byte alone cannot separate them because a
   length occupies the whole 0-255 range. */
/* A turnaround glitch is one error per poll. Many more is a real line fault, and
   re-arming forever would spin the ISR. */
#define JK_LINE_ERR_LIMIT (4u)

/* Comfortably past the transceiver's 9 us worst-case driver-enable time at
   72 MHz, whatever the compiler makes of the loop. */
#define JK_DE_SETTLE_LOOPS (300u)

/* Pack limits, from the BMSMaster_MasterVoltCurrTemp signal ranges. They key on
   the JK because it measures the cells directly; the master's own divider reads
   30 % low and its Hall sensor is not wired (docs/adc.md).
   JKP_CENTIAMPS_MAX is also 30000, so the decoder rejects any frame past this
   current outright: the test is >= rather than > so the limit is reachable at
   all. Give this a real operational limit below 300 A and an over-current is
   caught with margin instead of exactly at the edge. */
#define PACK_VOLT_MIN_CV    (6300u)    /* 63.0 V */
#define PACK_VOLT_MAX_CV    (8700u)    /* 87.0 V */
#define PACK_CURRENT_MAX_CA (30000)    /* 300.0 A */

#define JK_REASON_FRAME   (0u)   /* detail = received length */
#define JK_REASON_LINE    (1u)   /* detail = USART error bits: PE 1, NE 2, FE 4, ORE 8 */

static void reportFrameInvalid(uint8_t detail, uint8_t kind)
{
    if (ehandler != NULL) {
        const uint8_t blob[5] = { detail, kind, 0u, 0u, 0u };
        EH_reportEx(ehandler, BMS_ERR_JK_FRAME_INVALID, ERROR_SEVERITY_WARNING, blob, 2u);
    }
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

/* Codes 6 and 7 against the JK's own reading, in the decivolt and deciamp units
   the error payloads have always used. Evaluated once per decoded frame, which
   is the rate the data actually changes at. */
static void evaluatePackLimits(void)
{
    if (ehandler == NULL) { return; }

    /* Every field is optional in the TLV walk, so a frame without 0x83 decodes
       to 0. A live pack is never 0.00 V, so that means absent, not flat. */
    const uint16_t dv = (uint16_t)((data.packCentivolts + 5u) / 10u);
    if (data.packCentivolts == 0u) {
        EH_clear(ehandler, BMS_ERR_PACK_VOLT_RANGE);
    } else if (data.packCentivolts < PACK_VOLT_MIN_CV
               || data.packCentivolts > PACK_VOLT_MAX_CV) {
        const uint8_t blob[5] = { (uint8_t)(dv & 0xFFu), (uint8_t)(dv >> 8), 0u, 0u, 0u };
        EH_reportEx(ehandler, BMS_ERR_PACK_VOLT_RANGE, ERROR_SEVERITY_ERROR, blob, 2u);
    } else {
        EH_clear(ehandler, BMS_ERR_PACK_VOLT_RANGE);
    }

    const int16_t da = (int16_t)(data.packCentiamps / 10);
    if (data.packCentiamps >= PACK_CURRENT_MAX_CA || data.packCentiamps <= -PACK_CURRENT_MAX_CA) {
        const uint8_t blob[5] = { (uint8_t)(da & 0xFF), (uint8_t)((da >> 8) & 0xFF), 0u, 0u, 0u };
        EH_reportEx(ehandler, BMS_ERR_PACK_CURRENT_HIGH, ERROR_SEVERITY_ERROR, blob, 2u);
    } else {
        EH_clear(ehandler, BMS_ERR_PACK_CURRENT_HIGH);
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
    /* No reading is not an out-of-range reading; codes 4 and 5 cover a dead
       link on their own. */
    if (ehandler != NULL) {
        EH_clear(ehandler, BMS_ERR_PACK_VOLT_RANGE);
        EH_clear(ehandler, BMS_ERR_PACK_CURRENT_HIGH);
    }
    setDirection(DE_IDLE, RE_LISTENING);
    state = JK_IDLE;
    return failCount;
}

/* Spec 7.3: only three consecutive failures raise code 4. The next poll retries
   the same read - nothing gates it, so the link recovers on the first good
   frame however long it has been down. */
static void onTimeout(void)
{
    const uint8_t failures = endFailedPoll();
    if (failures >= JK_FAIL_LIMIT) { report(BMS_ERR_JK_COMMS_TIMEOUT, failures); }
}

/* Spec 7.3: a per-frame warning, reported at once. The next poll retries the
   same read, so a glitch costs one poll and nothing more. */
static void onFrameInvalid(uint8_t reason)
{
    (void)endFailedPoll();
    reportFrameInvalid(reason, JK_REASON_FRAME);
}

static void sendRequest(uint32_t nowMs, uint8_t cmd)
{
    (void)JKP_BuildRequest(request, cmd);
    pendingCmd = cmd;
    setDirection(DE_ACTIVE, RE_MUTED);

    /*
     * SN65HVD72 driver enable is up to 9 us with the receiver disabled, and one
     * bit at 115200 is 8.68 us. Transmitting immediately puts the start bit on a
     * driver that is still turning on, so the far end sees a corrupt frame and
     * never answers. Wait for the driver before handing the bytes over.
     *
     * Task context, not an ISR, and one poll per second: the cost is noise.
     */
    for (volatile uint32_t i = 0u; i < JK_DE_SETTLE_LOOPS; i++) { }

    rxReady = 0u;
    rxLen = 0u;
    lineErrCount = 0u;
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
    linkValid = false;
    pendingCmd = JKP_CMD_READ_ALL;
    rxReady = 0u;
    rxLen = 0u;
    lineErrPending = 0u;
    lineErrBits = 0u;
    lineErrCount = 0u;
    memset(&data, 0, sizeof data);
    setDirection(DE_IDLE, RE_LISTENING);
}

/* Releasing needs no hold: SN65HVD72 driver disable is 0.4 us max, far inside
   one 8.68 us bit. */
void JK_OnTxComplete(void)
{
    if (state != JK_SENDING) { return; }
    setDirection(DE_IDLE, RE_LISTENING);
    state = JK_RECEIVING;
    /* The line has been idle since before the request, so IDLE is already set:
       arming on it fires an instant zero-length event. */
    __HAL_UART_CLEAR_IDLEFLAG(uart);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(uart, rxBuf, JKP_RX_BUF_LEN);
}

/*
 * ISR context. A line error aborts the DMA, so re-arm or the reply is lost.
 *
 * DE and /RE are one net on this board, so releasing the bus switches the
 * receiver on at the same instant - exactly when the line settles from driven
 * to biased. That edge frames as a spurious byte on nearly every turnaround.
 * Discarding it and listening again costs nothing; ending the poll on it loses
 * a reply that was still arriving. The response timeout stays the backstop.
 *
 * Reporting is left to JK_Task: EH_reportEx walks the scheduler and must not
 * run against the superloop.
 */
void JK_OnUartError(uint32_t errorBits)
{
    if (uart == NULL) { return; }

    lineErrBits = (uint8_t)(errorBits & 0xFFu);
    if (lineErrCount < 0xFFu) { lineErrCount++; }

    if (state != JK_RECEIVING || lineErrCount > JK_LINE_ERR_LIMIT) {
        (void)HAL_UART_AbortReceive(uart);
        lineErrPending = 1u;          /* give up on this poll; JK_Task reports */
        return;
    }

    __HAL_UART_CLEAR_FEFLAG(uart);    /* reads SR then DR: clears the error and drops the byte */
    (void)HAL_UART_AbortReceive(uart);
    __HAL_UART_CLEAR_IDLEFLAG(uart);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(uart, rxBuf, JKP_RX_BUF_LEN);
}

void JK_OnRxEvent(uint16_t size)
{
    if (uart == NULL) { return; }

    /* HAL raises this on the DMA half-transfer and transfer-complete as well as
       on an idle line - 256 bytes into a 512-byte buffer, which lands mid-reply
       for a 306-byte frame. Only idle means the far end has stopped talking;
       the others leave the DMA running, so returning keeps the reply intact. */
    if (HAL_UARTEx_GetRxEventType(uart) != HAL_UART_RXEVENT_IDLE) { return; }

    /* Too short to be a frame: the reply is still on its way. Listening again
       costs nothing; ending the poll here loses it. The response timeout stays
       the backstop. */
    if (size < JKP_FRAME_MIN) {
        __HAL_UART_CLEAR_IDLEFLAG(uart);
        (void)HAL_UARTEx_ReceiveToIdle_DMA(uart, rxBuf, JKP_RX_BUF_LEN);
        return;
    }

    /* Clamp rather than trust: a size above the armed buffer is a HAL anomaly,
       and JKP_Validate rejects the frame anyway. */
    rxLen = (size > JKP_RX_BUF_LEN) ? JKP_RX_BUF_LEN : size;
    rxReady = 1u;
}

void JK_Task(uint32_t nowMs)
{
    if (uart == NULL) { return; }

    if (lineErrPending != 0u) {
        lineErrPending = 0u;
        const uint8_t bits = lineErrBits;
        /* Only reported once the errors outlast JK_LINE_ERR_LIMIT: a single
           turnaround glitch per poll is absorbed in the ISR and never gets here. */
        if (state == JK_RECEIVING) {
            (void)endFailedPoll();
            reportFrameInvalid(bits, JK_REASON_LINE);
        }
    }

    if (state == JK_RECEIVING) {
        if (rxReady != 0u) {
            rxReady = 0u;
            const uint16_t len = rxLen;
            JK_Data_t decoded;
            if (JKP_Decode(rxBuf, len, &decoded)) {
                data = decoded;
                linkValid = true;
                failCount = 0u;
                if (ehandler != NULL) {
                    EH_clear(ehandler, BMS_ERR_JK_COMMS_TIMEOUT);
                    EH_clear(ehandler, BMS_ERR_JK_FRAME_INVALID);
                }
                evaluatePackLimits();
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
        sendRequest(nowMs, JKP_CMD_READ_ALL);
    }
}

bool JK_Valid(void) { return linkValid; }
const JK_Data_t *JK_Data(void) { return &data; }

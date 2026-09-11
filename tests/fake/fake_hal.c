#include "stm32f1xx_hal.h"

static GPIO_TypeDef portA = {0}, portB = {1}, portC = {2}, portD = {3};
GPIO_TypeDef *GPIOA = &portA, *GPIOB = &portB, *GPIOC = &portC, *GPIOD = &portD;

#define MAX_TX 4096
#define MAX_UART 512
#define MAX_RX 64
#define FAKE_FILTER_BANKS 28   /* mirrors CAN_FILTER_BANK_COUNT in can_driver.h */

static uint32_t fakeTick;
static struct { uint32_t id, tick; uint8_t dlc, data[8]; } txLog[MAX_TX];
static uint32_t txCount;
static uint16_t pinState[4];
static uint32_t lastCompare;
static uint8_t  uartTx[MAX_UART]; static uint16_t uartTxLen;
static uint8_t  uartRxQueued[MAX_UART]; static uint16_t uartRxQueuedLen;
static uint8_t *uartRxDest; static uint16_t uartRxCap;
static uint32_t capturedValue[16];
static uint32_t compareValue[16];
static HAL_TIM_ChannelStateTypeDef channelState[16];
static uint8_t  uartTxFail;

/* Decoded bxCAN filter banks, keyed by bank index and owning hcan - enough to
   reproduce hardware admit/reject behaviour for Fake_QueueCanRx. */
static struct {
    uint8_t active;
    uint32_t mode;
    uint16_t id[4];
    uint16_t mask;
    CAN_HandleTypeDef *owner;
} filterBanks[FAKE_FILTER_BANKS];

static struct { CAN_HandleTypeDef *h; uint32_t id; uint8_t dlc, data[8]; } rxQ[MAX_RX];
static uint32_t rxHead, rxTail;

/* bxCAN has exactly three TX mailboxes per peripheral, each held until its
   frame is on the wire: one 8-byte frame at 500 kbit/s takes ~216 us, below the
   1 ms resolution of the fake tick, so a mailbox is modelled as busy for one
   tick - slower than hardware, never faster. Without this a test cannot observe
   mailbox exhaustion and a starving scheduler looks healthy. */
#define FAKE_TX_MAILBOXES 3u
#define FAKE_TX_TICKS     1u
#define FAKE_CAN_UNITS    2u

static struct {
    CAN_HandleTypeDef *owner;
    uint8_t  busy[FAKE_TX_MAILBOXES];
    uint32_t freeAtTick[FAKE_TX_MAILBOXES];
} txUnit[FAKE_CAN_UNITS];
static uint8_t  canTxHold;        /* mailboxes never free: an unacknowledged frame */
static uint32_t canAbortCount;

void Fake_Reset(void)
{
    fakeTick = 0; txCount = 0; lastCompare = 0;
    uartTxLen = 0; uartRxQueuedLen = 0; uartRxDest = NULL; uartRxCap = 0;
    uartTxFail = 0;
    rxHead = 0; rxTail = 0;
    canTxHold = 0; canAbortCount = 0;
    memset(txUnit, 0, sizeof txUnit);
    memset(pinState, 0, sizeof pinState);
    memset(capturedValue, 0, sizeof capturedValue);
    memset(compareValue, 0, sizeof compareValue);
    memset(filterBanks, 0, sizeof filterBanks);
    memset(rxQ, 0, sizeof rxQ);
    /* On hardware, MX_TIM3_Init() -> HAL_TIM_PWM_Init() puts every channel in
       READY before app_main() runs. Match that instead of defaulting to RESET,
       or PWM_Out_Init() sees a state the real board never has. */
    for (uint32_t i = 0; i < 16u; i++) { channelState[i] = HAL_TIM_CHANNEL_STATE_READY; }
}

void     Fake_SetTick(uint32_t ms) { fakeTick = ms; }
uint32_t HAL_GetTick(void)         { return fakeTick; }
uint32_t Fake_TxCount(void)        { return txCount; }
uint32_t Fake_LastCompare(void)    { return lastCompare; }

void Fake_SetCompare(TIM_HandleTypeDef *h, uint32_t ch, uint32_t v)
{
    compareValue[ch / 4u] = v;
    /* TIM_InstanceTypeDef only backs CCR3 (channel 3); mirror the register
       write there when that is genuinely the channel being set, instead of
       hardcoding it regardless of ch. */
    if (ch == TIM_CHANNEL_3) { h->Instance->CCR3 = v; }
    lastCompare = v;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) { pinState[port->port] |= pin; }
    else                       { pinState[port->port] &= (uint16_t)~pin; }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (pinState[port->port] & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin) { pinState[port->port] ^= pin; }
GPIO_PinState Fake_PinState(GPIO_TypeDef *port, uint16_t pin) { return HAL_GPIO_ReadPin(port, pin); }
void Fake_SetPin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s) { HAL_GPIO_WritePin(port, pin, s); }

/* Standard ID occupies bits [15:5] of the filter registers in both list and
   mask mode (STM32F1 bxCAN); decode back to a plain 11-bit id here so
   Fake_QueueCanRx can replicate admit/reject without re-deriving the shift. */
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f)
{
    if (f->FilterBank >= FAKE_FILTER_BANKS) { return HAL_OK; }
    filterBanks[f->FilterBank].active = 1u;
    filterBanks[f->FilterBank].mode   = f->FilterMode;
    filterBanks[f->FilterBank].owner  = h;
    if (f->FilterMode == CAN_FILTERMODE_IDLIST) {
        filterBanks[f->FilterBank].id[0] = (uint16_t)(f->FilterIdHigh >> 5);
        filterBanks[f->FilterBank].id[1] = (uint16_t)(f->FilterIdLow >> 5);
        filterBanks[f->FilterBank].id[2] = (uint16_t)(f->FilterMaskIdHigh >> 5);
        filterBanks[f->FilterBank].id[3] = (uint16_t)(f->FilterMaskIdLow >> 5);
    } else {
        filterBanks[f->FilterBank].id[0] = (uint16_t)(f->FilterIdHigh >> 5);
        filterBanks[f->FilterBank].mask  = (uint16_t)(f->FilterMaskIdHigh >> 5);
    }
    return HAL_OK;
}

static int filterAccepts(CAN_HandleTypeDef *h, uint32_t stdId)
{
    for (uint32_t i = 0; i < FAKE_FILTER_BANKS; i++) {
        if (!filterBanks[i].active || filterBanks[i].owner != h) { continue; }
        if (filterBanks[i].mode == CAN_FILTERMODE_IDLIST) {
            if (filterBanks[i].id[0] == stdId || filterBanks[i].id[1] == stdId ||
                filterBanks[i].id[2] == stdId || filterBanks[i].id[3] == stdId) { return 1; }
        } else {
            if ((stdId & filterBanks[i].mask) == (filterBanks[i].id[0] & filterBanks[i].mask)) { return 1; }
        }
    }
    return 0;
}

void Fake_QueueCanRx(CAN_HandleTypeDef *h, uint32_t stdId, const uint8_t *data, uint8_t dlc)
{
    if (!filterAccepts(h, stdId)) { return; }        /* hardware would drop this silently */
    if (((rxHead + 1u) % MAX_RX) == rxTail) { return; }
    rxQ[rxHead].h = h; rxQ[rxHead].id = stdId; rxQ[rxHead].dlc = dlc;
    memset(rxQ[rxHead].data, 0, 8);
    if (data != NULL) { memcpy(rxQ[rxHead].data, data, (dlc > 8u) ? 8u : dlc); }
    rxHead = (rxHead + 1u) % MAX_RX;
}

uint32_t Fake_RxPending(CAN_HandleTypeDef *h)
{
    uint32_t n = 0u;
    for (uint32_t i = rxTail; i != rxHead; i = (i + 1u) % MAX_RX) {
        if (rxQ[i].h == h) { n++; }
    }
    return n;
}

HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *hdr, uint8_t *data)
{
    UNUSED(fifo);
    for (uint32_t i = rxTail; i != rxHead; i = (i + 1u) % MAX_RX) {
        if (rxQ[i].h != h) { continue; }
        memset(hdr, 0, sizeof *hdr);
        hdr->StdId = rxQ[i].id; hdr->IDE = CAN_ID_STD; hdr->DLC = rxQ[i].dlc;
        memcpy(data, rxQ[i].data, 8);
        rxQ[i].h = NULL;
        while (rxTail != rxHead && rxQ[rxTail].h == NULL) { rxTail = (rxTail + 1u) % MAX_RX; }
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h) { UNUSED(h); return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t it) { UNUSED(h); UNUSED(it); return HAL_OK; }
/* CAN1 and CAN2 have their own mailboxes; bind a unit to each handle on first
   use so one peripheral's traffic cannot block the other. */
static uint32_t canUnitOf(CAN_HandleTypeDef *h)
{
    for (uint32_t u = 0; u < FAKE_CAN_UNITS; u++) { if (txUnit[u].owner == h) { return u; } }
    for (uint32_t u = 0; u < FAKE_CAN_UNITS; u++) {
        if (txUnit[u].owner == NULL) { txUnit[u].owner = h; return u; }
    }
    return 0u;
}

HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t m)
{
    const uint32_t u = canUnitOf(h);
    canAbortCount++;
    for (uint32_t i = 0; i < FAKE_TX_MAILBOXES; i++) {
        if (m & ((uint32_t)CAN_TX_MAILBOX0 << i)) { txUnit[u].busy[i] = 0u; }
    }
    return HAL_OK;
}

void     Fake_HoldCanTx(int on)     { canTxHold = on ? 1u : 0u; }
uint32_t Fake_CanAbortCount(void)   { return canAbortCount; }

uint32_t Fake_CanTxMailboxesFree(CAN_HandleTypeDef *h)
{
    const uint32_t u = canUnitOf(h);
    uint32_t n = 0u;
    for (uint32_t i = 0; i < FAKE_TX_MAILBOXES; i++) {
        if (!txUnit[u].busy[i] || (!canTxHold && fakeTick >= txUnit[u].freeAtTick[i])) { n++; }
    }
    return n;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *hdr,
                                       uint8_t *data, uint32_t *mailbox)
{
    if (txCount >= MAX_TX) { return HAL_ERROR; }
    const uint32_t u = canUnitOf(h);
    for (uint32_t i = 0; i < FAKE_TX_MAILBOXES; i++) {
        if (txUnit[u].busy[i] && !canTxHold && fakeTick >= txUnit[u].freeAtTick[i]) {
            txUnit[u].busy[i] = 0u;
        }
    }
    for (uint32_t i = 0; i < FAKE_TX_MAILBOXES; i++) {
        if (txUnit[u].busy[i]) { continue; }
        txUnit[u].busy[i]       = 1u;
        txUnit[u].freeAtTick[i] = fakeTick + FAKE_TX_TICKS;
        txLog[txCount].id   = hdr->StdId;
        txLog[txCount].tick = fakeTick;
        txLog[txCount].dlc  = (uint8_t)hdr->DLC;
        memcpy(txLog[txCount].data, data, 8);
        txCount++;
        if (mailbox) { *mailbox = (uint32_t)CAN_TX_MAILBOX0 << i; }
        return HAL_OK;
    }
    return HAL_ERROR;                        /* all three in flight, as on hardware */
}

int Fake_FindTx(uint32_t stdId, uint8_t out[8], uint32_t *atTick)
{
    for (uint32_t i = txCount; i > 0u; i--) {
        if (txLog[i - 1u].id == stdId) {
            if (out)    { memcpy(out, txLog[i - 1u].data, 8); }
            if (atTick) { *atTick = txLog[i - 1u].tick; }
            return 1;
        }
    }
    return 0;
}

uint32_t Fake_TxCountFor(uint32_t stdId)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < txCount; i++) { if (txLog[i].id == stdId) { n++; } }
    return n;
}

void Fake_ForceUartTxFail(void) { uartTxFail = 1; }

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n)
{
    UNUSED(h);
    if (uartTxFail) { uartTxFail = 0; return HAL_ERROR; }
    uartTxLen = (n > MAX_UART) ? MAX_UART : n;
    memcpy(uartTx, d, uartTxLen);
    return HAL_OK;
}
uint16_t Fake_LastUartTx(uint8_t *out, uint16_t cap)
{
    uint16_t n = (uartTxLen > cap) ? cap : uartTxLen;
    memcpy(out, uartTx, n);
    return n;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n)
{
    UNUSED(h);
    uartRxDest = d; uartRxCap = n;
    if (uartRxQueuedLen > 0u) {
        uint16_t len = (uartRxQueuedLen > n) ? n : uartRxQueuedLen;
        memcpy(d, uartRxQueued, len);
        uartRxQueuedLen = 0;
        HAL_UARTEx_RxEventCallback(h, len);
    }
    return HAL_OK;
}
void Fake_QueueUartRx(const uint8_t *data, uint16_t len)
{
    uartRxQueuedLen = (len > MAX_UART) ? MAX_UART : len;
    memcpy(uartRxQueued, data, uartRxQueuedLen);
}

/* Weak default: a test that cares about received data provides a strong override. */
__attribute__((weak)) void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size)
{
    UNUSED(h); UNUSED(size);
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch)
{
    UNUSED(h);
    uint32_t idx = ch / 4u;
    if (channelState[idx] == HAL_TIM_CHANNEL_STATE_BUSY) { return HAL_ERROR; }
    channelState[idx] = HAL_TIM_CHANNEL_STATE_BUSY;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *h, TIM_IC_InitTypeDef *cfg, uint32_t ch)
{
    UNUSED(h); UNUSED(cfg); UNUSED(ch);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *h, uint32_t ch)
{
    UNUSED(h);
    channelState[ch / 4u] = HAL_TIM_CHANNEL_STATE_BUSY;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *h, uint32_t ch)
{
    return HAL_TIM_IC_Start(h, ch);
}

uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *h, uint32_t ch)
{
    UNUSED(h);
    return capturedValue[ch / 4u];
}

HAL_TIM_ChannelStateTypeDef TIM_CHANNEL_STATE_GET(TIM_HandleTypeDef *h, uint32_t ch)
{
    UNUSED(h);
    return channelState[ch / 4u];
}

void Error_Handler(void) { }

/* Linked only by tests that build app.c; harmless elsewhere. */
uint32_t uwTick;
uint32_t uwTickFreq = 1u;

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t pre, uint32_t sub)
{
    UNUSED(irq); UNUSED(pre); UNUSED(sub);
}
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { UNUSED(irq); }
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h) { UNUSED(h); return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *buf, uint32_t len)
{
    UNUSED(h); UNUSED(buf); UNUSED(len);
    return HAL_OK;
}

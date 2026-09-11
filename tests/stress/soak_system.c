/*
 * soak_system.c - long-soak whole-system simulation of the BMS Master firmware.
 *
 * Why this exists: every other test in tests/ drives one module for a few
 * simulated seconds. Nothing has ever run the whole superloop for weeks of
 * simulated time, and nothing has interleaved the five ISRs against it at
 * random. Two bug classes only appear that way - time-dependent drift across
 * the 49.7-day uint32 tick wrap, and interleaving-dependent state corruption.
 *
 * Structure:
 *   part 1  a private HAL backend implementing tests/fake/stm32f1xx_hal.h.
 *           It is NOT fake_hal.c: that one logs at most 4096 TX frames and
 *           cannot be reset without also resetting the tick, so it cannot
 *           survive a multi-billion-frame soak. The shared fake is left
 *           untouched, as instructed; this file substitutes for it at link
 *           time and models the same hardware (3 TX mailboxes per bxCAN unit,
 *           28 shared filter banks, RS485 direction pins, TIM3 CCR3).
 *   part 2  the real application, textually included so that the genuine
 *           static step() and initAll() and the genuine static EH_HandleTypeDef
 *           are driven and observed. No application file is modified.
 *   part 3  the soak driver: world model, ISR injection, invariant checks and
 *           the four campaigns.
 *
 * ISR injection is priority-aware: hooks sit inside the HAL entry points that
 * step() actually calls, so an ISR really does land in the middle of
 * CAN_HandleScheduled, JK_Task or CONTACTOR_Task, and only a strictly
 * higher-priority handler may nest inside a running one - the NVIC rule for
 * spec 3.4's priorities (CAN1/CAN2 RX0 = 2, ADC DMA = 3, USART1 = 4).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"

/* ========================================================================== *
 * part 0: failure reporting and the event trace
 * ========================================================================== */

#define SOAK_TRACE_LEN   512u
#define SOAK_MAX_REPORTS 120

typedef struct {
    uint32_t    tick;
    uint64_t    pass;
    const char *what;
    uint32_t    a, b;
} SoakEvent;

static SoakEvent soakTrace[SOAK_TRACE_LEN];
static uint64_t  soakTraceSeq;
static uint64_t  soakSeed;
static const char *soakCampaign = "init";
static uint64_t  soakPass;
static uint32_t  soakTick;
static uint32_t  soakViolations;
static uint32_t  soakReported;
static uint64_t  soakIsrEvents;

static void soakTraceAdd(const char *what, uint32_t a, uint32_t b)
{
    SoakEvent *e = &soakTrace[soakTraceSeq % SOAK_TRACE_LEN];
    e->tick = soakTick; e->pass = soakPass; e->what = what; e->a = a; e->b = b;
    soakTraceSeq++;
}

static void soakDumpTrace(void)
{
    const uint64_t n = (soakTraceSeq < SOAK_TRACE_LEN) ? soakTraceSeq : SOAK_TRACE_LEN;
    printf("  --- last %llu events (oldest first) ---\n", (unsigned long long)n);
    for (uint64_t i = 0; i < n; i++) {
        const SoakEvent *e = &soakTrace[(soakTraceSeq - n + i) % SOAK_TRACE_LEN];
        printf("  [pass %-12llu tick 0x%08X] %-22s a=%-10u b=%u\n",
               (unsigned long long)e->pass, e->tick,
               e->what ? e->what : "?", e->a, e->b);
    }
    printf("  --- end trace ---\n");
}

#define SOAK_FAIL(...) do {                                                    \
    soakViolations++;                                                          \
    if (soakReported < SOAK_MAX_REPORTS) {                                      \
        soakReported++;                                                        \
        printf("\n*** VIOLATION #%u  campaign=%s  seed=%llu  tick=0x%08X (%u) "  \
               "pass=%llu\n    ", soakViolations, soakCampaign,                 \
               (unsigned long long)soakSeed, soakTick, soakTick,               \
               (unsigned long long)soakPass);                                   \
        printf(__VA_ARGS__);                                                   \
        printf("\n    at %s:%d\n", __FILE__, __LINE__);                        \
        soakDumpTrace();                                                       \
        fflush(stdout);                                                        \
    }                                                                          \
} while (0)

/* Sanitizer aborts bypass atexit, so hook the death callback when the runtime
   provides one; otherwise the periodic progress line is the last context. */
void __sanitizer_set_death_callback(void (*cb)(void)) __attribute__((weak));
static void soakOnDeath(void)
{
    printf("\n*** SANITIZER ABORT  campaign=%s  seed=%llu  tick=0x%08X  pass=%llu\n",
           soakCampaign, (unsigned long long)soakSeed, soakTick,
           (unsigned long long)soakPass);
    soakDumpTrace();
    fflush(stdout);
}

/* ========================================================================== *
 * part 1: private HAL backend (link substitute for tests/fake/fake_hal.c)
 * ========================================================================== */

static GPIO_TypeDef portA = {0}, portB = {1}, portC = {2}, portD = {3};
GPIO_TypeDef *GPIOA = &portA, *GPIOB = &portB, *GPIOC = &portC, *GPIOD = &portD;

uint32_t uwTick;
uint32_t uwTickFreq = 1u;

static uint16_t halPinState[4];
static uint32_t halGpioConfigured[4];
static GPIO_TypeDef *halGpioClocked[4];
static uint32_t halCompare[16];
static HAL_TIM_ChannelStateTypeDef halChannelState[16];
static uint32_t halCapturedValue[16];
static uint32_t halErrorHandlerCalls;

/* ---- CAN: 3 TX mailboxes per unit, busy for one tick, as on hardware ---- */
#define HAL_TX_MAILBOXES 3u
#define HAL_TX_TICKS     1u
#define HAL_CAN_UNITS    2u

static struct {
    CAN_HandleTypeDef *owner;
    uint8_t  busy[HAL_TX_MAILBOXES];
    uint32_t freeAtTick[HAL_TX_MAILBOXES];
} halTxUnit[HAL_CAN_UNITS];

/* Per-ID transmit statistics. Unbounded in time, O(1) in memory: this is the
   whole point of not reusing the shared fake's 4096-entry TX log. */
#define HAL_TXSTAT_SLOTS 64u
typedef struct {
    uint32_t id;
    uint8_t  used;
    uint64_t count;
    uint32_t lastTick;
    uint32_t maxGap;
    uint32_t maxGapAtTick;
    uint8_t  lastData[8];
} HalTxStat;
static HalTxStat halTxStat[HAL_TXSTAT_SLOTS];
static uint64_t  halTxTotal;

static HalTxStat *halTxSlot(uint32_t id)
{
    uint32_t h = (id * 2654435761u) % HAL_TXSTAT_SLOTS;
    for (uint32_t probe = 0; probe < HAL_TXSTAT_SLOTS; probe++) {
        HalTxStat *s = &halTxStat[(h + probe) % HAL_TXSTAT_SLOTS];
        if (!s->used) { s->used = 1u; s->id = id; s->lastTick = soakTick; return s; }
        if (s->id == id) { return s; }
    }
    return NULL;
}

#define HAL_FILTER_BANKS 28u
static struct {
    uint8_t  active;
    uint32_t mode;
    uint16_t id[4];
    uint16_t mask;
    CAN_HandleTypeDef *owner;
} halFilter[HAL_FILTER_BANKS];

#define HAL_RX_RING 1024u
static struct { CAN_HandleTypeDef *h; uint32_t id; uint8_t dlc, data[8]; } halRxQ[HAL_RX_RING];
static uint32_t halRxHead, halRxTail;
static uint64_t halRxDropped;

/* ---- UART / RS485 ---- */
static uint8_t  halUartTx[64];
static uint16_t halUartTxLen;
static uint64_t halUartTxCount;
static uint32_t halUartTxLastTick;
static uint32_t halUartTxMaxGap;
static uint32_t halUartTxMaxGapAt;
static uint8_t *halUartRxDest;
static uint16_t halUartRxCap;
static uint64_t halUartRxArmCount;

/* ---- injection hook, installed by part 3 ---- */
static void soakInjectHook(void);
static int  soakInjectArmed;

static void halHook(void) { if (soakInjectArmed) { soakInjectHook(); } }

uint32_t HAL_GetTick(void) { return soakTick; }

static int halGpioSlot(GPIO_TypeDef *port)
{
    for (int i = 0; i < 4; i++) { if (halGpioClocked[i] == port) { return i; } }
    for (int i = 0; i < 4; i++) {
        if (halGpioClocked[i] == NULL) { halGpioClocked[i] = port; return i; }
    }
    return 0;
}

void Fake_EnableGpioClock(GPIO_TypeDef *port) { (void)halGpioSlot(port); }

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *cfg)
{
    halGpioConfigured[halGpioSlot(port)] |= cfg->Pin;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) { halPinState[port->port] |= pin; }
    else                       { halPinState[port->port] = (uint16_t)(halPinState[port->port] & ~pin); }
    halHook();   /* setDirection(), LED_Handle() and CAN_App_Init() land here */
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (halPinState[port->port] & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    halPinState[port->port] ^= pin;
}

void Fake_SetCompare(TIM_HandleTypeDef *h, uint32_t ch, uint32_t v)
{
    halCompare[ch / 4u] = v;
    if (ch == TIM_CHANNEL_3) { h->Instance->CCR3 = v; }
    halHook();   /* lands inside CONTACTOR_Task, between state and duty */
}

static uint32_t halCcr3(void) { return halCompare[TIM_CHANNEL_3 / 4u]; }

HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f)
{
    if (f->FilterBank >= HAL_FILTER_BANKS) { return HAL_OK; }
    halFilter[f->FilterBank].active = 1u;
    halFilter[f->FilterBank].mode   = f->FilterMode;
    halFilter[f->FilterBank].owner  = h;
    if (f->FilterMode == CAN_FILTERMODE_IDLIST) {
        halFilter[f->FilterBank].id[0] = (uint16_t)(f->FilterIdHigh >> 5);
        halFilter[f->FilterBank].id[1] = (uint16_t)(f->FilterIdLow >> 5);
        halFilter[f->FilterBank].id[2] = (uint16_t)(f->FilterMaskIdHigh >> 5);
        halFilter[f->FilterBank].id[3] = (uint16_t)(f->FilterMaskIdLow >> 5);
    } else {
        halFilter[f->FilterBank].id[0] = (uint16_t)(f->FilterIdHigh >> 5);
        halFilter[f->FilterBank].mask  = (uint16_t)(f->FilterMaskIdHigh >> 5);
    }
    return HAL_OK;
}

static int halFilterAccepts(CAN_HandleTypeDef *h, uint32_t stdId)
{
    for (uint32_t i = 0; i < HAL_FILTER_BANKS; i++) {
        if (!halFilter[i].active || halFilter[i].owner != h) { continue; }
        if (halFilter[i].mode == CAN_FILTERMODE_IDLIST) {
            if (halFilter[i].id[0] == stdId || halFilter[i].id[1] == stdId ||
                halFilter[i].id[2] == stdId || halFilter[i].id[3] == stdId) { return 1; }
        } else if ((stdId & halFilter[i].mask) == (halFilter[i].id[0] & halFilter[i].mask)) {
            return 1;
        }
    }
    return 0;
}

static void halQueueCanRx(CAN_HandleTypeDef *h, uint32_t stdId, const uint8_t *data, uint8_t dlc)
{
    if (!halFilterAccepts(h, stdId)) { return; }         /* hardware drops silently */
    if (((halRxHead + 1u) % HAL_RX_RING) == halRxTail) { halRxDropped++; return; }
    halRxQ[halRxHead].h = h; halRxQ[halRxHead].id = stdId; halRxQ[halRxHead].dlc = dlc;
    memset(halRxQ[halRxHead].data, 0, 8);
    if (data != NULL) { memcpy(halRxQ[halRxHead].data, data, (dlc > 8u) ? 8u : dlc); }
    halRxHead = (halRxHead + 1u) % HAL_RX_RING;
}

HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                       CAN_RxHeaderTypeDef *hdr, uint8_t *data)
{
    UNUSED(fifo);
    for (uint32_t i = halRxTail; i != halRxHead; i = (i + 1u) % HAL_RX_RING) {
        if (halRxQ[i].h != h) { continue; }
        memset(hdr, 0, sizeof *hdr);
        hdr->StdId = halRxQ[i].id; hdr->IDE = CAN_ID_STD; hdr->DLC = halRxQ[i].dlc;
        memcpy(data, halRxQ[i].data, 8);
        halRxQ[i].h = NULL;
        while (halRxTail != halRxHead && halRxQ[halRxTail].h == NULL) {
            halRxTail = (halRxTail + 1u) % HAL_RX_RING;
        }
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h) { UNUSED(h); return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t it)
{
    UNUSED(h); UNUSED(it); return HAL_OK;
}

static uint32_t halCanUnit(CAN_HandleTypeDef *h)
{
    for (uint32_t u = 0; u < HAL_CAN_UNITS; u++) { if (halTxUnit[u].owner == h) { return u; } }
    for (uint32_t u = 0; u < HAL_CAN_UNITS; u++) {
        if (halTxUnit[u].owner == NULL) { halTxUnit[u].owner = h; return u; }
    }
    return 0u;
}

static uint8_t  halCanTxHold;
static uint32_t halCanAbortCount;

HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t m)
{
    const uint32_t u = halCanUnit(h);
    halCanAbortCount++;
    for (uint32_t i = 0; i < HAL_TX_MAILBOXES; i++) {
        if (m & ((uint32_t)CAN_TX_MAILBOX0 << i)) { halTxUnit[u].busy[i] = 0u; }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *hdr,
                                       uint8_t *data, uint32_t *mailbox)
{
    const uint32_t u = halCanUnit(h);
    halHook();   /* lands inside CAN_HandleScheduled's loop over the scheduler */

    for (uint32_t i = 0; i < HAL_TX_MAILBOXES; i++) {
        if (halTxUnit[u].busy[i] && !halCanTxHold && soakTick >= halTxUnit[u].freeAtTick[i]) {
            halTxUnit[u].busy[i] = 0u;
        }
    }
    for (uint32_t i = 0; i < HAL_TX_MAILBOXES; i++) {
        if (halTxUnit[u].busy[i]) { continue; }
        halTxUnit[u].busy[i]       = 1u;
        halTxUnit[u].freeAtTick[i] = soakTick + HAL_TX_TICKS;
        halTxTotal++;
        HalTxStat *s = halTxSlot(hdr->StdId);
        if (s != NULL) {
            if (s->count > 0u) {
                const uint32_t gap = soakTick - s->lastTick;
                if (gap > s->maxGap) { s->maxGap = gap; s->maxGapAtTick = soakTick; }
            }
            s->count++;
            s->lastTick = soakTick;
            memcpy(s->lastData, data, 8);
        }
        if (mailbox) { *mailbox = (uint32_t)CAN_TX_MAILBOX0 << i; }
        return HAL_OK;
    }
    return HAL_ERROR;                        /* all three in flight, as on hardware */
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n)
{
    UNUSED(h);
    halUartTxLen = (n > sizeof halUartTx) ? (uint16_t)sizeof halUartTx : n;
    memcpy(halUartTx, d, halUartTxLen);
    if (halUartTxCount > 0u) {
        const uint32_t gap = soakTick - halUartTxLastTick;
        if (gap > halUartTxMaxGap) { halUartTxMaxGap = gap; halUartTxMaxGapAt = soakTick; }
    }
    halUartTxCount++;
    halUartTxLastTick = soakTick;
    halHook();   /* lands inside JK_Task's sendRequest */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n)
{
    UNUSED(h);
    halUartRxDest = d; halUartRxCap = n;
    halUartRxArmCount++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch)
{
    UNUSED(h);
    const uint32_t idx = ch / 4u;
    if (halChannelState[idx] == HAL_TIM_CHANNEL_STATE_BUSY) { return HAL_ERROR; }
    halChannelState[idx] = HAL_TIM_CHANNEL_STATE_BUSY;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *h, TIM_IC_InitTypeDef *c, uint32_t ch)
{
    UNUSED(h); UNUSED(c); UNUSED(ch); return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *h, uint32_t ch)
{
    UNUSED(h); halChannelState[ch / 4u] = HAL_TIM_CHANNEL_STATE_BUSY; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *h, uint32_t ch)
{
    return HAL_TIM_IC_Start(h, ch);
}
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *h, uint32_t ch)
{
    UNUSED(h); return halCapturedValue[ch / 4u];
}
HAL_TIM_ChannelStateTypeDef TIM_CHANNEL_STATE_GET(TIM_HandleTypeDef *h, uint32_t ch)
{
    UNUSED(h); return halChannelState[ch / 4u];
}

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t pre, uint32_t sub)
{
    UNUSED(irq); UNUSED(pre); UNUSED(sub);
}
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { UNUSED(irq); }
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h) { UNUSED(h); return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *buf, uint32_t len)
{
    UNUSED(h); UNUSED(buf); UNUSED(len); return HAL_OK;
}

void Error_Handler(void) { halErrorHandlerCalls++; }

static void halInit(void)
{
    memset(halPinState, 0, sizeof halPinState);
    memset(halGpioConfigured, 0, sizeof halGpioConfigured);
    memset(halGpioClocked, 0, sizeof halGpioClocked);
    memset(halCompare, 0, sizeof halCompare);
    memset(halCapturedValue, 0, sizeof halCapturedValue);
    memset(halTxUnit, 0, sizeof halTxUnit);
    memset(halTxStat, 0, sizeof halTxStat);
    memset(halFilter, 0, sizeof halFilter);
    memset(halRxQ, 0, sizeof halRxQ);
    halRxHead = halRxTail = 0u; halRxDropped = 0u;
    halTxTotal = 0u; halCanTxHold = 0u; halCanAbortCount = 0u;
    halUartTxLen = 0u; halUartTxCount = 0u; halUartTxLastTick = 0u;
    halUartTxMaxGap = 0u; halUartTxMaxGapAt = 0u;
    halUartRxDest = NULL; halUartRxCap = 0u; halUartRxArmCount = 0u;
    halErrorHandlerCalls = 0u;
    /* MX_TIM3_Init() puts every channel in READY before app_main() runs. */
    for (uint32_t i = 0; i < 16u; i++) { halChannelState[i] = HAL_TIM_CHANNEL_STATE_READY; }
}

/* ========================================================================== *
 * part 2: the real application, textually included
 * ========================================================================== */

/* app.c's externs; the same set tests/test_app_fatal.c defines. */
static CAN_InstanceTypeDef  can1Inst, can2Inst;
CAN_HandleTypeDef    hcan1 = { &can1Inst, { DISABLE } };
CAN_HandleTypeDef    hcan2 = { &can2Inst, { DISABLE } };
ADC_HandleTypeDef    hadc1;
static TIM_InstanceTypeDef  tim3Inst = { 999u, 0u, 0u };   /* ARR 999 = 1 kHz */
TIM_HandleTypeDef    htim3 = { &tim3Inst };
static UART_InstanceTypeDef uart1Inst;
UART_HandleTypeDef   huart1 = { &uart1Inst };

#include "../../App/Src/app.c"

/* ========================================================================== *
 * part 3: the soak driver
 * ========================================================================== */

/* ---- deterministic RNG (xorshift64*), seed printed on every run ---------- */
static uint64_t soakRngState;
static uint32_t soakRnd(void)
{
    uint64_t x = soakRngState;
    x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
    soakRngState = x;
    return (uint32_t)((x * 0x2545F4914F6CDD1DULL) >> 32);
}
static uint32_t soakRndBelow(uint32_t n) { return (n == 0u) ? 0u : (soakRnd() % n); }

/* ---- absolute simulated time -------------------------------------------- *
 * 64-bit, so the reference models below do NOT share the firmware's uint32
 * wrap. A divergence between the two is exactly the bug class campaign 1
 * exists to find.
 * ------------------------------------------------------------------------- */
static uint64_t soakAbsMs;
static uint32_t soakTickBase;         /* soakTick == (uint32_t)(soakTickBase + soakAbsMs) */

static void soakSetTime(uint64_t absMs)
{
    soakAbsMs = absMs;
    soakTick  = (uint32_t)(soakTickBase + absMs);
    uwTick    = soakTick;
    LED_SetSyncTick(soakTick);        /* HAL_IncTick would do this once per ms */
}

/* ---- world model --------------------------------------------------------- */
typedef struct {
    uint16_t adcTemp, adcCurr, adcVolt;
    uint8_t  thermBase;      /* fed to the 61 ordinary thermistors */
    uint8_t  thermProbe;     /* fed to module 1 thermistor 1: the liveness probe */
    uint8_t  thermHot;       /* fed to module 7 thermistor 9: the overtemp lever */
    int      feedTherm;
    int      feedActiv;
    int      feedNode;
    int      jkReply;        /* 0 = JK link dead */
    int      adcRun;         /* 0 = the ADC DMA stream has stopped */
    int      canRxPath;      /* 1 = through filters + RX FIFO + the real ISR   */
} SoakWorld;

static SoakWorld world;
static volatile uint16_t *soakAdcBuf;   /* app.c's adcBuf, captured after init  */

static void soakProbeReset(void);

static void soakWorldHealthy(void)
{
    world.adcTemp   = 2100u;   /* ~26 degC on the NTC table, inside the guards   */
    world.adcCurr   = 2108u;   /* the zero-current calibration point             */
    world.adcVolt   = 3000u;   /* 68.6 V, inside 63..87 V                        */
    world.thermBase = 110u;    /* ~43 degC raw, below the 153 limit              */
    world.thermProbe= 100u;
    world.thermHot  = 120u;
    world.feedTherm = 1;
    world.feedActiv = 1;
    world.feedNode  = 1;
    world.jkReply   = 1;
    world.adcRun    = 1;
    world.canRxPath = 1;
    soakProbeReset();
}

/* ---- thermistor frame id map (mirrors app_therm.c's MODULE_OF/THERM_OF) --- */
static uint32_t soakThermId(uint8_t module, uint8_t therm)
{
    const uint32_t base = 10u * (uint32_t)(module + 20u);
    return (module & 1u) ? (base + therm) : (base + (10u - therm));
}

/* ---- interrupt model ----------------------------------------------------- *
 * Priority numbers from spec 3.4, asserted by app.c's initAll(). Lower number
 * = higher priority; only a strictly higher-priority handler may nest.
 * ------------------------------------------------------------------------- */
#define ISR_CAN1_RX  0
#define ISR_CAN2_RX  1
#define ISR_ADC_DMA  2
#define ISR_UART_TX  3
#define ISR_UART_RX  4
#define ISR_COUNT    5
#define PRIO_MAIN    255

static const int soakIsrPrio[ISR_COUNT] = { 2, 2, 3, 4, 4 };
static const char *soakIsrName[ISR_COUNT] =
    { "isr:CAN1_RX0", "isr:CAN2_RX0", "isr:DMA1_CH1", "isr:USART1_TC", "isr:USART1_RX" };
static int soakCurPrio = PRIO_MAIN;
static uint64_t soakIsrFired[ISR_COUNT];

/* A pending JK reply the injector may deliver at a random moment. */
static uint8_t  soakJkBuf[600];
static uint16_t soakJkLen;
static int      soakInjectWild;     /* campaign 3: adversarial payloads */

static void soakDeliverJkBytes(const uint8_t *src, uint16_t srcLen, uint16_t reportSize)
{
    if (halUartRxDest != NULL && srcLen > 0u) {
        const uint16_t n = (srcLen > halUartRxCap) ? halUartRxCap : srcLen;
        memcpy(halUartRxDest, src, n);
    }
    HAL_UARTEx_RxEventCallback(&huart1, reportSize);
}

static void soakFireIsr(int which)
{
    const int prio = soakIsrPrio[which];
    if (prio >= soakCurPrio) { return; }        /* the NVIC would not preempt */
    const int saved = soakCurPrio;
    soakCurPrio = prio;
    soakIsrFired[which]++;
    soakIsrEvents++;

    switch (which) {
    case ISR_CAN1_RX: {
        /* Only deliver Activ when the modelled SafeState node is asserting, or
           the contactor can never leave the open state and the pull-in check
           gets no coverage at all. */
        const uint32_t id = soakInjectWild ? (1u + soakRndBelow(4u))
                                           : ((world.feedActiv && (soakRnd() & 1u)) ? 1u : 3u);
        halQueueCanRx(&hcan1, id, NULL, 8u);
        soakTraceAdd(soakIsrName[which], id, 0u);
        HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
        break;
    }
    case ISR_CAN2_RX: {
        uint32_t id;
        uint8_t  payload[8] = {0};
        if (soakInjectWild && (soakRnd() & 3u) == 0u) {
            id  = 208u + soakRndBelow(80u);      /* includes NODE and out-of-range */
            payload[0] = (uint8_t)soakRnd();
        } else {
            const uint8_t m = (uint8_t)(1u + soakRndBelow(7u));
            const uint8_t t = (uint8_t)(1u + soakRndBelow(9u));
            id  = soakThermId(m, t);
            payload[0] = world.thermBase;
        }
        halQueueCanRx(&hcan2, id, payload, 8u);
        soakTraceAdd(soakIsrName[which], id, payload[0]);
        HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
        break;
    }
    case ISR_ADC_DMA:
        if (soakAdcBuf != NULL) {
            /* The DMA is a bus master: it rewrites the buffer whenever it
               likes, including in the middle of ADC_Task's read-out. */
            soakAdcBuf[0] = (uint16_t)(world.adcTemp + (soakRnd() & 1u));
            soakAdcBuf[1] = (uint16_t)(world.adcCurr + (soakRnd() & 1u));
            soakAdcBuf[2] = (uint16_t)(world.adcVolt + (soakRnd() & 1u));
        }
        HAL_ADC_ConvCpltCallback(&hadc1);
        break;                      /* not traced: it fires every pass */
    case ISR_UART_TX:
        soakTraceAdd(soakIsrName[which], 0u, 0u);
        HAL_UART_TxCpltCallback(&huart1);
        break;
    case ISR_UART_RX: {
        uint16_t size = soakJkLen;
        if (soakInjectWild) {
            static const uint16_t nasty[] = { 0u, 1u, 19u, 20u, 123u, 339u, 511u, 512u };
            size = nasty[soakRndBelow((uint32_t)(sizeof nasty / sizeof nasty[0]))];
        }
        soakTraceAdd(soakIsrName[which], size, 0u);
        soakDeliverJkBytes(soakJkBuf, soakJkLen, size);
        break;
    }
    default: break;
    }
    soakCurPrio = saved;
}

static uint32_t soakInjectOdds;      /* 0 = off; else inject with p = 1/odds */

static void soakInjectHook(void)
{
    if (soakInjectOdds == 0u || soakCurPrio <= soakIsrPrio[ISR_CAN1_RX]) { return; }
    if (soakRndBelow(soakInjectOdds) != 0u) { return; }
    const int which = (int)soakRndBelow(ISR_COUNT);
    if (which == ISR_ADC_DMA && !world.adcRun) { return; }   /* a stopped DMA stays stopped */
    soakFireIsr(which);
}

/* ---- JK protocol frame builder ------------------------------------------ */
static void be16put(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void be32put(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

static uint16_t soakBuildJk(uint8_t *out, int withData, uint16_t packCv, uint8_t soc)
{
    uint16_t i;
    memset(out, 0, 256);
    out[0] = 0x4Eu; out[1] = 0x57u;
    i = 11u;                                   /* JKP_PAYLOAD_START */
    if (withData) {
        out[i++] = 0x79u; out[i++] = 63u;      /* 21 cells x 3 bytes */
        for (uint8_t c = 1u; c <= 21u; c++) {
            out[i++] = c; be16put(&out[i], (uint16_t)(3200u + c)); i = (uint16_t)(i + 2u);
        }
        out[i++] = 0x80u; be16put(&out[i], 30u);     i = (uint16_t)(i + 2u);
        out[i++] = 0x81u; be16put(&out[i], 28u);     i = (uint16_t)(i + 2u);
        out[i++] = 0x83u; be16put(&out[i], packCv);  i = (uint16_t)(i + 2u);
        out[i++] = 0x84u; be16put(&out[i], 10000u);  i = (uint16_t)(i + 2u);
        out[i++] = 0x85u; out[i++] = soc;
        out[i++] = 0x87u; be16put(&out[i], 42u);     i = (uint16_t)(i + 2u);
        out[i++] = 0x8Au; be16put(&out[i], 21u);     i = (uint16_t)(i + 2u);
        out[i++] = 0x8Bu; be16put(&out[i], 0u);      i = (uint16_t)(i + 2u);
        out[i++] = 0x8Cu; be16put(&out[i], 3u);      i = (uint16_t)(i + 2u);
        out[i++] = 0xAAu; be32put(&out[i], 100u);    i = (uint16_t)(i + 4u);
        out[i++] = 0xB9u; be32put(&out[i], 95u);     i = (uint16_t)(i + 4u);
        out[i++] = 0xC0u; out[i++] = 0x00u;
    }
    i = (uint16_t)(i + 4u);                    /* record number */
    out[i++] = 0x68u;                          /* end flag */
    i = (uint16_t)(i + 2u);                    /* CRC16 slot, disabled */
    const uint16_t len = (uint16_t)(i + 2u);
    be16put(&out[2], (uint16_t)(len - 2u));
    uint16_t sum = 0u;
    for (uint16_t k = 0u; k <= (uint16_t)(len - 5u); k++) { sum = (uint16_t)(sum + out[k]); }
    be16put(&out[len - 2u], sum);
    return len;
}

/* ---- contactor reference model, in 64-bit time -------------------------- */
static int      refActivPending, refActivSeen, refNodeSeen, refClosed;
static uint64_t refLastActivAbs, refClosedAtAbs;
static uint32_t refCcr;
static int      refModelEnabled;
static uint32_t refStateMismatches, refCcrMismatches;
static uint64_t refFirstMismatchAbs;
static uint64_t refQuietUntilAbs;      /* one report per episode, not per pass */
static uint32_t refEpisodes;
static uint64_t soakCloseCount;

static void soakRefContactor(uint64_t absNow)
{
    if (refActivPending) { refActivPending = 0; refLastActivAbs = absNow; refActivSeen = 1; }
    const int safe = refActivSeen && ((absNow - refLastActivAbs) < 300u);
    if (safe || !refNodeSeen) {
        if (refClosed) { refClosed = 0; refCcr = 0u; }
        return;
    }
    if (!refClosed) {
        refClosed = 1; refClosedAtAbs = absNow; refCcr = 999u;
        return;
    }
    refCcr = ((absNow - refClosedAtAbs) < 2000u) ? 999u : 500u;
}

/* ---- feeding the world, between passes ---------------------------------- */
static uint64_t feedThermNextAbs, feedSafeNextAbs;
static int      nodeEverDelivered, activEverDelivered;

static void soakFeedSafeState(void)
{
    if (world.feedNode) {
        nodeEverDelivered = 1; refNodeSeen = 1;
        if (world.canRxPath) {
            halQueueCanRx(&hcan1, 3u, NULL, 8u);
            HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
        } else {
            CONTACTOR_OnSafeStateFrame(3u);
        }
        soakIsrEvents++;
    }
    if (world.feedActiv) {
        activEverDelivered = 1; refActivPending = 1;
        if (world.canRxPath) {
            halQueueCanRx(&hcan1, 1u, NULL, 8u);
            HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
        } else {
            CONTACTOR_OnSafeStateFrame(1u);
        }
        soakIsrEvents++;
    }
}

static void soakFeedTherm(void)
{
    if (!world.feedTherm) { return; }
    for (uint8_t m = 1u; m <= 7u; m++) {
        for (uint8_t t = 1u; t <= 9u; t++) {
            uint8_t payload[8] = {0};
            payload[0] = world.thermBase;
            if (m == 1u && t == 1u) { payload[0] = world.thermProbe; }
            if (m == 7u && t == 9u) { payload[0] = world.thermHot; }
            const uint32_t id = soakThermId(m, t);
            if (world.canRxPath) {
                halQueueCanRx(&hcan2, id, payload, 8u);
            } else {
                THERM_OnFrame(id, payload[0]);
            }
            soakIsrEvents++;
        }
    }
    if (world.canRxPath) { HAL_CAN_RxFifo0MsgPendingCallback(&hcan2); }
}

/* ---- JK exchange driver -------------------------------------------------- */
static int      jkPhase;             /* 0 idle, 1 awaiting TC, 2 awaiting reply */
static uint64_t jkPhaseAtAbs;
static uint64_t jkLastTxSeen;

static void soakDriveJk(void)
{
    if (jkPhase == 0) {
        if (halUartTxCount != jkLastTxSeen) {
            jkLastTxSeen = halUartTxCount;
            jkPhase = 1; jkPhaseAtAbs = soakAbsMs;
            /* request byte 8 carries the command; 0x01 = activate */
            soakJkLen = soakBuildJk(soakJkBuf, (halUartTx[8] == 0x01u) ? 0 : 1,
                                    6850u, (uint8_t)(50u + soakRndBelow(40u)));
        }
        return;
    }
    if (jkPhase == 1 && (soakAbsMs - jkPhaseAtAbs) >= 2u) {
        jkPhase = 2; jkPhaseAtAbs = soakAbsMs;
        soakFireIsr(ISR_UART_TX);
        return;
    }
    if (jkPhase == 2 && (soakAbsMs - jkPhaseAtAbs) >= 2u) {
        jkPhase = 0;
        if (world.jkReply) {
            const int saved = soakCurPrio;
            soakCurPrio = soakIsrPrio[ISR_UART_RX];
            soakIsrFired[ISR_UART_RX]++; soakIsrEvents++;
            soakDeliverJkBytes(soakJkBuf, soakJkLen, soakJkLen);
            soakCurPrio = saved;
        }
    }
}

/* ---- error-handler inspection -------------------------------------------- */
static int soakErrActive(uint16_t code)
{
    for (uint8_t i = 0u; i < eh.activeErrorCount; i++) {
        if (eh.activeErrors[i].errorCode == code) { return 1; }
    }
    return 0;
}

static const char *soakErrName(uint16_t c)
{
    switch (c) {
    case 1u:  return "TEMP_HIGH";
    case 2u:  return "CAN2_TEMP_HIGH";
    case 3u:  return "CAN2_MODULE_SILENT";
    case 4u:  return "JK_COMMS_TIMEOUT";
    case 5u:  return "JK_FRAME_INVALID";
    case 6u:  return "PACK_VOLT_RANGE";
    case 7u:  return "PACK_CURRENT_HIGH";
    case 8u:  return "TEMP_SENSOR_FAULT";
    case 9u:  return "CAN1_TX_FAIL";
    case 10u: return "FATAL_INIT";
    case 11u: return "ADC_STALLED";
    default:  return "?";
    }
}

/* ---- per-pass invariants -------------------------------------------------- */
static uint32_t prevCcr;
static int      prevClosed;

static void soakCheckPass(void)
{
    const int closed = CONTACTOR_IsClosed();
    const uint32_t ccr = halCcr3();

    /* Safety-critical: the contactor may never close before the one-time NODE
       latch is set by a SafeState_NODE frame. */
    if (closed && !nodeEverDelivered) {
        SOAK_FAIL("contactor CLOSED with no SafeState_NODE frame ever delivered");
    }
    if (!closed && ccr != 0u && prevClosed) {
        SOAK_FAIL("contactor open but coil compare is %u, expected 0", ccr);
    }
    if (ccr != 0u && ccr != 500u && ccr != 999u) {
        SOAK_FAIL("coil compare %u is neither open(0), hold(500) nor pull-in(999)", ccr);
    }
    /* 100 %% pull-in on EVERY transition to closed, not only the first. */
    if (closed && !prevClosed) {
        soakCloseCount++;
        soakTraceAdd("contactor:CLOSE", ccr, 0u);
        if (ccr != 999u) {
            SOAK_FAIL("transition to closed did not command 100%% pull-in (compare %u)", ccr);
        }
    }
    if (!closed && prevClosed) { soakTraceAdd("contactor:OPEN", ccr, 0u); }

    if (refModelEnabled) {
        if (refClosed != closed) {
            refStateMismatches++;
            if (refFirstMismatchAbs == 0u) { refFirstMismatchAbs = soakAbsMs; }
            if (soakAbsMs >= refQuietUntilAbs) {
              refEpisodes++;
              refQuietUntilAbs = soakAbsMs + 10000u;
              SOAK_FAIL("contactor is %s but the 64-bit reference model says %s. "
                      "The firmware's 32-bit (now - lastActivMs) has wrapped: last "
                      "SafeState_Activ was %llu ms ago in real time, firmware sees "
                      "0x%08X - 0x%08X = %u ms. activSeen=%d nodeSeen=%d",
                      closed ? "CLOSED" : "OPEN", refClosed ? "CLOSED" : "OPEN",
                      (unsigned long long)(soakAbsMs - refLastActivAbs),
                      soakTick, (uint32_t)(soakTickBase + refLastActivAbs),
                      (uint32_t)(soakTick - (uint32_t)(soakTickBase + refLastActivAbs)),
                      refActivSeen, refNodeSeen);
            }
            /* Resynchronise so one episode is reported once, not every pass. */
            refClosed = closed;
            if (closed) { refClosedAtAbs = soakAbsMs; }
            refCcr = ccr;
        } else if (refCcr != ccr) {
            refCcrMismatches++;
            if (refFirstMismatchAbs == 0u) { refFirstMismatchAbs = soakAbsMs; }
            if (soakAbsMs >= refQuietUntilAbs) {
              refEpisodes++;
              refQuietUntilAbs = soakAbsMs + 10000u;
              SOAK_FAIL("coil compare is %u but the reference model says %u. The "
                        "contactor has been closed for %llu ms in real time (%.3f days); "
                        "the firmware's 32-bit (now - closedAtMs) reads %u ms, so the "
                        "2000 ms pull-in window has re-opened",
                        ccr, refCcr, (unsigned long long)(soakAbsMs - refClosedAtAbs),
                        (double)(soakAbsMs - refClosedAtAbs) / 86400000.0,
                        (uint32_t)(soakTick - (uint32_t)(soakTickBase + refClosedAtAbs)));
            }
            refCcr = ccr;
        }
    }

    if (halErrorHandlerCalls != 0u) {
        SOAK_FAIL("Error_Handler() reached %u times during the soak", halErrorHandlerCalls);
        halErrorHandlerCalls = 0u;
    }
    if (eh.isHalted) { SOAK_FAIL("error handler halted the node"); }
    if (eh.activeErrorCount > 16u) {
        SOAK_FAIL("activeErrorCount %u exceeds MAX_ACTIVE_ERRORS", eh.activeErrorCount);
    }

    prevClosed = closed;
    prevCcr = ccr;
}

/* ---- cadence / liveness sweep, run every few hundred passes -------------- */
typedef struct { uint32_t id; uint32_t period; uint32_t allow; } SoakFrameSpec;

static void soakCheckCadence(void)
{
    struct CAN_scheduledMsgList *s = CAN_App_Scheduler();
    for (uint8_t i = 0u; i < s->size; i++) {
        const uint32_t id = s->list[i].header.StdId;
        /* The node frame alternates 5000 ms heartbeat and 100..300 ms error
           mode; hold it to the slower of the two plus one period of slack. */
        const uint32_t period = (id == BMSMASTER_NODE_FRAME_ID) ? 5000u : s->list[i].periodMs;
        const uint32_t allow  = (period * 2u) + 1000u;
        HalTxStat *st = halTxSlot(id);
        if (st == NULL) { continue; }
        const uint32_t gap = soakTick - st->lastTick;
        if (st->count == 0u) {
            if (soakAbsMs > allow) {
                SOAK_FAIL("scheduled frame 0x%03X (%u ms) has NEVER transmitted", id, period);
                st->lastTick = soakTick;
            }
        } else if (gap > allow) {
            SOAK_FAIL("frame 0x%03X (%u ms period) silent for %u ms (allowance %u ms, "
                      "%llu sent so far)", id, period, gap, allow,
                      (unsigned long long)st->count);
            st->lastTick = soakTick;          /* report once per outage, not per sweep */
        }
        if (st->maxGap > allow) {
            SOAK_FAIL("frame 0x%03X (%u ms period) had a %u ms gap at tick 0x%08X",
                      id, period, st->maxGap, st->maxGapAtTick);
            st->maxGap = 0u;
        }
    }

    /* JK poll cadence: HAL_UART_Transmit_DMA is the request going out. */
    if (halUartTxCount > 0u) {
        const uint32_t gap = soakTick - halUartTxLastTick;
        if (gap > 3000u) {
            SOAK_FAIL("JK poll silent for %u ms (%llu polls so far, state machine wedged?)",
                      gap, (unsigned long long)halUartTxCount);
            halUartTxLastTick = soakTick;
        }
    }
}

/* ---- thermistor sweep liveness probe ------------------------------------- */
static uint64_t probeChangedAbs;
static int      probePending;
static int      probeEnabled;

static void soakProbeReset(void)
{
    probePending = 0;
    probeChangedAbs = soakAbsMs;
}

static void soakProbeStep(void)
{
    if (!probeEnabled) { return; }
    if (!world.feedTherm) { probePending = 0; probeChangedAbs = soakAbsMs; return; }
    if (probePending && (soakAbsMs - probeChangedAbs) >= 15000u) {
        const uint8_t got = THERM_Filtered(1u, 1u);
        if (got != world.thermProbe) {
            SOAK_FAIL("thermistor sweep stalled: fed module1/therm1 = %u for 15 s, "
                      "THERM_Filtered still reads %u", world.thermProbe, got);
        }
        probePending = 0;
    }
    if (!probePending && (soakAbsMs - probeChangedAbs) >= 20000u) {
        world.thermProbe = (world.thermProbe == 100u) ? 120u : 100u;
        probeChangedAbs = soakAbsMs;
        probePending = 1;
        soakTraceAdd("probe:therm-step", world.thermProbe, 0u);
    }
}

/* ---- one superloop pass -------------------------------------------------- */
static void soakOnePass(uint32_t stepMs)
{
    /* Feeds happen between passes: the reference model then sees exactly what
       CONTACTOR_Task sees on this pass. */
    if (soakAbsMs >= feedSafeNextAbs) { soakFeedSafeState(); feedSafeNextAbs += 100u; }
    if (soakAbsMs >= feedThermNextAbs) { soakFeedTherm(); feedThermNextAbs += 1000u; }

    /* The ADC DMA completes a scan every ~84 us on hardware: always ready. */
    if (world.adcRun) { soakFireIsr(ISR_ADC_DMA); }

    soakDriveJk();

    if (refModelEnabled) { soakRefContactor(soakAbsMs); }

    step(soakTick);                       /* the genuine app.c superloop body */

    soakCheckPass();
    soakProbeStep();

    soakPass++;
    soakSetTime(soakAbsMs + stepMs);
}

/* ---- campaign bookkeeping ------------------------------------------------ */
static uint64_t campaignStartAbs;
static uint64_t campaignStartPass;

static void soakStatsReset(const char *name)
{
    soakCampaign = name;
    campaignStartAbs = soakAbsMs;
    campaignStartPass = soakPass;
    for (uint32_t i = 0; i < HAL_TXSTAT_SLOTS; i++) {
        if (halTxStat[i].used) {
            halTxStat[i].lastTick = soakTick;
            halTxStat[i].maxGap = 0u;
            halTxStat[i].maxGapAtTick = 0u;
        }
    }
    halUartTxLastTick = soakTick;
    halUartTxMaxGap = 0u;
    printf("\n=== campaign: %s  (start tick 0x%08X, pass %llu)\n",
           name, soakTick, (unsigned long long)soakPass);
    fflush(stdout);
}

static void soakCadenceSweep(void)
{
    if ((soakAbsMs - campaignStartAbs) < 12000u) { return; }   /* let it warm up */
    soakCheckCadence();
}

static uint32_t soakNodeFrameCount(void)
{
    HalTxStat *s = halTxSlot(BMSMASTER_NODE_FRAME_ID);
    return (s == NULL) ? 0u : (uint32_t)s->count;
}

static void soakRunPasses(uint64_t count, uint32_t stepMs, uint32_t sweepEvery)
{
    for (uint64_t k = 0; k < count; k++) {
        soakOnePass(stepMs);
        if (sweepEvery != 0u && (soakPass % sweepEvery) == 0u) { soakCadenceSweep(); }
    }
}

/* ========================================================================== *
 * campaign 1: weeks of simulated time, across the 0xFFFFFFFF tick wrap
 * ========================================================================== */
/* Restart the whole system on a fresh tick base. initAll() re-runs every
   module's Init, so this is the same state the board reaches after a reset,
   with the tick starting just below the 0xFFFFFFFF rollover again. */
static void soakRestart(void)
{
    soakTickBase = (uint32_t)(0xFFFF0000u - (uint32_t)soakAbsMs);
    soakSetTime(soakAbsMs);
    memset(halTxUnit, 0, sizeof halTxUnit);
    halRxHead = halRxTail = 0u;
    memset(halRxQ, 0, sizeof halRxQ);
    refActivPending = refActivSeen = refNodeSeen = refClosed = 0;
    refLastActivAbs = refClosedAtAbs = 0u; refCcr = 0u;
    refStateMismatches = refCcrMismatches = refEpisodes = 0u;
    refFirstMismatchAbs = 0u; refQuietUntilAbs = 0u;
    nodeEverDelivered = activEverDelivered = 0;
    prevClosed = 0; prevCcr = 0u;
    jkPhase = 0; jkPhaseAtAbs = soakAbsMs; jkLastTxSeen = halUartTxCount;
    feedThermNextAbs = soakAbsMs; feedSafeNextAbs = soakAbsMs;
    soakWorldHealthy();
    initAll();
    soakAdcBuf = adcBuf;
    soakProbeReset();
}

/* profile 0: SafeState_Activ present for the first 5 s, then silent forever -
 *            exercises the 2^32 wrap of (now - lastActivMs).
 * profile 1: SafeState_Activ never seen at all - the contactor closes on the
 *            first NODE frame and stays closed, so the 2^32 wrap of
 *            (now - closedAtMs) is reached on its own instead of being masked
 *            by profile 0's spurious re-open.
 */
static void soakCampaign1(uint64_t durationMs, uint32_t coarseMs, int profile)
{
    soakStatsReset(profile ? "1b-weeks-no-activ-ever" : "1a-weeks-across-the-tick-wrap");
    probeEnabled = 1;
    soakProbeReset();

    /* Campaign-relative: ticks start at 0xFFFF0000, so the rollover is 0x10000 ms
       in, and any timestamp latched in the first seconds wraps 2^32 ms later. */
    const uint64_t WRAP_AT     = 0x10000ULL;
    const uint64_t SECOND_WRAP = 0x100000000ULL;

    uint64_t nextReport = campaignStartAbs;
    uint64_t nextErrCheck = campaignStartAbs;
    int activStopped = profile ? 1 : 0;
    world.feedActiv = profile ? 0 : 1;
    const uint64_t endAbs = campaignStartAbs + durationMs;

    while (soakAbsMs < endAbs) {
        /* Stop the Activ frames after 5 s so the contactor closes and stays
           closed: that is the state a stale timestamp can corrupt. */
        if (!activStopped && (soakAbsMs - campaignStartAbs) >= 5000u) {
            world.feedActiv = 0; activStopped = 1;
            soakTraceAdd("world:activ-off", 0u, 0u);
        }

        const uint64_t t = soakAbsMs - campaignStartAbs;
        const int fine =
            (t < 150000u) ||
            (t + 150000u > WRAP_AT && t < WRAP_AT + 200000u) ||
            (t + 200000u > SECOND_WRAP && t < SECOND_WRAP + 400000u) ||
            (t + 150000u > durationMs);

        soakOnePass(fine ? 1u : coarseMs);

        if ((soakPass & 0xFFu) == 0u) { soakCadenceSweep(); }

        if (soakAbsMs >= nextErrCheck) {
            nextErrCheck = soakAbsMs + 60000u;
            if ((soakAbsMs - campaignStartAbs) > 30000u) {
                for (uint8_t i = 0u; i < eh.activeErrorCount; i++) {
                    SOAK_FAIL("healthy vehicle but fault %u (%s) is active",
                              eh.activeErrors[i].errorCode,
                              soakErrName(eh.activeErrors[i].errorCode));
                }
                if (!ADC_Ready()) { SOAK_FAIL("ADC_Ready() false on a healthy vehicle"); }
                if (!JK_Valid())  { SOAK_FAIL("JK_Valid() false on a healthy vehicle"); }
            }
        }

        if (soakAbsMs >= nextReport) {
            nextReport = soakAbsMs + 86400000ULL;      /* one simulated day */
            printf("  day %6.2f  tick 0x%08X  passes %-12llu isr %-13llu "
                   "tx %-11llu closed=%d ccr=%u errs=%u\n",
                   (double)(soakAbsMs - campaignStartAbs) / 86400000.0, soakTick,
                   (unsigned long long)soakPass, (unsigned long long)soakIsrEvents,
                   (unsigned long long)halTxTotal, CONTACTOR_IsClosed(), halCcr3(),
                   eh.activeErrorCount);
            fflush(stdout);
        }
    }
    printf("  contactor state mismatches vs the 64-bit model: %u (in %u distinct "
           "episodes), coil-duty mismatches: %u, first at %.4f days after start\n",
           refStateMismatches, refEpisodes, refCcrMismatches,
           refFirstMismatchAbs ?
               (double)(refFirstMismatchAbs - campaignStartAbs) / 86400000.0 : 0.0);
    printf("  done: %.2f simulated days, %llu passes, tick rollover crossed %llu ms in\n",
           (double)(soakAbsMs - campaignStartAbs) / 86400000.0,
           (unsigned long long)(soakPass - campaignStartPass),
           (unsigned long long)WRAP_AT);
}

/* ========================================================================== *
 * campaign 2: randomised ISR interleaving, millions of passes
 * ========================================================================== */
static void soakCampaign2(uint64_t passes)
{
    soakStatsReset("2-randomised-isr-interleaving");
    probeEnabled = 0;
    refModelEnabled = 0;                  /* injected SafeState frames land mid-step */
    soakInjectOdds = 6u;
    soakInjectWild = 0;
    world.feedActiv = 1;
    uint64_t c2NextActivToggle = soakAbsMs;

    for (uint64_t k = 0; k < passes; k++) {
        /* Random subset of the ISR entry points before the task calls, on top
           of the hooks that fire inside them. */
        const uint32_t mask = soakRnd();
        for (int which = 0; which < ISR_COUNT; which++) {
            if (mask & (1u << which)) { soakFireIsr(which); }
        }
        /* Plausible but drifting analogue inputs. */
        if ((soakRnd() & 0x3Fu) == 0u) {
            world.adcVolt = (uint16_t)(2900u + soakRndBelow(200u));
            world.adcCurr = (uint16_t)(2000u + soakRndBelow(200u));
            world.adcTemp = (uint16_t)(1800u + soakRndBelow(900u));
        }
        /* Dwell long enough on each side of the 300 ms window that the
           contactor really opens and closes, thousands of times. */
        if (soakAbsMs >= c2NextActivToggle) {
            world.feedActiv = !world.feedActiv;
            c2NextActivToggle = soakAbsMs + 200u + soakRndBelow(2000u);
        }

        static const uint32_t steps[8] = { 0u, 0u, 0u, 1u, 1u, 1u, 2u, 4u };
        soakOnePass(steps[soakRndBelow(8u)]);

        if ((soakPass & 0x3FFu) == 0u) { soakCadenceSweep(); }
        if ((k & 0xFFFFFu) == 0u) {
            printf("  pass %llu  tick 0x%08X  isr %llu  tx %llu  errs %u\n",
                   (unsigned long long)soakPass, soakTick,
                   (unsigned long long)soakIsrEvents, (unsigned long long)halTxTotal,
                   eh.activeErrorCount);
            fflush(stdout);
        }
    }
    soakInjectOdds = 0u;
    printf("  campaign 2 done: %llu passes, %llu ISR events\n",
           (unsigned long long)(soakPass - campaignStartPass),
           (unsigned long long)soakIsrEvents);
}

/* ========================================================================== *
 * campaign 3: adversarial event storms
 * ========================================================================== */
static void soakCampaign3(uint64_t passes)
{
    soakStatsReset("3-adversarial-event-storms");
    refModelEnabled = 0;
    soakInjectOdds = 3u;
    soakInjectWild = 1;

    for (uint64_t k = 0; k < passes; k++) {
        const uint32_t r = soakRnd();

        /* Flood CAN2 far faster than 1 Hz, including NODE ids and ids outside
           the 211..279 window that the hardware filter lets through. */
        const uint32_t burst2 = soakRndBelow(64u);
        for (uint32_t i = 0; i < burst2; i++) {
            const uint32_t id = 208u + soakRndBelow(88u);
            uint8_t payload[8];
            for (uint32_t q = 0; q < 8u; q++) { payload[q] = (uint8_t)soakRnd(); }
            halQueueCanRx(&hcan2, id, payload, (uint8_t)(soakRndBelow(9u)));
            soakIsrEvents++;
        }
        if (burst2 != 0u) { soakFireIsr(ISR_CAN2_RX); HAL_CAN_RxFifo0MsgPendingCallback(&hcan2); }

        /* Flood CAN1 with SafeState frames, valid and not. */
        const uint32_t burst1 = soakRndBelow(32u);
        for (uint32_t i = 0; i < burst1; i++) {
            const uint32_t id = 1u + soakRndBelow(5u);
            if (id == 3u) { nodeEverDelivered = 1; }
            halQueueCanRx(&hcan1, id, NULL, 8u);
            soakIsrEvents++;
        }
        if (burst1 != 0u) { HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); }

        /* Spurious TX-complete with nothing in flight. */
        if ((r & 0x7u) == 0u) { soakFireIsr(ISR_UART_TX); soakIsrEvents++; }

        /* RX events at sizes the decoder must survive, at moments when no
           request is outstanding. 513 is handled by the separate --probe-jk-oob
           run: it exceeds the armed DMA length and is not reachable from the
           hardware, so letting it abort here would end the soak early. */
        if ((r & 0xFu) == 0u) {
            static const uint16_t sizes[] = { 0u, 1u, 2u, 19u, 20u, 21u, 123u, 338u,
                                              339u, 340u, 510u, 511u, 512u };
            const uint16_t sz = sizes[soakRndBelow((uint32_t)(sizeof sizes / sizeof sizes[0]))];
            soakJkLen = soakBuildJk(soakJkBuf, 1, 6850u, 55u);
            if ((r & 0x30u) == 0u) { soakJkBuf[soakRndBelow(120u)] ^= (uint8_t)soakRnd(); }
            soakTraceAdd("storm:jk-rx", sz, 0u);
            const int saved = soakCurPrio;
            soakCurPrio = soakIsrPrio[ISR_UART_RX];
            soakDeliverJkBytes(soakJkBuf, soakJkLen, sz);
            soakCurPrio = saved;
            soakIsrEvents++;
        }

        if ((r & 0x1Fu) == 0u) { world.jkReply = (int)(soakRnd() & 1u); }

        soakOnePass((soakRnd() & 3u) ? 1u : 0u);

        if ((soakPass & 0x3FFu) == 0u) { soakCadenceSweep(); }
        if ((k & 0x7FFFFu) == 0u) {
            printf("  pass %llu  tick 0x%08X  isr %llu  rxDropped %llu  errs %u\n",
                   (unsigned long long)soakPass, soakTick,
                   (unsigned long long)soakIsrEvents, (unsigned long long)halRxDropped,
                   eh.activeErrorCount);
            fflush(stdout);
        }
    }
    soakInjectOdds = 0u;
    soakInjectWild = 0;

    /* Settle back to a healthy world and prove nothing wedged. */
    soakWorldHealthy();
    world.feedActiv = 0;
    soakRunPasses(40000u, 1u, 1024u);
    if (halUartTxCount == jkLastTxSeen && jkPhase != 0) {
        SOAK_FAIL("JK state machine wedged after the storm (phase %d)", jkPhase);
    }
    soakRunPasses(40000u, 1u, 1024u);
    for (uint8_t i = 0u; i < eh.activeErrorCount; i++) {
        SOAK_FAIL("fault %u (%s) still active 80 s after the storm ended",
                  eh.activeErrors[i].errorCode, soakErrName(eh.activeErrors[i].errorCode));
    }
    printf("  campaign 3 done: %llu passes, %llu rx frames dropped by the 1024-deep fake FIFO\n",
           (unsigned long long)(soakPass - campaignStartPass),
           (unsigned long long)halRxDropped);
}

/* ========================================================================== *
 * campaign 4: fault churn - every fault must CLEAR when its condition clears
 * ========================================================================== */
typedef struct {
    const char *name;
    uint16_t    code;
    uint32_t    riseMs;      /* condition held this long: the fault must be up   */
    uint32_t    fallMs;      /* condition released this long: it must be down    */
} SoakFaultSpec;

static const SoakFaultSpec soakFaults[] = {
    { "board over-temperature",            1u,  2000u,  2000u },
    { "pack thermistor over-temperature",  2u, 14000u, 18000u },
    { "PCBCells module silent",            3u,  7000u,  4000u },
    { "JK link loss",                      4u,  7000u,  6000u },
    { "pack voltage out of range",         6u, 20000u,  2000u },
    { "pack current above 300 A",          7u, 20000u,  2000u },
    { "on-board NTC open",                 8u, 20000u,  2000u },
    { "ADC conversion stream stopped",     11u, 20000u,  2000u },
};
#define SOAK_FAULT_COUNT ((int)(sizeof soakFaults / sizeof soakFaults[0]))

static void soakApplyFault(int idx, int on)
{
    switch (soakFaults[idx].code) {
    case 1u:  world.adcTemp = on ? 3200u : 2100u; break;   /* >60 degC / ~26 degC   */
    case 2u:  world.thermHot = on ? 220u : 100u;  break;   /* raw 220 > 153 limit   */
    case 3u:  world.feedTherm = !on;              break;
    case 4u:  world.jkReply   = !on;              break;
    case 6u:  world.adcVolt = on ? 2700u : 3000u; break;   /* 61.7 V / 68.6 V       */
    case 7u:  world.adcCurr = on ? 4000u : 2108u; break;   /* 473 A / 0 A           */
    case 8u:  world.adcTemp = on ? 100u : 2100u;  break;   /* below the open guard  */
    case 11u: world.adcRun  = !on;                break;
    default:  break;
    }
    soakTraceAdd(on ? "fault:apply" : "fault:clear", soakFaults[idx].code, 0u);
}

static void soakChurnOne(int idx)
{
    const SoakFaultSpec *f = &soakFaults[idx];

    soakApplyFault(idx, 1);
    soakRunPasses(f->riseMs, 1u, 4096u);
    if (!soakErrActive(f->code)) {
        SOAK_FAIL("fault %u (%s) did NOT rise after %u ms of '%s'",
                  f->code, soakErrName(f->code), f->riseMs, f->name);
    }

    soakApplyFault(idx, 0);
    soakRunPasses(f->fallMs, 1u, 4096u);
    if (soakErrActive(f->code)) {
        SOAK_FAIL("fault %u (%s) LATCHED: still active %u ms after '%s' cleared",
                  f->code, soakErrName(f->code), f->fallMs, f->name);
    }
}

/* The scheduler entry for BMSMaster_NODE is removed and re-added on every
   0 <-> 1 active-error transition, and CAN_AddScheduledMsg stamps lastTick
   with HAL_GetTick(). A fault that flutters faster than the frame's period
   therefore keeps resetting its cadence. Measure it rather than assume. */
static void soakNodeStarvationSweep(void)
{
    static const uint32_t togglePeriods[] = { 40u, 120u, 250u, 600u, 1500u, 4000u };
    const uint32_t windowMs = 60000u;

    printf("  node-frame (0x%03X) rate under fault flutter, %u ms windows:\n",
           BMSMASTER_NODE_FRAME_ID, windowMs);

    for (uint32_t p = 0; p < sizeof togglePeriods / sizeof togglePeriods[0]; p++) {
        const uint32_t period = togglePeriods[p];
        soakWorldHealthy();
        world.feedActiv = 0;
        soakRunPasses(20000u, 1u, 0u);            /* settle, no active faults */

        const uint32_t before = soakNodeFrameCount();
        const uint64_t startAbs = soakAbsMs;
        uint64_t nextToggle = soakAbsMs;
        int out = 0;
        uint32_t toggles = 0u;
        while ((soakAbsMs - startAbs) < windowMs) {
            if (soakAbsMs >= nextToggle) {
                out = !out;
                world.adcVolt = out ? 2700u : 3000u;   /* out of / inside 63..87 V */
                nextToggle += period;
                toggles++;
            }
            soakOnePass(1u);
        }
        const uint32_t sent = soakNodeFrameCount() - before;
        {   /* why: is the frame still registered at all, and with what anchor? */
            struct CAN_scheduledMsgList *sl = CAN_App_Scheduler();
            int idx = -1;
            for (uint8_t q = 0u; q < sl->size; q++) {
                if (sl->list[q].header.StdId == BMSMASTER_NODE_FRAME_ID) { idx = (int)q; break; }
            }
            printf("      [scheduler size=%u nodeIdx=%d period=%u lastTick=0x%08X "
                   "now=0x%08X delta=%d errs=%u]\n",
                   sl->size, idx,
                   (idx >= 0) ? sl->list[idx].periodMs : 0u,
                   (idx >= 0) ? sl->list[idx].lastTick : 0u, soakTick,
                   (idx >= 0) ? (int)(soakTick - sl->list[idx].lastTick) : -1,
                   eh.activeErrorCount);
        }
        const uint32_t floorFrames = windowMs / 5000u;   /* the heartbeat rate */
        printf("    flutter every %5u ms (%3u transitions): node frame sent %4u times "
               "in %u ms%s\n", period, toggles, sent, windowMs,
               (sent < floorFrames) ? "   <-- BELOW THE 5 s HEARTBEAT FLOOR" : "");
        if (sent < floorFrames) {
            SOAK_FAIL("BMSMaster_NODE (0x%03X) transmitted only %u times in %u ms while a "
                      "fault flutters every %u ms; even the 5000 ms heartbeat demands %u. "
                      "The node frame is the vehicle's only view of BMS health.",
                      BMSMASTER_NODE_FRAME_ID, sent, windowMs, period, floorFrames);
        }
    }
    soakWorldHealthy();
    world.feedActiv = 0;
}

/* The flutter sweep showed the node frame going fully silent. Narrow it down:
   hold ONE fault steady (no flutter at all), then hold TWO. EH_reportEx only
   re-installs the scheduler entry when activeErrorCount == 1 && existingIndex
   == 0, and existingIndex is 0 for an UPDATE of the single active error as
   well as for its first insertion - so a module that re-reports every pass
   (ADC_Task does) re-stamps lastTick every pass. */
static void soakSteadyFaultSweep(void)
{
    printf("  node-frame (0x%03X) rate under STEADY faults, 60000 ms windows:\n",
           BMSMASTER_NODE_FRAME_ID);
    struct { const char *what; int a; int b; } cases[] = {
        { "no fault (heartbeat, 5000 ms)",            -1, -1 },
        { "one steady fault: pack voltage (code 6)",   4, -1 },
        { "one steady fault: pack current (code 7)",   5, -1 },
        { "one steady fault: NTC open (code 8)",       6, -1 },
        { "one steady fault: module silent (code 3)",  2, -1 },
        { "one steady fault: JK link loss (code 4)",   3, -1 },
        { "TWO steady faults: codes 6 and 7",          4,  5 },
    };
    for (uint32_t c = 0; c < sizeof cases / sizeof cases[0]; c++) {
        soakWorldHealthy();
        world.feedActiv = 0;
        for (uint16_t code = 1u; code <= 11u; code++) { EH_clear(&eh, code); }
        soakRunPasses(20000u, 1u, 0u);
        if (cases[c].a >= 0) { soakApplyFault(cases[c].a, 1); }
        if (cases[c].b >= 0) { soakApplyFault(cases[c].b, 1); }
        soakRunPasses(20000u, 1u, 0u);           /* let the fault establish */

        const uint32_t before = soakNodeFrameCount();
        soakRunPasses(60000u, 1u, 0u);
        const uint32_t sent = soakNodeFrameCount() - before;
        printf("    %-44s active=%u  node frame sent %4u in 60 s\n",
               cases[c].what, eh.activeErrorCount, sent);
        if (sent == 0u) {
            SOAK_FAIL("BMSMaster_NODE (0x%03X) transmitted ZERO times in 60 s with "
                      "'%s' (activeErrorCount=%u). The fault is recorded in the error "
                      "handler but the only frame that carries it off the board never "
                      "goes out.", BMSMASTER_NODE_FRAME_ID, cases[c].what,
                      eh.activeErrorCount);
        }
        if (cases[c].a >= 0) { soakApplyFault(cases[c].a, 0); }
        if (cases[c].b >= 0) { soakApplyFault(cases[c].b, 0); }
    }
    soakWorldHealthy();
    world.feedActiv = 0;
}

static void soakCampaign4(uint32_t rounds)
{
    soakStatsReset("4-fault-churn");
    refModelEnabled = 0;
    soakInjectOdds = 0u;
    soakWorldHealthy();
    world.feedActiv = 0;                   /* contactor closed for the duration */
    soakRunPasses(20000u, 1u, 0u);

    for (uint32_t r = 0; r < rounds; r++) {
        /* Every fault, in a shuffled order, one at a time. */
        int order[SOAK_FAULT_COUNT];
        for (int i = 0; i < SOAK_FAULT_COUNT; i++) { order[i] = i; }
        for (int i = SOAK_FAULT_COUNT - 1; i > 0; i--) {
            const int j = (int)soakRndBelow((uint32_t)(i + 1));
            const int t = order[i]; order[i] = order[j]; order[j] = t;
        }
        for (int i = 0; i < SOAK_FAULT_COUNT; i++) { soakChurnOne(order[i]); }

        /* Then two at once, to exercise the multiplexed error frame and the
           eviction path in EH_reportEx. */
        const int a = (int)soakRndBelow(SOAK_FAULT_COUNT);
        int b = (int)soakRndBelow(SOAK_FAULT_COUNT);
        if (b == a) { b = (a + 1) % SOAK_FAULT_COUNT; }
        soakApplyFault(a, 1); soakApplyFault(b, 1);
        soakRunPasses(20000u, 1u, 4096u);
        soakApplyFault(a, 0); soakApplyFault(b, 0);
        soakRunPasses(24000u, 1u, 4096u);

        /* Nothing may still be latched once every condition is gone. */
        soakWorldHealthy();
        world.feedActiv = 0;
        soakRunPasses(25000u, 1u, 4096u);
        for (uint8_t i = 0u; i < eh.activeErrorCount; i++) {
            SOAK_FAIL("round %u: fault %u (%s) LATCHED - still active 25 s after every "
                      "condition returned to healthy", r, eh.activeErrors[i].errorCode,
                      soakErrName(eh.activeErrors[i].errorCode));
        }
        if (eh.activeErrorCount != 0u) {
            /* Do not let one latched fault mask the next round. */
            for (uint16_t c = 1u; c <= 11u; c++) { EH_clear(&eh, c); }
        }
        printf("  round %u/%u done at tick 0x%08X, pass %llu\n", r + 1u, rounds, soakTick,
               (unsigned long long)soakPass);
        fflush(stdout);
    }

    soakSteadyFaultSweep();
    soakNodeStarvationSweep();
    printf("  campaign 4 done: %llu passes\n",
           (unsigned long long)(soakPass - campaignStartPass));
}

/* ========================================================================== *
 * isolated probe: JK_OnRxEvent(513)
 * ========================================================================== *
 * The RX DMA is armed with JKP_RX_BUF_LEN (512), so a HAL size above that is
 * not reachable from the hardware - but the task list asks for it, and the
 * decoder has no length ceiling of its own. Run in its own process so that an
 * abort here does not end the soak.
 */
static void soakProbeJkOob(void)
{
    printf("probe: JK_OnRxEvent(513) against a 512-byte rxBuf\n");
    /* Drive the state machine into JK_RECEIVING with a request outstanding. */
    world.jkReply = 0;
    for (int i = 0; i < 4000; i++) { soakOnePass(1u); }
    while (jkPhase != 2) { soakOnePass(1u); }

    /* A buffer that satisfies magic + LENGTH + end flag for a claimed total of
       513 bytes. JKP_Validate then reads the checksum at buf[511] and buf[512];
       the second byte is one past the end of rxBuf. */
    static uint8_t evil[512];
    memset(evil, 0, sizeof evil);
    evil[0] = 0x4Eu; evil[1] = 0x57u;
    evil[2] = 0x01u; evil[3] = 0xFFu;       /* LENGTH 511 -> claimed len 513 */
    evil[513u - 5u] = 0x68u;                /* end flag at index 508 */
    printf("probe: delivering size=513 now (ASan should abort on the read of rxBuf[512])\n");
    fflush(stdout);
    soakDeliverJkBytes(evil, (uint16_t)sizeof evil, 513u);
    /* JK_OnRxEvent only latches the size; JKP_Validate runs in JK_Task. */
    for (int i = 0; i < 50; i++) { soakOnePass(1u); }
    printf("probe: returned without a sanitizer abort\n");
}

/* ========================================================================== *
 * main
 * ========================================================================== */
static void soakPrintTxTable(void)
{
    printf("\n--- CAN1 transmit totals over the whole soak ---\n");
    struct CAN_scheduledMsgList *s = CAN_App_Scheduler();
    for (uint8_t i = 0u; i < s->size; i++) {
        const uint32_t id = s->list[i].header.StdId;
        HalTxStat *st = halTxSlot(id);
        printf("  0x%03X  period %5u ms  sent %12llu  worst gap %8u ms\n",
               id, s->list[i].periodMs, st ? (unsigned long long)st->count : 0ull,
               st ? st->maxGap : 0u);
    }
    printf("  total frames enqueued: %llu   mailbox aborts: %u   rx dropped: %llu\n",
           (unsigned long long)halTxTotal, halCanAbortCount,
           (unsigned long long)halRxDropped);
    printf("  ISR events fired: %llu  (CAN1 %llu, CAN2 %llu, ADC %llu, UART-TC %llu, UART-RX %llu)\n",
           (unsigned long long)soakIsrEvents,
           (unsigned long long)soakIsrFired[ISR_CAN1_RX],
           (unsigned long long)soakIsrFired[ISR_CAN2_RX],
           (unsigned long long)soakIsrFired[ISR_ADC_DMA],
           (unsigned long long)soakIsrFired[ISR_UART_TX],
           (unsigned long long)soakIsrFired[ISR_UART_RX]);
    printf("  JK polls issued: %llu   contactor closes observed: %llu\n",
           (unsigned long long)halUartTxCount, (unsigned long long)soakCloseCount);
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);

    uint64_t seed = 0x5EEDB115C0FFEEULL;      /* fixed by default, printed always */
    double   days = 62.0;
    double   daysB = 50.2;
    uint32_t coarseMs = 5u;
    uint64_t c2Passes = 6000000ULL;
    uint64_t c3Passes = 1500000ULL;
    uint32_t c4Rounds = 3u;
    int      probeOob = 0;

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--seed=", 7) == 0)        { seed = strtoull(argv[i] + 7, NULL, 0); }
        else if (strncmp(argv[i], "--days=", 7) == 0)   { days = atof(argv[i] + 7); }
        else if (strncmp(argv[i], "--daysB=", 8) == 0)  { daysB = atof(argv[i] + 8); }
        else if (strncmp(argv[i], "--coarse=", 9) == 0) { coarseMs = (uint32_t)atoi(argv[i] + 9); }
        else if (strncmp(argv[i], "--c2=", 5) == 0)     { c2Passes = strtoull(argv[i] + 5, NULL, 0); }
        else if (strncmp(argv[i], "--c3=", 5) == 0)     { c3Passes = strtoull(argv[i] + 5, NULL, 0); }
        else if (strncmp(argv[i], "--c4=", 5) == 0)     { c4Rounds = (uint32_t)atoi(argv[i] + 5); }
        else if (strcmp(argv[i], "--probe-jk-oob") == 0){ probeOob = 1; }
        else { printf("unknown argument %s\n", argv[i]); return 2; }
    }
    if (coarseMs == 0u) { coarseMs = 1u; }
    soakSeed = seed;
    soakRngState = seed ? seed : 1u;
    if (__sanitizer_set_death_callback) { __sanitizer_set_death_callback(soakOnDeath); }

    printf("BMS Master whole-system soak\n");
    printf("  seed           = %llu (0x%llX)\n", (unsigned long long)seed,
           (unsigned long long)seed);
    printf("  campaign 1a    = %.2f simulated days (Activ for 5 s then silent), "
           "coarse step %u ms\n", days, coarseMs);
    printf("  campaign 1b    = %.2f simulated days (Activ never seen)\n", daysB);
    printf("  campaign 2     = %llu passes\n", (unsigned long long)c2Passes);
    printf("  campaign 3     = %llu passes\n", (unsigned long long)c3Passes);
    printf("  campaign 4     = %u rounds\n", c4Rounds);
    printf("  start tick     = 0x%08X (wrap after %u ms)\n", 0xFFFF0000u, 0x10000u);

    halInit();
    soakTickBase = 0xFFFF0000u;
    soakSetTime(0);
    soakWorldHealthy();
    world.adcRun = 1;
    if (soakAdcBuf == NULL) { /* primed below, after ADC_Init binds the buffer */ }

    initAll();                            /* the genuine app.c initialisation */
    soakAdcBuf = adcBuf;
    soakInjectArmed = 1;
    refModelEnabled = 1;

    if (probeOob) { soakProbeJkOob(); return 0; }

    soakCampaign1((uint64_t)(days * 86400000.0), coarseMs, 0);
    soakRestart();
    soakCampaign1((uint64_t)(daysB * 86400000.0), coarseMs, 1);
    refModelEnabled = 0;
    soakCampaign2(c2Passes);
    soakCampaign3(c3Passes);
    soakCampaign4(c4Rounds);

    soakPrintTxTable();
    printf("\n================ SOAK SUMMARY ================\n");
    printf("  seed                : %llu\n", (unsigned long long)seed);
    printf("  simulated duration  : %.3f days (%llu ms)\n",
           (double)soakAbsMs / 86400000.0, (unsigned long long)soakAbsMs);
    printf("  superloop passes    : %llu\n", (unsigned long long)soakPass);
    printf("  ISR events fired    : %llu\n", (unsigned long long)soakIsrEvents);
    printf("  invariant violations: %u\n", soakViolations);
    printf("==============================================\n");
    return soakViolations ? 1 : 0;
}

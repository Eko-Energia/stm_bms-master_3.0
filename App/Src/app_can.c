#include "app_can.h"
#include "app_adc.h"
#include "app_contactor.h"
#include "app_jk.h"
#include "app_therm.h"
#include "bms_errors.h"
#include "CAN_DB.h"
#include "main.h"
#include <string.h>

/* CAN1 needs exactly two IDs, so one 16-bit list bank covers it. */
#define CAN1_FILTER_BANK        (0u)
#define SAFESTATE_ACTIV_ID      (1u)
#define SAFESTATE_NODE_ID       (3u)

/*
 * 211..279 is not cleanly maskable, so five 32-bit mask banks cover
 * 0x0D0..0x11F (208..287) and THERM_OnFrame bounds-checks the remainder in
 * software. CAN2 banks must start at the slave boundary or the filter lands in
 * a CAN1 bank and CAN2 accepts nothing.
 */
#define CAN2_FILTER_BANK_BASE   (14u)
#define CAN2_FILTER_MASK        (0x7F0u)
_Static_assert(CAN2_FILTER_BANK_BASE == CAN_SLAVE_START_FILTER_BANK,
               "CAN2 filters must start at the slave filter boundary");

/*
 * Legacy PCBCells boards send raw = (degC + 49) / 0.39216 and wrap above
 * 51 degC (stm_PCB-Cells 47a1036, fixed by its PR #13). Undo both so the
 * filter, the threshold and CAN1 see the (0.39216, 0) the databases specify.
 *
 * Set to 0 when the corrected firmware is flashed - a fixed board's byte would
 * otherwise be de-biased twice, and the two encodings cannot be told apart.
 */
#define THERM_LEGACY_DEBIAS  (1)
#define THERM_LEGACY_BIAS    (125u)   /* 49 degC / 0.39216, rounded */
#define THERM_LEGACY_LOWEST  (124u)   /* what a legacy board sends for 0 degC */

#if THERM_LEGACY_DEBIAS
/* 0..51 degC leaves a legacy board as 124..255; only a wrap lands in 0..123. */
static uint8_t debiasLegacyTherm(uint8_t raw)
{
    if (raw < THERM_LEGACY_LOWEST) {
        return (uint8_t)(raw + (256u - THERM_LEGACY_BIAS));  /* wrapped: add the 256 back */
    }
    /* 124 de-biases to -1, so floor it at zero rather than wrapping round. */
    return (raw > THERM_LEGACY_BIAS) ? (uint8_t)(raw - THERM_LEGACY_BIAS) : 0u;
}
#else
#define debiasLegacyTherm(raw) (raw)
#endif

static struct CAN_scheduledMsgList scheduler;
static CAN_HandleTypeDef *can1;
static EH_HandleTypeDef  *ehandler;
static uint32_t txFailId;
static bool     initOk;        /* frame currently reported as blocked, 0 = none */

/* Persistent frame structs: getData packs from these, so nothing is stale. */
static struct BMSMaster_MasterVoltCurrTemp_t frameMeas;
static struct BMSMaster_JK_Pack_t             frameJkPack;
static struct BMSMaster_JK_Temp_t             frameJkTemp;
static struct BMSMaster_JK_CycleStats_t       frameJkCycles;

static void getMeasurements(uint8_t *data, void *ctx)
{
    (void)ctx;
    BMSMaster_MasterVoltCurrTemp_init(&frameMeas);
    frameMeas.BMSMaster_MasterBatteryVoltage   = ADC_PackDecivolts();
    frameMeas.BMSMaster_MasterBatteryCurrent   = ADC_PackDeciamps();
    frameMeas.BMSMaste_MasterBatteryTemperatur = ADC_TempCenti();
    (void)BMSMaster_MasterVoltCurrTemp_pack(data, &frameMeas,
                                            BMSMASTER_MASTERVOLTCURRTEMP_LENGTH);
}

/* One callback for all nine thermistor frames; the context is the 1-based index.
   The frame is the transpose: thermistor y from all seven modules. Written
   directly - these are plain per-byte raw counts, not multiplexed signals. */
static void getTherm(uint8_t *data, void *ctx)
{
    const uint8_t therm = (uint8_t)(uintptr_t)ctx;
    for (uint8_t module = 1u; module <= THERM_MODULES; module++) {
        data[module - 1u] = THERM_Filtered(module, therm);
    }
}

static void getJkPack(uint8_t *data, void *ctx)
{
    (void)ctx;
    const JK_Data_t *d = JK_Data();
    BMSMaster_JK_Pack_init(&frameJkPack);
    frameJkPack.BMSMaster_JK_PackVoltage = d->packCentivolts;
    frameJkPack.BMSMaster_JK_PackCurrent = d->packCentiamps;
    frameJkPack.BMSMaster_JK_SOC         = d->soc;
    frameJkPack.BMSMaster_JK_SOH         = d->soh;
    frameJkPack.BMSMaster_JK_StatusFlags = d->statusFlags;
    frameJkPack.BMSMaster_JK_ModeFlags   = d->modeFlags;
    (void)BMSMaster_JK_Pack_pack(data, &frameJkPack, BMSMASTER_JK_PACK_LENGTH);
}

/* One callback for the six cell frames; the context is the 1-based first cell.
   Cells beyond JKP_CELLS_MAX stay zero, which is also the link-down state.
   Written directly - plain little-endian u16 mV, scale 1, no packing needed. */
static void getJkCells(uint8_t *data, void *ctx)
{
    const uint8_t first = (uint8_t)(uintptr_t)ctx;
    const JK_Data_t *d = JK_Data();
    for (uint8_t k = 0u; k < 4u; k++) {
        const uint8_t cell = (uint8_t)(first + k);
        if (cell > JKP_CELLS_MAX) { break; }
        const uint16_t mv = d->cellMillivolts[cell - 1u];
        data[k * 2u]        = (uint8_t)(mv & 0xFFu);
        data[(k * 2u) + 1u] = (uint8_t)((mv >> 8) & 0xFFu);
    }
}

static void getJkTemp(uint8_t *data, void *ctx)
{
    (void)ctx;
    const JK_Data_t *d = JK_Data();
    BMSMaster_JK_Temp_init(&frameJkTemp);
    frameJkTemp.BMSMaster_JK_MosTemp = d->mosTempC;
    frameJkTemp.BMSMaster_JK_BalTemp = d->balTempC;
    (void)BMSMaster_JK_Temp_pack(data, &frameJkTemp, BMSMASTER_JK_TEMP_LENGTH);
}

static void getJkCycles(uint8_t *data, void *ctx)
{
    (void)ctx;
    const JK_Data_t *d = JK_Data();
    BMSMaster_JK_CycleStats_init(&frameJkCycles);
    frameJkCycles.BMSMaster_JK_Cycles    = d->cycles;
    frameJkCycles.BMSMaster_JK_CellCount = d->cellCount;
    (void)BMSMaster_JK_CycleStats_pack(data, &frameJkCycles, BMSMASTER_JK_CYCLESTATS_LENGTH);
}

static void add(uint32_t id, uint8_t dlc, uint32_t periodMs,
                void (*fn)(uint8_t *, void *), void *ctx)
{
    struct CAN_scheduledMsg msg = {0};
    msg.header.StdId = id;
    msg.header.IDE   = CAN_ID_STD;
    msg.header.RTR   = CAN_RTR_DATA;
    msg.header.DLC   = dlc;
    msg.periodMs     = periodMs;
    msg.getData      = fn;
    msg.context       = ctx;
    if (CAN_AddScheduledMsg(&msg, &scheduler) != HAL_OK) {
        /* EH_init runs after this, so a report here would be dropped. Fail the
           init instead and let the caller escalate. */
        initOk = false;
    }
}

bool CAN_App_Init(CAN_HandleTypeDef *hcan1, CAN_HandleTypeDef *hcan2, EH_HandleTypeDef *eh)
{
    initOk = true;
    can1 = hcan1;
    ehandler = eh;
    txFailId = 0u;
    memset(&scheduler, 0, sizeof scheduler);

    /* LOW is normal operation. Written explicitly rather than relying on the
       generated MX_GPIO_Init reset state, so it survives regeneration. */
    HAL_GPIO_WritePin(nCAN1_Stby_GPIO_Port, nCAN1_Stby_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(nCAN2_Stby_GPIO_Port, nCAN2_Stby_Pin, GPIO_PIN_RESET);

    /* Filters before CAN_Init: NART is only writable before HAL_CAN_Start. */
    const uint16_t safeStateIds[4] = { SAFESTATE_ACTIV_ID, SAFESTATE_NODE_ID,
                                       SAFESTATE_ACTIV_ID, SAFESTATE_ACTIV_ID };
    initOk = initOk && (CAN_ConfigFilterList16(hcan1, CAN1_FILTER_BANK, safeStateIds) == HAL_OK);
    for (uint8_t k = 0u; k < 5u; k++) {
        initOk = initOk && (CAN_ConfigFilterMask32(hcan2, (uint8_t)(CAN2_FILTER_BANK_BASE + k),
                                          (uint16_t)(0x0D0u + (k * 0x010u)), CAN2_FILTER_MASK) == HAL_OK);
    }
    initOk = initOk && (CAN_Init(hcan1) == HAL_OK);
    initOk = initOk && (CAN_Init(hcan2) == HAL_OK);

    add(BMSMASTER_MASTERVOLTCURRTEMP_FRAME_ID, BMSMASTER_MASTERVOLTCURRTEMP_LENGTH,
        BMSMASTER_MASTERVOLTCURRTEMP_CYCLE_TIME_MS, getMeasurements, NULL);

    static const uint32_t thermIds[THERM_PER_MODULE] = {
        BMSMASTER_PCBSTHERM1TEMP_FRAME_ID, BMSMASTER_PCBSTHERM2TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM3TEMP_FRAME_ID, BMSMASTER_PCBSTHERM4TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM5TEMP_FRAME_ID, BMSMASTER_PCBSTHERM6TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM7TEMP_FRAME_ID, BMSMASTER_PCBSTHERM8TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM9TEMP_FRAME_ID
    };
    for (uint8_t t = 0u; t < THERM_PER_MODULE; t++) {
        add(thermIds[t], BMSMASTER_PCBSTHERM1TEMP_LENGTH, BMSMASTER_PCBSTHERM1TEMP_CYCLE_TIME_MS,
            getTherm, (void *)(uintptr_t)(t + 1u));
    }

    add(BMSMASTER_JK_PACK_FRAME_ID, BMSMASTER_JK_PACK_LENGTH, BMSMASTER_JK_PACK_CYCLE_TIME_MS,
        getJkPack, NULL);
    add(BMSMASTER_JK_CELLS_1_4_FRAME_ID,   BMSMASTER_JK_CELLS_1_4_LENGTH,   BMSMASTER_JK_CELLS_1_4_CYCLE_TIME_MS,   getJkCells, (void *)(uintptr_t)1u);
    add(BMSMASTER_JK_CELLS_5_8_FRAME_ID,   BMSMASTER_JK_CELLS_5_8_LENGTH,   BMSMASTER_JK_CELLS_5_8_CYCLE_TIME_MS,   getJkCells, (void *)(uintptr_t)5u);
    add(BMSMASTER_JK_CELLS_9_12_FRAME_ID,  BMSMASTER_JK_CELLS_9_12_LENGTH,  BMSMASTER_JK_CELLS_9_12_CYCLE_TIME_MS,  getJkCells, (void *)(uintptr_t)9u);
    add(BMSMASTER_JK_CELLS_13_16_FRAME_ID, BMSMASTER_JK_CELLS_13_16_LENGTH, BMSMASTER_JK_CELLS_13_16_CYCLE_TIME_MS, getJkCells, (void *)(uintptr_t)13u);
    add(BMSMASTER_JK_CELLS_17_20_FRAME_ID, BMSMASTER_JK_CELLS_17_20_LENGTH, BMSMASTER_JK_CELLS_17_20_CYCLE_TIME_MS, getJkCells, (void *)(uintptr_t)17u);
    add(BMSMASTER_JK_CELLS_21_FRAME_ID,    BMSMASTER_JK_CELLS_21_LENGTH,    BMSMASTER_JK_CELLS_21_CYCLE_TIME_MS,    getJkCells, (void *)(uintptr_t)21u);
    add(BMSMASTER_JK_TEMP_FRAME_ID,        BMSMASTER_JK_TEMP_LENGTH,        BMSMASTER_JK_TEMP_CYCLE_TIME_MS,        getJkTemp, NULL);
    add(BMSMASTER_JK_CYCLESTATS_FRAME_ID,  BMSMASTER_JK_CYCLESTATS_LENGTH,  BMSMASTER_JK_CYCLESTATS_CYCLE_TIME_MS,  getJkCycles, NULL);

    /* END has no signals defined; honour the cycle time with 8 zero bytes.
       A NULL getData leaves the scheduler's zeroed buffer untouched. */
    add(BMSMASTER_END_FRAME_ID, BMSMASTER_END_LENGTH, BMSMASTER_END_CYCLE_TIME_MS, NULL, NULL);

    return initOk;
}

/*
 * txFailCount is the driver's own count of missed PERIODS, reset on every
 * successful enqueue, so reaching CAN_TX_FAIL_LIMIT means "this frame cannot
 * get out" - the threshold the driver itself aborts on - not "a mailbox was
 * busy". Ordinary burst contention never reaches it. Spec 10: code 9 is a
 * warning carrying the frame ID as u16.
 */
static void checkTxBlocked(void)
{
    if (ehandler == NULL || CAN_TX_FAIL_LIMIT == 0u) { return; }

    uint32_t blockedId = 0u;
    for (uint8_t i = 0u; i < scheduler.size; i++) {
        if (scheduler.list[i].txFailCount >= CAN_TX_FAIL_LIMIT) {
            blockedId = scheduler.list[i].header.StdId;   /* every CAN1 frame is standard */
            break;
        }
    }

    if (blockedId == txFailId) { return; }                /* no edge: nothing to do */
    if (blockedId != 0u) {
        const uint8_t blob[5] = { (uint8_t)(blockedId & 0xFFu),
                                  (uint8_t)((blockedId >> 8) & 0xFFu), 0u, 0u, 0u };
        EH_reportEx(ehandler, BMS_ERR_CAN1_TX_FAIL, ERROR_SEVERITY_WARNING, blob, 2u);
    } else {
        EH_clear(ehandler, BMS_ERR_CAN1_TX_FAIL);
    }
    txFailId = blockedId;
}

void CAN_App_Task(void)
{
    CAN_HandleScheduled(can1, &scheduler);
    checkTxBlocked();
}

void CAN_App_OnRx1(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[CAN_MAX_DLC];
    while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK) {
        if (header.IDE == CAN_ID_STD) {
            CONTACTOR_OnSafeStateFrame(header.StdId);
        }
    }
}

void CAN_App_OnRx2(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[CAN_MAX_DLC];
    while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK) {
        if (header.IDE == CAN_ID_STD && header.DLC >= 1u) {
            THERM_OnFrame(header.StdId, debiasLegacyTherm(data[0]));
        }
    }
}

struct CAN_scheduledMsgList *CAN_App_Scheduler(void) { return &scheduler; }

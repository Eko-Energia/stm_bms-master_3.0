#include "app_therm.h"
#include "bms_errors.h"
#include "CAN2_DB.h"
#include <string.h>

#define THERM_WINDOW      (10u)   /* 10 s at 1 Hz */
#define THERM_MISS_LIMIT  (3u)    /* consecutive empty periods before reporting */
#define THERM_ID_FIRST    (211u)
#define THERM_ID_LAST     (279u)

/* Even modules number DOWNWARD, so a plain id-minus-base decode silently
   mismaps 2, 4 and 6. The asserts below break the build on a renumber. */
#define MODULE_OF(id)   ((uint8_t)(((id) / 10u) - 20u))
#define OFFSET_OF(id) ((uint8_t)((id) % 10u))
#define THERM_OF(id)  ((MODULE_OF(id) & 1u) ? OFFSET_OF(id) : (uint8_t)(10u - OFFSET_OF(id)))

_Static_assert(MODULE_OF(PCBCELLS1_THERM1_FRAME_ID) == 1u, "module map drifted");
_Static_assert(THERM_OF(PCBCELLS1_THERM1_FRAME_ID) == 1u, "odd module counts up");
_Static_assert(THERM_OF(PCBCELLS1_THERM9_FRAME_ID) == 9u, "odd module counts up");
_Static_assert(MODULE_OF(PCBCELLS2_THERM9_FRAME_ID) == 2u, "module map drifted");
_Static_assert(THERM_OF(PCBCELLS2_THERM9_FRAME_ID) == 9u, "even module counts down");
_Static_assert(THERM_OF(PCBCELLS2_THERM1_FRAME_ID) == 1u, "even module counts down");
_Static_assert(THERM_OF(PCBCELLS6_THERM9_FRAME_ID) == 9u, "even module counts down");
_Static_assert(THERM_OF(PCBCELLS7_THERM9_FRAME_ID) == 9u, "odd module counts up");
_Static_assert(MODULE_OF(PCBCELLS4_THERM9_FRAME_ID) == 4u, "module map drifted");
_Static_assert(THERM_OF(PCBCELLS4_THERM9_FRAME_ID) == 9u, "even module counts down");

/* Written by the CAN2 ISR, read by THERM_Task. Byte stores are atomic on M3,
   so no critical section is needed anywhere in this module. */
static volatile uint8_t thermLatest[THERM_MODULES][THERM_PER_MODULE];
static volatile uint8_t thermSeen[THERM_MODULES][THERM_PER_MODULE];

static uint8_t thermWindow[THERM_MODULES][THERM_PER_MODULE][THERM_WINDOW];
static uint8_t thermFiltered[THERM_MODULES][THERM_PER_MODULE];
static uint8_t thermMiss[THERM_MODULES][THERM_PER_MODULE];
static uint8_t thermIdx;
static uint8_t thermFill;
static uint8_t thermMax;
static uint8_t thermMaxModule;   /* 1..7, where thermMax was read */
static uint8_t thermMaxTherm;    /* 1..9 */
static EH_HandleTypeDef *ehandler;

static uint8_t trimmedMean(const uint8_t *samples, uint8_t fill)
{
    uint16_t sum = 0u;              /* 10 * 255 = 2550, fits uint16 */
    uint8_t lo = 0xFFu, hi = 0u;

    for (uint8_t i = 0u; i < fill; i++) {
        const uint8_t s = samples[i];
        sum = (uint16_t)(sum + s);
        if (s < lo) { lo = s; }
        if (s > hi) { hi = s; }
    }
    if (fill < 3u) {
        return (uint8_t)((sum + (fill / 2u)) / fill);
    }
    sum = (uint16_t)(sum - lo - hi);
    const uint8_t n = (uint8_t)(fill - 2u);
    return (uint8_t)((sum + (n / 2u)) / n);
}

void THERM_Init(EH_HandleTypeDef *eh)
{
    ehandler = eh;
    thermIdx = 0u;
    thermFill = 0u;
    thermMax = 0u;
    thermMaxModule = 1u;
    thermMaxTherm = 1u;
    memset((void *)thermLatest, 0, sizeof thermLatest);
    memset((void *)thermSeen, 0, sizeof thermSeen);
    memset(thermWindow, 0, sizeof thermWindow);
    memset(thermFiltered, 0, sizeof thermFiltered);
    memset(thermMiss, 0, sizeof thermMiss);
}

void THERM_OnFrame(uint32_t stdId, uint8_t raw)
{
    if (stdId < THERM_ID_FIRST || stdId > THERM_ID_LAST) { return; }
    const uint8_t offset = OFFSET_OF(stdId);
    if (offset == 0u) { return; }               /* PCBCells<x>_NODE */

    const uint8_t module = MODULE_OF(stdId);
    const uint8_t therm = THERM_OF(stdId);      /* the macro the asserts guard */

    thermLatest[module - 1u][therm - 1u] = raw;
    thermSeen[module - 1u][therm - 1u] = 1u;
}

void THERM_Task(void)
{
    uint8_t silentModules = 0u;

    if (thermFill < THERM_WINDOW) { thermFill++; }

    for (uint8_t p = 0u; p < THERM_MODULES; p++) {
        for (uint8_t t = 0u; t < THERM_PER_MODULE; t++) {
            /*
             * Clear the flag BEFORE reading the value: a frame landing
             * mid-sweep is still used, and no sample is lost. */
            const uint8_t seen = thermSeen[p][t];
            thermSeen[p][t] = 0u;
            const uint8_t latest = thermLatest[p][t];

            if (seen != 0u) {
                thermWindow[p][t][thermIdx] = latest;
                thermMiss[p][t] = 0u;
            } else {
                /* Re-push the previous slot so all 63 windows stay in lockstep
                   on one shared index. */
                const uint8_t prev = (thermIdx == 0u) ? (uint8_t)(THERM_WINDOW - 1u)
                                                      : (uint8_t)(thermIdx - 1u);
                thermWindow[p][t][thermIdx] = thermWindow[p][t][prev];
                if (thermMiss[p][t] < THERM_MISS_LIMIT) { thermMiss[p][t]++; }
            }

            thermFiltered[p][t] = trimmedMean(thermWindow[p][t], thermFill);
            if (thermMiss[p][t] >= THERM_MISS_LIMIT) { silentModules |= (uint8_t)(1u << p); }
        }
    }

    thermIdx = (uint8_t)((thermIdx + 1u) % THERM_WINDOW);

    /* Keep the argmax: CAN2_TEMP_HIGH must say which thermistor is hot (spec 6.4). */
    uint8_t hottest = 0u, hotModule = 1u, hotTherm = 1u;
    for (uint8_t p = 0u; p < THERM_MODULES; p++) {
        for (uint8_t t = 0u; t < THERM_PER_MODULE; t++) {
            if (thermFiltered[p][t] > hottest) {
                hottest = thermFiltered[p][t];
                hotModule = (uint8_t)(p + 1u);
                hotTherm = (uint8_t)(t + 1u);
            }
        }
    }
    thermMax = hottest;
    thermMaxModule = hotModule;
    thermMaxTherm = hotTherm;

    if (ehandler != NULL) {
        if (silentModules != 0u) {
            const uint8_t blob[5] = { silentModules, 0u, 0u, 0u, 0u };
            EH_reportEx(ehandler, BMS_ERR_CAN2_MODULE_SILENT, ERROR_SEVERITY_ERROR, blob, 1u);
        } else {
            EH_clear(ehandler, BMS_ERR_CAN2_MODULE_SILENT);
        }
    }
}

uint8_t THERM_Filtered(uint8_t module, uint8_t therm)
{
    if (module == 0u || module > THERM_MODULES || therm == 0u || therm > THERM_PER_MODULE) {
        return 0u;
    }
    return thermFiltered[module - 1u][therm - 1u];
}

uint8_t THERM_MaxRaw(void)    { return thermMax; }
uint8_t THERM_MaxModule(void) { return thermMaxModule; }
uint8_t THERM_MaxTherm(void)  { return thermMaxTherm; }

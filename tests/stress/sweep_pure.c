/*
 * sweep_pure.c - exhaustive numeric-domain sweeps of the BMS Master pure
 * functions. Every domain small enough to enumerate is enumerated in full;
 * the sweeps assert invariants rather than merely watching for crashes.
 *
 * The modules under test are #included as translation units so that their
 * file-static helpers (countToCenti, decodeTemp, trimmedMean, MODULE_OF...)
 * are reachable without editing App/. Two of them define a static
 * trimmedMean and a static ehandler, so those names are renamed per include.
 *
 * Build/run: ./run_sweep.sh   (see that script for the sanitizer flags)
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "error_handler.h"
#include "CAN_DB.h"
#include "CAN2_DB.h"

#define trimmedMean adcTrimmedMean
#define ehandler    adcEhandler
#include "app_adc.c"
#undef trimmedMean
#undef ehandler

#define trimmedMean thermTrimmedMean
#define ehandler    thermEhandler
#include "app_therm.c"
#undef trimmedMean
#undef ehandler

#include "jk_protocol.c"

/* ------------------------------------------------------------------ */
/* violation bookkeeping                                              */
/* ------------------------------------------------------------------ */
#define MAX_SITES 64
#define SHOW_PER_SITE 4

static struct { const char *name; unsigned long hits; } gSite[MAX_SITES];
static int gSiteN;
static unsigned long gFails;
static unsigned long gChecks;

static void fail(const char *site, const char *fmt, ...)
{
    int i;
    for (i = 0; i < gSiteN; i++) {
        if (strcmp(gSite[i].name, site) == 0) { break; }
    }
    if (i == gSiteN) {
        if (gSiteN >= MAX_SITES) { gFails++; return; }
        gSite[gSiteN].name = site;
        gSite[gSiteN].hits = 0u;
        gSiteN++;
    }
    gSite[i].hits++;
    gFails++;
    if (gSite[i].hits <= SHOW_PER_SITE) {
        va_list ap;
        va_start(ap, fmt);
        printf("  VIOLATION [%s] ", site);
        vprintf(fmt, ap);
        printf("\n");
        va_end(ap);
    } else if (gSite[i].hits == SHOW_PER_SITE + 1u) {
        printf("  VIOLATION [%s] ... further occurrences counted only\n", site);
    }
}

#define CK(cond, site, ...) do { gChecks++; if (!(cond)) { fail(site, __VA_ARGS__); } } while (0)

static void section(const char *name) { printf("\n== %s\n", name); }

/* ------------------------------------------------------------------ */
/* 1. countToCenti: all 4096 12-bit ADC counts                        */
/* ------------------------------------------------------------------ */

/* Independent double-precision reference for the same interpolation. */
static double refCenti(uint16_t count, int *segOut)
{
    *segOut = -1;
    if (count >= calibNtcCount[100]) { return 10000.0; }
    if (count <= calibNtcCount[0])   { return 0.0; }
    for (int i = 0; i < 100; i++) {
        if (count < calibNtcCount[i + 1]) {
            const double lo = (double)calibNtcCount[i];
            const double hi = (double)calibNtcCount[i + 1];
            *segOut = i;
            return ((double)i * 100.0) + (((double)count - lo) * 100.0 / (hi - lo));
        }
    }
    return 10000.0;
}

static void sweepCountToCenti(void)
{
    section("1. countToCenti - all 4096 counts");

    /* The interpolation divides by the span, so a non-increasing table would
       be a divide-by-zero (equal) or a wild span (decreasing). */
    uint16_t minSpan = 0xFFFFu, maxSpan = 0u;
    for (int i = 0; i < 100; i++) {
        CK(calibNtcCount[i] < calibNtcCount[i + 1], "ntc_table_strictly_increasing",
           "calibNtcCount[%d]=%u is not < [%d]=%u (span would be %d)",
           i, calibNtcCount[i], i + 1, calibNtcCount[i + 1],
           (int)calibNtcCount[i + 1] - (int)calibNtcCount[i]);
        if (calibNtcCount[i + 1] > calibNtcCount[i]) {
            const uint16_t s = (uint16_t)(calibNtcCount[i + 1] - calibNtcCount[i]);
            if (s < minSpan) { minSpan = s; }
            if (s > maxSpan) { maxSpan = s; }
        }
    }

    uint16_t prev = 0u;
    int maxJump = 0;
    uint32_t jumpAt = 0u;
    int worstErr = 0;
    for (uint32_t c = 0u; c < 4096u; c++) {
        const uint16_t v = countToCenti((uint16_t)c);

        CK(v <= TEMP_MAX_CENTI, "centi_in_range",
           "count=%u -> %u centi, expected 0..%u", c, v, (unsigned)TEMP_MAX_CENTI);

        if (c > 0u) {
            CK(v >= prev, "centi_monotone_non_decreasing",
               "count=%u -> %u centi, but count=%u -> %u centi", c, v, c - 1u, prev);
            const int jump = (int)v - (int)prev;
            if (jump > maxJump) { maxJump = jump; jumpAt = c; }
            /* One table step is 100 centi; a bigger step means the segment
               search or the interpolation skipped a degree. */
            CK(jump <= 100, "centi_continuity_one_table_step",
               "count=%u..%u jumps %d centi (>100)", c - 1u, c, jump);
        }

        int seg = -1;
        const double ref = refCenti((uint16_t)c, &seg);
        /* calibNtcCount[] is indexed [seg] and [seg+1], so seg must stay in
           0..99 for the read to be inside the 101-entry table. */
        if (seg >= 0) {
            CK(seg >= 0 && seg <= 99, "ntc_index_in_range",
               "count=%u selects table index %d (and %d), table is 0..100", c, seg, seg + 1);
        }
        const int err = (int)v - (int)(ref + 0.5);
        if (err > worstErr || -err > worstErr) { worstErr = (err < 0) ? -err : err; }
        CK(err <= 1 && err >= -1, "centi_matches_reference",
           "count=%u -> %u centi, reference %.3f", c, v, ref);

        prev = v;
    }
    printf("  4096/4096 counts swept. table spans %u..%u counts/degC, "
           "max adjacent jump %d centi at count %u, max deviation from the "
           "double reference %d centi\n", minSpan, maxSpan, maxJump, jumpAt, worstErr);
    printf("  countToCenti(0)=%u  (%u)=%u  (%u)=%u  (%u)=%u  (4095)=%u\n",
           countToCenti(0), calibNtcCount[0], countToCenti(calibNtcCount[0]),
           calibNtcCount[50], countToCenti(calibNtcCount[50]),
           calibNtcCount[100], countToCenti(calibNtcCount[100]), countToCenti(4095));
}

/* ------------------------------------------------------------------ */
/* 2. decodeTemp: all 65536 raw values                                */
/* ------------------------------------------------------------------ */
static void sweepDecodeTemp(void)
{
    section("2. decodeTemp - all 65536 raws");

    /* decodeTemp now gates on the vendor range instead of returning a value:
       0..140 is accepted (0..100 positive degC, 101..140 negative with
       101 == -1), everything else is rejected so the frame is dropped. */
    unsigned long accepted = 0u, rejected = 0u;
    for (uint32_t r = 0u; r <= 0xFFFFu; r++) {
        int8_t out = 0;
        const bool ok = decodeTemp((uint16_t)r, &out);
        const bool shouldAccept = (r <= 140u);

        gChecks++;
        if (ok != shouldAccept) {
            fail("decode_gate_matches_vendor_range",
                 "raw=%u: accepted=%d, vendor range says %d", r, (int)ok, (int)shouldAccept);
            continue;
        }

        if (ok) {
            accepted++;
            const long rule = (r > 100u) ? -(long)(r - 100u) : (long)r;
            CK(out == (int8_t)rule, "decode_value_matches_rule",
               "raw=%u -> %d, rule says %ld", r, (int)out, rule);
            CK(rule >= -128 && rule <= 127, "accepted_value_fits_int8",
               "raw=%u: rule value %ld outside int8_t", r, rule);
            if (r > 100u) {
                CK(out < 0, "above_one_hundred_is_sub_zero", "raw=%u -> %d", r, (int)out);
            }
        } else {
            rejected++;
        }
    }
    printf("    accepted %lu raws (0..140), rejected %lu - no value can now exceed int8_t\n",
           accepted, rejected);
}

/* ------------------------------------------------------------------ */
/* 3. MODULE_OF / OFFSET_OF / THERM_OF over all 2048 standard IDs     */
/* ------------------------------------------------------------------ */
static int accepted(uint32_t id)
{
    return (id >= THERM_ID_FIRST && id <= THERM_ID_LAST && (id % 10u) != 0u);
}

static void sweepIdMacros(void)
{
    section("3. MODULE_OF / OFFSET_OF / THERM_OF - all 2048 standard IDs");

    int acc = 0, wouldEscape = 0;
    uint32_t firstEscapeHi = 0u, firstEscapeLo = 0u;
    for (uint32_t id = 0u; id < 2048u; id++) {
        const uint8_t m = MODULE_OF(id);
        const uint8_t off = OFFSET_OF(id);
        const uint8_t t = THERM_OF(id);

        if (accepted(id)) {
            acc++;
            CK(m >= 1u && m <= THERM_MODULES, "module_1_based_in_range",
               "id=%u -> module %u, expected 1..%u", id, m, (unsigned)THERM_MODULES);
            CK(t >= 1u && t <= THERM_PER_MODULE, "therm_1_based_in_range",
               "id=%u -> therm %u, expected 1..%u", id, t, (unsigned)THERM_PER_MODULE);
            /* the actual subscripts used by THERM_OnFrame */
            CK((uint8_t)(m - 1u) < THERM_MODULES, "module_index_in_range",
               "id=%u -> thermLatest[%u][..], bound %u", id, (uint8_t)(m - 1u),
               (unsigned)THERM_MODULES);
            CK((uint8_t)(t - 1u) < THERM_PER_MODULE, "therm_index_in_range",
               "id=%u -> thermLatest[..][%u], bound %u", id, (uint8_t)(t - 1u),
               (unsigned)THERM_PER_MODULE);
            CK(off >= 1u && off <= 9u, "offset_in_range",
               "id=%u -> offset %u", id, off);
            /* the documented direction rule */
            const uint8_t want = (m & 1u) ? off : (uint8_t)(10u - off);
            CK(t == want, "therm_direction_rule", "id=%u module %u -> therm %u, "
               "expected %u", id, m, t, want);
        } else {
            /* Not a bug - documents how load-bearing the guard is. */
            if ((uint8_t)(m - 1u) >= THERM_MODULES || (uint8_t)(t - 1u) >= THERM_PER_MODULE) {
                if (wouldEscape == 0) { firstEscapeLo = id; }
                firstEscapeHi = id;
                wouldEscape++;
            }
        }
    }
    CK(acc == 63, "accepted_id_count", "%d ids accepted, expected 63", acc);
    printf("  2048/2048 ids swept, %d accepted, all indices inside "
           "[0,%u) x [0,%u)\n", acc, (unsigned)THERM_MODULES, (unsigned)THERM_PER_MODULE);
    printf("  guard is load-bearing: %d of the 1985 rejected ids would index "
           "out of bounds if it were removed (e.g. id=%u -> [%u][%u], "
           "id=%u -> [%u][%u])\n", wouldEscape,
           firstEscapeLo, (uint8_t)(MODULE_OF(firstEscapeLo) - 1u),
           (uint8_t)(THERM_OF(firstEscapeLo) - 1u),
           firstEscapeHi, (uint8_t)(MODULE_OF(firstEscapeHi) - 1u),
           (uint8_t)(THERM_OF(firstEscapeHi) - 1u));
    /* offset 0 inside the accepted id band is the PCBCells<x>_NODE frame */
    for (uint32_t id = THERM_ID_FIRST; id <= THERM_ID_LAST; id++) {
        if ((id % 10u) != 0u) { continue; }
        const uint8_t m = MODULE_OF(id), t = THERM_OF(id);
        printf("  offset-0 id %u (NODE frame) would map to [%u][%u] - rejected\n",
               id, (uint8_t)(m - 1u), (uint8_t)(t - 1u));
    }
}

/* ------------------------------------------------------------------ */
/* 4. THERM_OnFrame over every id, with boundary raws                 */
/* ------------------------------------------------------------------ */
static const struct { uint32_t id; uint8_t module; uint8_t therm; const char *name; } kDbc[] = {
#include "dbc_therm_ids.inc"
};

#define PROBE 200u

static void sweepThermOnFrame(void)
{
    section("4. THERM_OnFrame - all 2048 ids x boundary raws");

    static const uint8_t raws[] = { 0u, 1u, 127u, 128u, 254u, 255u };

    /* Pass 1: every id x every boundary raw. ASan aborts on any escape. */
    THERM_Init(NULL);
    for (uint32_t id = 0u; id < 2048u; id++) {
        for (size_t k = 0u; k < sizeof raws / sizeof raws[0]; k++) {
            THERM_OnFrame(id, raws[k]);
        }
    }
    gChecks += 2048u * (sizeof raws / sizeof raws[0]);
    printf("  pass 1: %zu frames ingested (2048 ids x %zu raws) with no ASan "
           "abort\n", 2048u * (sizeof raws / sizeof raws[0]),
           sizeof raws / sizeof raws[0]);

    /* Pass 2: locate the cell each accepted id lands in, and prove the 63
       accepted ids map to 63 distinct cells. */
    long owner[THERM_MODULES][THERM_PER_MODULE];
    for (int p = 0; p < (int)THERM_MODULES; p++) {
        for (int t = 0; t < (int)THERM_PER_MODULE; t++) { owner[p][t] = -1; }
    }
    int distinct = 0;
    for (uint32_t id = 0u; id < 2048u; id++) {
        THERM_Init(NULL);
        THERM_OnFrame(id, (uint8_t)PROBE);
        THERM_Task();                      /* fill == 1, mean of one sample */

        int hitP = -1, hitT = -1, hits = 0;
        for (uint8_t p = 1u; p <= THERM_MODULES; p++) {
            for (uint8_t t = 1u; t <= THERM_PER_MODULE; t++) {
                const uint8_t v = THERM_Filtered(p, t);
                if (v == (uint8_t)PROBE) { hits++; hitP = p; hitT = t; }
                else {
                    CK(v == 0u, "therm_untouched_cell_stays_zero",
                       "id=%u left (%u,%u) = %u", id, p, t, v);
                }
            }
        }
        if (accepted(id)) {
            CK(hits == 1, "therm_frame_lands_in_exactly_one_cell",
               "id=%u landed in %d cells", id, hits);
            if (hits == 1) {
                CK(owner[hitP - 1][hitT - 1] == -1, "therm_cell_collision",
                   "id=%u and id=%ld both map to (module %d, therm %d)",
                   id, owner[hitP - 1][hitT - 1], hitP, hitT);
                if (owner[hitP - 1][hitT - 1] == -1) {
                    owner[hitP - 1][hitT - 1] = (long)id;
                    distinct++;
                }
            }
        } else {
            CK(hits == 0, "rejected_id_must_be_a_no_op",
               "rejected id=%u wrote into (module %d, therm %d)", id, hitP, hitT);
        }
    }
    CK(distinct == 63, "therm_all_63_cells_covered",
       "%d distinct cells reached, expected 63", distinct);
    printf("  pass 2: 63 accepted ids -> %d distinct (module, thermistor) "
           "cells, 1985 rejected ids are no-ops\n", distinct);

    /* Pass 3: cross-check the macro mapping against every generated CAN2_DB
       frame id, not just the 10 the _Static_asserts cover. */
    const size_t n = sizeof kDbc / sizeof kDbc[0];
    for (size_t i = 0u; i < n; i++) {
        THERM_Init(NULL);
        THERM_OnFrame(kDbc[i].id, (uint8_t)PROBE);
        THERM_Task();
        CK(THERM_Filtered(kDbc[i].module, kDbc[i].therm) == (uint8_t)PROBE,
           "dbc_name_matches_cell",
           "%s (id %u) did not land in (module %u, therm %u)",
           kDbc[i].name, kDbc[i].id, kDbc[i].module, kDbc[i].therm);
        CK(MODULE_OF(kDbc[i].id) == kDbc[i].module, "dbc_module_matches_macro",
           "%s (id %u): MODULE_OF = %u, name says %u", kDbc[i].name, kDbc[i].id,
           MODULE_OF(kDbc[i].id), kDbc[i].module);
        CK(THERM_OF(kDbc[i].id) == kDbc[i].therm, "dbc_therm_matches_macro",
           "%s (id %u): THERM_OF = %u, name says %u", kDbc[i].name, kDbc[i].id,
           THERM_OF(kDbc[i].id), kDbc[i].therm);
    }
    printf("  pass 3: all %zu generated PCBCELLS<m>_THERM<t>_FRAME_ID names "
           "cross-checked against the macros\n", n);

    /* Pass 4: raw round-trip. A raw of 0 is indistinguishable from "never
       written", so it is checked through the max-tracking path instead. */
    for (size_t k = 0u; k < sizeof raws / sizeof raws[0]; k++) {
        for (size_t i = 0u; i < n; i++) {
            THERM_Init(NULL);
            THERM_OnFrame(kDbc[i].id, raws[k]);
            THERM_Task();
            CK(THERM_Filtered(kDbc[i].module, kDbc[i].therm) == raws[k],
               "therm_raw_round_trip", "%s raw=%u came back as %u",
               kDbc[i].name, raws[k], THERM_Filtered(kDbc[i].module, kDbc[i].therm));
            if (raws[k] > 0u) {
                CK(THERM_MaxRaw() == raws[k] &&
                   THERM_MaxModule() == kDbc[i].module &&
                   THERM_MaxTherm() == kDbc[i].therm, "therm_argmax",
                   "%s raw=%u: max reported as %u at (%u,%u)", kDbc[i].name,
                   raws[k], THERM_MaxRaw(), THERM_MaxModule(), THERM_MaxTherm());
            }
        }
    }
    printf("  pass 4: 63 cells x %zu boundary raws round-tripped through the "
           "filter and the argmax\n", sizeof raws / sizeof raws[0]);
}

/* ------------------------------------------------------------------ */
/* 5. ADC conversion chain, driven through the DMA buffer             */
/* ------------------------------------------------------------------ */
#define CH_T 0
#define CH_I 1
#define CH_V 2
#define PARK_TEMP 2048u   /* mid-table, no guard-band fault */
#define PARK_CURR 2108u   /* CALIB_CURRENT_OFFSET, exactly 0 A */
#define PARK_VOLT 3000u   /* 686 dV, inside 630..870 */

static CAN_InstanceTypeDef gInst;
static CAN_HandleTypeDef gHcan = { &gInst, { DISABLE } };
static struct CAN_scheduledMsgList gSched;
static EH_HandleTypeDef gEh;
static volatile uint16_t gBuf[3];

static int errIndex(uint16_t code)
{
    for (uint8_t i = 0u; i < gEh.activeErrorCount; i++) {
        if (gEh.activeErrors[i].errorCode == code) { return (int)i; }
    }
    return -1;
}

static void feed(uint16_t t, uint16_t i, uint16_t v, int n)
{
    for (int k = 0; k < n; k++) {
        gBuf[CH_T] = t; gBuf[CH_I] = i; gBuf[CH_V] = v;
        ADC_OnConvComplete();
        ADC_Task(0u);
    }
}

static unsigned long refDecivolts(uint32_t count)
{
    return (((unsigned long)count * CALIB_PACK_V_NUM) + (CALIB_PACK_V_DEN / 2u))
           / CALIB_PACK_V_DEN;
}

static long refDeciamps(uint32_t count)
{
    const long delta = (long)count - (long)CALIB_CURRENT_OFFSET;
    long da = delta * (long)CALIB_CURRENT_NUM;
    da = (da >= 0) ? ((da + (CALIB_CURRENT_DEN / 2)) / CALIB_CURRENT_DEN)
                   : ((da - (CALIB_CURRENT_DEN / 2)) / CALIB_CURRENT_DEN);
    return da;
}

static void sweepAdcChain(void)
{
    section("5. ADC chain - full 12-bit range per channel");

    memset(&gSched, 0, sizeof gSched);
    memset(&gEh, 0, sizeof gEh);
    gInst.MCR = 0u;
    Fake_Reset();
    EH_init(&gEh, &gHcan, BMSMASTER_NODE_FRAME_ID, &gSched);
    ADC_Init(gBuf, &gEh);

    /* ---- voltage ---- */
    uint32_t vLo = 0u, vHi = 0u;
    int vLoSet = 0;
    for (uint32_t c = 0u; c < 4096u; c++) {
        feed(PARK_TEMP, PARK_CURR, (uint16_t)c, 10);
        const unsigned long raw = refDecivolts(c);
        const unsigned long want = (raw < VOLT_MIN_DV) ? VOLT_MIN_DV
                                 : (raw > VOLT_MAX_DV) ? VOLT_MAX_DV : raw;
        const int faulted = (errIndex(BMS_ERR_PACK_VOLT_RANGE) >= 0);
        const int wantFault = (raw < VOLT_MIN_DV || raw > VOLT_MAX_DV);

        CK(ADC_PackDecivolts() == (uint16_t)want, "volt_output_clamped",
           "count=%u raw=%lu dV -> output %u, expected %lu", c,
           raw, ADC_PackDecivolts(), want);
        CK(faulted == wantFault, "volt_fault_code",
           "count=%u raw=%lu dV: fault %d, expected %d", c, raw, faulted, wantFault);
        if (faulted) {
            const int ix = errIndex(BMS_ERR_PACK_VOLT_RANGE);
            const unsigned b0 = gEh.activeErrors[ix].specificData[0];
            const unsigned b1 = gEh.activeErrors[ix].specificData[1];
            CK(b0 == (raw & 0xFFu) && b1 == ((raw >> 8) & 0xFFu),
               "volt_blob_reports_unclamped",
               "count=%u raw=%lu dV: blob %02x %02x", c, raw, b0, b1);
            CK(gEh.activeErrors[ix].specificDataLen == 2u, "volt_blob_len",
               "count=%u len %u", c, gEh.activeErrors[ix].specificDataLen);
        }
        if (!wantFault) { if (!vLoSet) { vLo = c; vLoSet = 1; } vHi = c; }
    }
    printf("  voltage: 4096/4096 counts. no fault for counts %u..%u "
           "(%lu..%lu dV); clamped to %u below and %u above\n",
           vLo, vHi, refDecivolts(vLo), refDecivolts(vHi),
           (unsigned)VOLT_MIN_DV, (unsigned)VOLT_MAX_DV);

    /* ---- current ---- */
    uint32_t iLo = 0u, iHi = 0u;
    int iLoSet = 0;
    for (uint32_t c = 0u; c < 4096u; c++) {
        feed(PARK_TEMP, (uint16_t)c, PARK_VOLT, 10);
        const long raw = refDeciamps(c);
        const long want = (raw > CURRENT_MAX_DA) ? CURRENT_MAX_DA
                        : (raw < -CURRENT_MAX_DA) ? -CURRENT_MAX_DA : raw;
        const int faulted = (errIndex(BMS_ERR_PACK_CURRENT_HIGH) >= 0);
        const int wantFault = (raw > CURRENT_MAX_DA || raw < -CURRENT_MAX_DA);

        CK(ADC_PackDeciamps() == (int16_t)want, "curr_output_clamped",
           "count=%u raw=%ld dA -> output %d, expected %ld", c, raw,
           ADC_PackDeciamps(), want);
        CK(faulted == wantFault, "curr_fault_code",
           "count=%u raw=%ld dA: fault %d, expected %d", c, raw, faulted, wantFault);
        CK(raw >= -32768 && raw <= 32767, "curr_raw_fits_int16",
           "count=%u raw=%ld dA exceeds int16", c, raw);
        if (faulted) {
            const int ix = errIndex(BMS_ERR_PACK_CURRENT_HIGH);
            const unsigned b0 = gEh.activeErrors[ix].specificData[0];
            const unsigned b1 = gEh.activeErrors[ix].specificData[1];
            /* Two's-complement low 16 bits of the UNCLAMPED reading. The >>8
               of a negative int32 is the documented, already-ruled
               implementation-defined case. */
            const unsigned w = (unsigned)(raw & 0xFFFFL);
            CK(b0 == (w & 0xFFu) && b1 == ((w >> 8) & 0xFFu),
               "curr_blob_reports_unclamped",
               "count=%u raw=%ld dA: blob %02x %02x, expected %02x %02x",
               c, raw, b0, b1, w & 0xFFu, (w >> 8) & 0xFFu);
        }
        if (!wantFault) { if (!iLoSet) { iLo = c; iLoSet = 1; } iHi = c; }
    }
    printf("  current: 4096/4096 counts. no fault for counts %u..%u "
           "(%ld..%ld dA); clamped to +-%d outside; sign symmetric at "
           "offset+-1: %ld / %ld\n", iLo, iHi, refDeciamps(iLo), refDeciamps(iHi),
           CURRENT_MAX_DA, refDeciamps(CALIB_CURRENT_OFFSET - 1),
           refDeciamps(CALIB_CURRENT_OFFSET + 1));

    /* ---- temperature ---- */
    uint32_t openHi = 0u, shortLo = 0u;
    int shortSet = 0;
    for (uint32_t c = 0u; c < 4096u; c++) {
        feed((uint16_t)c, PARK_CURR, PARK_VOLT, 10);
        const uint16_t want = countToCenti((uint16_t)c);
        const int faulted = (errIndex(BMS_ERR_TEMP_SENSOR_FAULT) >= 0);
        const int wantFault = (c < NTC_OPEN_BELOW || c > NTC_SHORT_ABOVE);

        CK(ADC_TempCenti() == want, "temp_output",
           "count=%u -> %u centi, expected %u", c, ADC_TempCenti(), want);
        CK(ADC_TempCenti() <= TEMP_MAX_CENTI, "temp_output_in_range",
           "count=%u -> %u centi", c, ADC_TempCenti());
        CK(faulted == wantFault, "temp_fault_code",
           "count=%u: fault %d, expected %d", c, faulted, wantFault);
        if (faulted) {
            const int ix = errIndex(BMS_ERR_TEMP_SENSOR_FAULT);
            CK(gEh.activeErrors[ix].specificData[0] == (c & 0xFFu) &&
               gEh.activeErrors[ix].specificData[1] == ((c >> 8) & 0xFFu),
               "temp_blob_reports_count", "count=%u blob %02x %02x", c,
               gEh.activeErrors[ix].specificData[0],
               gEh.activeErrors[ix].specificData[1]);
        }
        if (c < NTC_OPEN_BELOW) { openHi = c; }
        if (c > NTC_SHORT_ABOVE && !shortSet) { shortLo = c; shortSet = 1; }
    }
    printf("  temperature: 4096/4096 counts. open band 0..%u reports %u centi, "
           "short band %u..4095 reports %u centi, both with code %u\n",
           openHi, countToCenti((uint16_t)openHi), shortLo,
           countToCenti((uint16_t)shortLo), (unsigned)BMS_ERR_TEMP_SENSOR_FAULT);
    printf("  gap: counts %u..%u are below the table but inside the guard "
           "band, so they report 0 centi with NO fault (documented DBC "
           "unsigned-signal limitation)\n", (unsigned)NTC_OPEN_BELOW,
           calibNtcCount[0]);
    printf("  gap: counts %u..%u are above the table but inside the guard "
           "band, so they report %u centi with NO fault\n",
           calibNtcCount[100], (unsigned)NTC_SHORT_ABOVE, (unsigned)TEMP_MAX_CENTI);

    /* ---- DMA buffer masking: all 65536 halfword values on one channel ---- */
    unsigned long masked = 0u;
    for (uint32_t v = 0u; v <= 0xFFFFu; v++) {
        feed(PARK_TEMP, PARK_CURR, (uint16_t)v, 10);
        const unsigned long raw = refDecivolts(v & 0x0FFFu);
        const unsigned long want = (raw < VOLT_MIN_DV) ? VOLT_MIN_DV
                                 : (raw > VOLT_MAX_DV) ? VOLT_MAX_DV : raw;
        CK(ADC_PackDecivolts() == (uint16_t)want, "dma_word_masked_to_12_bits",
           "buffer=0x%04x -> %u dV, expected %lu (masked count %u)", v,
           ADC_PackDecivolts(), want, v & 0x0FFFu);
        if (v > 0x0FFFu) { masked++; }
    }
    printf("  DMA buffer: 65536/65536 halfword values on the voltage channel, "
           "%lu of them above 12 bits, all masked to the low 12 bits\n", masked);
}

/* ------------------------------------------------------------------ */
/* 6. trimmedMean - both copies, adversarial windows                  */
/* ------------------------------------------------------------------ */

/* Reference: drop exactly one minimum and one maximum, then round-to-nearest
   on the remaining fill-2 samples. */
static unsigned long refTrim(const unsigned long *s, int fill)
{
    unsigned long c[16];
    for (int i = 0; i < fill; i++) { c[i] = s[i]; }
    for (int i = 1; i < fill; i++) {           /* insertion sort */
        const unsigned long k = c[i];
        int j = i - 1;
        while (j >= 0 && c[j] > k) { c[j + 1] = c[j]; j--; }
        c[j + 1] = k;
    }
    unsigned long sum = 0u;
    if (fill < 3) {
        for (int i = 0; i < fill; i++) { sum += c[i]; }
        return (sum + (unsigned long)(fill / 2)) / (unsigned long)fill;
    }
    for (int i = 1; i < fill - 1; i++) { sum += c[i]; }
    const unsigned long n = (unsigned long)(fill - 2);
    return (sum + (n / 2u)) / n;
}

static void checkWindow(const char *what, const unsigned long *vals, int fill)
{
    uint8_t u8[16];
    uint16_t u16[16];
    for (int i = 0; i < fill; i++) {
        u8[i] = (uint8_t)(vals[i] > 255u ? 255u : vals[i]);
        u16[i] = (uint16_t)vals[i];
    }
    const unsigned long want16 = refTrim(vals, fill);
    unsigned long v8[16];
    for (int i = 0; i < fill; i++) { v8[i] = u8[i]; }
    const unsigned long want8 = refTrim(v8, fill);

    CK(adcTrimmedMean(u16, (uint8_t)fill) == (uint16_t)want16, "trim_u16",
       "%s fill=%d -> %u, expected %lu", what, fill,
       adcTrimmedMean(u16, (uint8_t)fill), want16);
    CK(thermTrimmedMean(u8, (uint8_t)fill) == (uint8_t)want8, "trim_u8",
       "%s fill=%d -> %u, expected %lu", what, fill,
       thermTrimmedMean(u8, (uint8_t)fill), want8);
}

static void sweepTrimmedMean(void)
{
    section("6. trimmedMean - adversarial windows, both copies");

    /* Divisor must be 8 for a full 10-sample window: eight 10s survive. */
    {
        const uint8_t w8[10] = { 0u, 10u, 10u, 10u, 10u, 10u, 10u, 10u, 10u, 255u };
        const uint16_t w16[10] = { 0u, 10u, 10u, 10u, 10u, 10u, 10u, 10u, 10u, 4095u };
        CK(thermTrimmedMean(w8, 10u) == 10u, "trim_u8_divisor_is_8",
           "{0,10x8,255} -> %u, expected 10", thermTrimmedMean(w8, 10u));
        CK(adcTrimmedMean(w16, 10u) == 10u, "trim_u16_divisor_is_8",
           "{0,10x8,4095} -> %u, expected 10", adcTrimmedMean(w16, 10u));
        /* 8*x+4 rounding: a window whose eight survivors sum to 4 must be 1 */
        const uint8_t r8[10] = { 0u, 1u, 1u, 1u, 1u, 0u, 0u, 0u, 0u, 255u };
        CK(thermTrimmedMean(r8, 10u) == 1u, "trim_u8_rounding",
           "survivors sum 4 over 8 -> %u, expected 1 (round half up)",
           thermTrimmedMean(r8, 10u));
    }

    struct { const char *name; unsigned long v[10]; } cases[] = {
        { "all-equal-zero",      { 0,0,0,0,0,0,0,0,0,0 } },
        { "all-equal-mid",       { 137,137,137,137,137,137,137,137,137,137 } },
        { "all-min-u8",          { 0,0,0,0,0,0,0,0,0,0 } },
        { "all-max-u8",          { 255,255,255,255,255,255,255,255,255,255 } },
        { "all-max-u16",         { 65535,65535,65535,65535,65535,65535,65535,65535,65535,65535 } },
        { "all-max-12bit",       { 4095,4095,4095,4095,4095,4095,4095,4095,4095,4095 } },
        { "tied-at-both-ends",   { 0,0,0,10,10,10,10,255,255,255 } },
        { "two-min-two-max",     { 0,0,128,128,128,128,128,128,255,255 } },
        { "single-low-outlier",  { 0,200,200,200,200,200,200,200,200,200 } },
        { "single-high-outlier", { 200,200,200,200,200,200,200,200,200,255 } },
        { "outlier-both-ends",   { 0,200,200,200,200,200,200,200,200,255 } },
        { "alternating-extremes",{ 0,255,0,255,0,255,0,255,0,255 } },
        { "monotone-ramp",       { 0,28,56,85,113,141,170,198,226,255 } },
        { "u16-full-scale-mix",  { 0,65535,32768,32767,1,65534,2,65533,3,65532 } },
        { "u16-12bit-mix",       { 0,4095,2048,2047,1,4094,2,4093,3,4092 } },
    };
    const int nCases = (int)(sizeof cases / sizeof cases[0]);
    for (int i = 0; i < nCases; i++) {
        for (int fill = 1; fill <= 10; fill++) {
            checkWindow(cases[i].name, cases[i].v, fill);
        }
    }
    printf("  %d named adversarial windows x fill 1..10 = %d evaluations, "
           "both copies, against an independent sort-and-drop reference\n",
           nCases, nCases * 10);

    /* Exhaustive: every 3-sample u8 window. trimmedMean is order-insensitive
       for a full window, so this also covers every permutation. */
    unsigned long tri[3];
    for (unsigned a = 0u; a < 256u; a++) {
        for (unsigned b = 0u; b < 256u; b++) {
            for (unsigned c = 0u; c < 256u; c++) {
                const uint8_t w[3] = { (uint8_t)a, (uint8_t)b, (uint8_t)c };
                tri[0] = a; tri[1] = b; tri[2] = c;
                const unsigned long want = refTrim(tri, 3);
                if (thermTrimmedMean(w, 3u) != (uint8_t)want) {
                    fail("trim_u8_exhaustive_fill3",
                         "{%u,%u,%u} -> %u, expected %lu", a, b, c,
                         thermTrimmedMean(w, 3u), want);
                }
                gChecks++;
            }
        }
    }
    printf("  exhaustive: all 16777216 three-sample uint8 windows match the "
           "reference\n");

    /* Exhaustive over a boundary alphabet at full window length: all
       multisets of size 10, which is complete because the full window is
       order-insensitive. */
    static const unsigned long alpha[] = { 0u, 1u, 2u, 127u, 128u, 254u, 255u, 4095u, 65535u };
    const int A = (int)(sizeof alpha / sizeof alpha[0]);
    unsigned long win[10];
    int idx[10];
    long combos = 0;
    for (int i = 0; i < 10; i++) { idx[i] = 0; }
    for (;;) {
        for (int i = 0; i < 10; i++) { win[i] = alpha[idx[i]]; }
        checkWindow("alphabet-multiset", win, 10);
        combos++;
        /* next non-decreasing index tuple */
        int p = 9;
        while (p >= 0 && idx[p] == A - 1) { p--; }
        if (p < 0) { break; }
        idx[p]++;
        for (int q = p + 1; q < 10; q++) { idx[q] = idx[p]; }
    }
    printf("  exhaustive: all %ld size-10 multisets over the %d-symbol "
           "boundary alphabet {0,1,2,127,128,254,255,4095,65535}\n", combos, A);
}


/* ------------------------------------------------------------------ */
/* 7. the fill parameter and the THERM_Filtered accessor              */
/* ------------------------------------------------------------------ */
static void sweepFillAndAccessor(void)
{
    section("7. fill parameter (0..255) and THERM_Filtered(module, therm)");

    /* fill is a uint8_t, so its whole domain is 256 values. The u8 copy
       accumulates into a uint16 and the u16 copy into a uint32: sweep the
       worst case for each to see whether either accumulator can wrap. */
    static uint8_t big8[256];
    static uint16_t big16[256];
    unsigned long refBuf[256];

    struct { const char *name; int mode; } pat[] = {
        { "all-max",     0 }, { "all-min", 1 }, { "ramp", 2 }, { "alternating", 3 }
    };
    for (int q = 0; q < 4; q++) {
        for (int i = 0; i < 256; i++) {
            switch (pat[q].mode) {
            case 0:  big8[i] = 255u;              big16[i] = 0xFFFFu; break;
            case 1:  big8[i] = 0u;                big16[i] = 0u; break;
            case 2:  big8[i] = (uint8_t)i;        big16[i] = (uint16_t)(i * 257); break;
            default: big8[i] = (i & 1) ? 255u : 0u;
                     big16[i] = (i & 1) ? 0xFFFFu : 0u; break;
            }
        }
        for (int fill = 1; fill <= 255; fill++) {
            for (int i = 0; i < fill; i++) { refBuf[i] = big8[i]; }
            unsigned long want = 0u;
            {   /* reference over at most 255 samples */
                unsigned long c[256];
                for (int i = 0; i < fill; i++) { c[i] = refBuf[i]; }
                for (int i = 1; i < fill; i++) {
                    const unsigned long k = c[i];
                    int j = i - 1;
                    while (j >= 0 && c[j] > k) { c[j + 1] = c[j]; j--; }
                    c[j + 1] = k;
                }
                unsigned long sum = 0u;
                if (fill < 3) {
                    for (int i = 0; i < fill; i++) { sum += c[i]; }
                    want = (sum + (unsigned long)(fill / 2)) / (unsigned long)fill;
                } else {
                    for (int i = 1; i < fill - 1; i++) { sum += c[i]; }
                    const unsigned long n = (unsigned long)(fill - 2);
                    want = (sum + (n / 2u)) / n;
                }
            }
            CK(thermTrimmedMean(big8, (uint8_t)fill) == (uint8_t)want,
               "trim_u8_fill_domain", "%s fill=%d -> %u, expected %lu",
               pat[q].name, fill, thermTrimmedMean(big8, (uint8_t)fill), want);

            for (int i = 0; i < fill; i++) { refBuf[i] = big16[i]; }
            unsigned long want16 = 0u;
            {
                unsigned long c[256];
                for (int i = 0; i < fill; i++) { c[i] = refBuf[i]; }
                for (int i = 1; i < fill; i++) {
                    const unsigned long k = c[i];
                    int j = i - 1;
                    while (j >= 0 && c[j] > k) { c[j + 1] = c[j]; j--; }
                    c[j + 1] = k;
                }
                unsigned long sum = 0u;
                if (fill < 3) {
                    for (int i = 0; i < fill; i++) { sum += c[i]; }
                    want16 = (sum + (unsigned long)(fill / 2)) / (unsigned long)fill;
                } else {
                    for (int i = 1; i < fill - 1; i++) { sum += c[i]; }
                    const unsigned long n = (unsigned long)(fill - 2);
                    want16 = (sum + (n / 2u)) / n;
                }
            }
            CK(adcTrimmedMean(big16, (uint8_t)fill) == (uint16_t)want16,
               "trim_u16_fill_domain", "%s fill=%d -> %u, expected %lu",
               pat[q].name, fill, adcTrimmedMean(big16, (uint8_t)fill), want16);
        }
    }
    printf("  fill 1..255 x 4 worst-case patterns, both copies: the uint16 "
           "accumulator survives its whole domain (255*255=65025 < 65536) "
           "and so does the uint32 one\n");
    printf("  fill=0 is the one value that traps; probed separately with "
           "--hazard-zero-fill\n");

    /* THERM_Filtered takes two uint8_t: 65536 pairs, all enumerable. */
    THERM_Init(NULL);
    for (size_t i = 0u; i < sizeof kDbc / sizeof kDbc[0]; i++) {
        THERM_OnFrame(kDbc[i].id, (uint8_t)(i + 1u));
    }
    THERM_Task();
    unsigned long inRange = 0u;
    for (unsigned m = 0u; m < 256u; m++) {
        for (unsigned t = 0u; t < 256u; t++) {
            const uint8_t v = THERM_Filtered((uint8_t)m, (uint8_t)t);
            if (m >= 1u && m <= THERM_MODULES && t >= 1u && t <= THERM_PER_MODULE) {
                inRange++;
                CK(v != 0u, "filtered_in_range_returns_cell",
                   "(%u,%u) returned 0 but a frame was ingested for it", m, t);
            } else {
                CK(v == 0u, "filtered_out_of_range_returns_zero",
                   "(%u,%u) returned %u, expected 0", m, t, v);
            }
        }
    }
    printf("  THERM_Filtered: all 65536 (module, therm) pairs, %lu in range "
           "return their cell, the other %lu return 0 with no ASan abort\n",
           inRange, 65536u - inRange);
}

/* ------------------------------------------------------------------ */
/* opt-in: latent hazards that abort the process on purpose           */
/* ------------------------------------------------------------------ */
static void hazardZeroFill(void)
{
    printf("about to call trimmedMean(samples, fill=0) - expect a "
           "division-by-zero trap\n");
    fflush(stdout);
    const uint8_t w[1] = { 0u };
    const uint8_t r = thermTrimmedMean(w, 0u);
    printf("no trap; returned %u\n", r);
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--hazard-zero-fill") == 0) {
        hazardZeroFill();
        return 0;
    }

    printf("BMS Master exhaustive numeric sweep\n");
    printf("host: sizeof(int)=%zu sizeof(long)=%zu sizeof(void*)=%zu\n",
           sizeof(int), sizeof(long), sizeof(void *));

    sweepCountToCenti();
    sweepDecodeTemp();
    sweepIdMacros();
    sweepThermOnFrame();
    sweepAdcChain();
    sweepTrimmedMean();
    sweepFillAndAccessor();

    printf("\n== summary\n");
    printf("  %lu assertions evaluated, %lu violations\n", gChecks, gFails);
    for (int i = 0; i < gSiteN; i++) {
        printf("  %-40s %lu\n", gSite[i].name, gSite[i].hits);
    }
    return (gFails != 0u) ? 1 : 0;
}

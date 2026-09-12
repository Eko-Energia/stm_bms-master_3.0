/*
 * Standalone stress/fuzz harness for App/Src/jk_protocol.c
 *
 * jk_protocol is pure (no HAL, no static state, no I/O), so it links natively
 * with no fakes. This harness drives five input populations at it and checks
 * both memory safety (via ASan/UBSan) and semantic invariants that a
 * sanitizer cannot see.
 *
 * Build/run: tests/stress/run_fuzz.sh
 *   ./fuzz_jk [iterations_per_population] [seed] [-x]
 *   -x  abort on the first violation instead of collecting one witness per class
 *
 * Buffers handed to the parser are heap allocations of *exactly* the declared
 * len, so any read past len is a hard ASan error rather than a silent slop
 * read into a fat static buffer.
 */
#include "jk_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Mirror of jk_protocol.c's private JKP_OVERHEAD (2 STX + 2 LENGTH + 4 terminal
   + cmd + src + type + 4 record + end flag + 4 checksum). Kept local so the
   harness does not need to modify the module under test. */
#define JKP_OVERHEAD_PROBE (20u)

/* ------------------------------------------------------------------ PRNG */
static uint64_t g_rngState;
static uint64_t g_seed;
static int      g_abortFirst = 0;

#if defined(__clang__)
__attribute__((no_sanitize("integer")))
#endif
static uint64_t rnd64(void)
{
    uint64_t x = g_rngState;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    g_rngState = x;
    return x;
}
static uint32_t rnd(uint32_t bound) { return bound ? (uint32_t)(rnd64() % bound) : 0u; }
static uint8_t  rndByte(void)       { return (uint8_t)(rnd64() >> 33); }

/* ------------------------------------------------- violation bookkeeping */
#define VIOL_MAX 32
typedef struct {
    const char *name;
    unsigned long count;
    char         witness[8192];
} Viol_t;
static Viol_t g_viol[VIOL_MAX];
static int    g_violN;

#define HEX_CAP 140u   /* witnesses stay readable; minimal reproducers below carry the detail */

static void hexDump(char *dst, size_t dstLen, const uint8_t *b, uint16_t len)
{
    const uint16_t show = (len > HEX_CAP) ? HEX_CAP : len;
    size_t o = 0;
    for (uint16_t i = 0u; i < show && o + 4u < dstLen; i++) {
        o += (size_t)snprintf(dst + o, dstLen - o, "%02x ", b[i]);
    }
    if (o) { dst[o - 1] = '\0'; } else { dst[0] = '\0'; }
    if (show < len) {
        snprintf(dst + o, dstLen - o, " ... (+%u more bytes)", (unsigned)(len - show));
    }
}

static void report(const char *cls, const char *pop, unsigned long iter,
                   const uint8_t *buf, uint16_t len, const char *detail)
{
    Viol_t *v = NULL;
    for (int i = 0; i < g_violN; i++) {
        if (strcmp(g_viol[i].name, cls) == 0) { v = &g_viol[i]; break; }
    }
    if (v == NULL && g_violN < VIOL_MAX) {
        v = &g_viol[g_violN++];
        v->name = cls;
        v->count = 0u;
        char hex[6000];
        hexDump(hex, sizeof hex, buf, len);
        snprintf(v->witness, sizeof v->witness,
                 "  population : %s (iteration %lu)\n"
                 "  detail     : %s\n"
                 "  len        : %u\n"
                 "  bytes      : %s\n",
                 pop, iter, detail, (unsigned)len, hex);
    }
    if (v) { v->count++; }

    if (g_abortFirst) {
        fprintf(stderr, "\n*** VIOLATION: %s\n  seed=%llu\n%s",
                cls, (unsigned long long)g_seed, v ? v->witness : detail);
        abort();
    }
}

/* ------------------------------------------------------- frame synthesis */
/* Full frame from a TLV payload, with a correct LENGTH, end flag and sum. */
static uint16_t buildFrame(uint8_t *out, const uint8_t *payload, uint16_t payloadLen,
                           uint8_t cmd, uint8_t src, uint8_t type)
{
    const uint16_t total = (uint16_t)(payloadLen + 20u);
    out[0] = 0x4Eu; out[1] = 0x57u;
    out[2] = (uint8_t)(((total - 2u) >> 8) & 0xFFu);
    out[3] = (uint8_t)((total - 2u) & 0xFFu);
    memset(&out[4], 0, 4);                       /* terminal id */
    out[8] = cmd; out[9] = src; out[10] = type;
    if (payloadLen) { memcpy(&out[11], payload, payloadLen); }
    const uint16_t rec = (uint16_t)(11u + payloadLen);
    memset(&out[rec], 0, 4);                     /* record number */
    const uint16_t endIdx = (uint16_t)(rec + 4u);
    out[endIdx]      = 0x68u;
    out[endIdx + 1u] = 0x00u;                    /* disabled CRC16 slot */
    out[endIdx + 2u] = 0x00u;
    uint16_t sum = 0u;
    for (uint16_t i = 0u; i <= endIdx; i++) { sum = (uint16_t)(sum + out[i]); }
    out[endIdx + 3u] = (uint8_t)(sum >> 8);
    out[endIdx + 4u] = (uint8_t)(sum & 0xFFu);
    return total;
}

/* Recompute the trailing accumulated sum so a mutated frame still validates.
   Without this, mutations almost never reach the TLV walk. */
static void fixSum(uint8_t *buf, uint16_t len)
{
    if (len < 5u) { return; }
    uint16_t sum = 0u;
    for (uint16_t i = 0u; i <= (uint16_t)(len - 5u); i++) { sum = (uint16_t)(sum + buf[i]); }
    buf[len - 2u] = (uint8_t)(sum >> 8);
    buf[len - 1u] = (uint8_t)(sum & 0xFFu);
}

/* A realistic 0x06 read-all reply, derived from the protocol (spec 7.1/7.4),
   not from an invented capture. 21S pack, all fields the decoder consumes. */
static uint16_t goodFrame(uint8_t *out)
{
    uint8_t p[512];
    uint16_t n = 0u;
    p[n++] = 0x79u;                    /* cell block: 21 cells x 3 bytes */
    p[n++] = 63u;
    for (uint8_t c = 1u; c <= 21u; c++) {
        uint16_t mv = (uint16_t)(3600u + c);
        p[n++] = c;
        p[n++] = (uint8_t)(mv >> 8);
        p[n++] = (uint8_t)(mv & 0xFFu);
    }
    p[n++] = 0x80u; p[n++] = 0x00u; p[n++] = 30u;      /* mos temp  +30 C   */
    p[n++] = 0x81u; p[n++] = 0x00u; p[n++] = 105u;     /* bal temp  -5 C    */
    p[n++] = 0x82u; p[n++] = 0x00u; p[n++] = 28u;      /* batt temp (skipped)*/
    p[n++] = 0x83u; p[n++] = 0x1Du; p[n++] = 0x4Cu;    /* 74.68 V           */
    p[n++] = 0x84u; p[n++] = 0x2Bu; p[n++] = 0x67u;    /* current           */
    p[n++] = 0x85u; p[n++] = 76u;                      /* SOC 76 %          */
    p[n++] = 0x86u; p[n++] = 0x02u;                    /* temp sensor count */
    p[n++] = 0x87u; p[n++] = 0x00u; p[n++] = 0x2Au;    /* 42 cycles         */
    p[n++] = 0x89u; p[n++] = 0x00u; p[n++] = 0x00u; p[n++] = 0x01u; p[n++] = 0x00u;
    p[n++] = 0x8Au; p[n++] = 0x00u; p[n++] = 21u;      /* 21 strings        */
    p[n++] = 0x8Bu; p[n++] = 0x00u; p[n++] = 0x00u;    /* no warnings       */
    p[n++] = 0x8Cu; p[n++] = 0x00u; p[n++] = 0x03u;    /* mode flags        */
    p[n++] = 0xAAu; p[n++] = 0x00u; p[n++] = 0x00u; p[n++] = 0x00u; p[n++] = 0x64u; /* set 100 Ah */
    p[n++] = 0xB9u; p[n++] = 0x00u; p[n++] = 0x00u; p[n++] = 0x00u; p[n++] = 0x5Au; /*  act  90 Ah */
    p[n++] = 0xC0u; p[n++] = 0x01u;                    /* protocol version 1 */
    return buildFrame(out, p, n, 0x06u, 0x00u, 0x01u);
}

/* ----------------------------------------------------------- invariants */
/* Spec 7.4: temperature raw 0..100 -> +0..+100 C, 101..200 -> -1..-100 C.
   Anything outside +/-100 C means a raw the protocol never defines got
   published as a plausible-looking reading. */
#define TEMP_MIN (-100)
#define TEMP_MAX (100)

static void checkInvariants(const char *pop, unsigned long iter,
                            const uint8_t *buf, uint16_t len, const JK_Data_t *d)
{
    char det[256];

    if (d->soc > 100u) {
        snprintf(det, sizeof det, "JKP_Decode()==true with soc=%u (must be 0..100)", d->soc);
        report("soc out of range 0..100", pop, iter, buf, len, det);
    }
    if (d->soh > 100u) {
        snprintf(det, sizeof det, "soh=%u (must be 0..100)", d->soh);
        report("soh out of range 0..100", pop, iter, buf, len, det);
    }
    if (d->cellCount > JKP_CELLS_MAX) {
        snprintf(det, sizeof det, "cellCount=%u (pack is 21S, spec 7.5)", d->cellCount);
        report("cellCount > 21", pop, iter, buf, len, det);
    }
    if (d->mosTempC < TEMP_MIN || d->mosTempC > TEMP_MAX) {
        snprintf(det, sizeof det, "mosTempC=%d, outside the %d..%d C band the "
                 "0x80 encoding can express", d->mosTempC, TEMP_MIN, TEMP_MAX);
        report("mosTempC outside protocol band", pop, iter, buf, len, det);
    }
    if (d->balTempC < TEMP_MIN || d->balTempC > TEMP_MAX) {
        snprintf(det, sizeof det, "balTempC=%d, outside the %d..%d C band the "
                 "0x81 encoding can express", d->balTempC, TEMP_MIN, TEMP_MAX);
        report("balTempC outside protocol band", pop, iter, buf, len, det);
    }
    if (d->modeFlags > 0x0Fu) {
        /* spec 7.4: "only bits 0-3 are defined" */
        snprintf(det, sizeof det, "modeFlags=0x%02x, bits above b3 are undefined", d->modeFlags);
        report("modeFlags has undefined bits set", pop, iter, buf, len, det);
    }
}

/* Spec 7.5: a JK-reported cell above 21 must raise a fault, not be dropped -
   those cells would be invisible to the vehicle. Independently walk the
   payload of an accepted frame and see whether the decoder silently discarded
   any 0x79 entry. */
static void checkCellOverflowDropped(const char *pop, unsigned long iter,
                                     const uint8_t *buf, uint16_t len)
{
    const uint16_t payloadEnd = (uint16_t)(len - 9u);
    uint16_t i = 11u;
    while (i < payloadEnd) {
        const uint8_t id = buf[i];
        if (id == 0x79u) {
            if ((uint16_t)(i + 2u) > payloadEnd) { return; }
            const uint8_t bl = buf[i + 1u];
            if ((uint16_t)(i + 2u + bl) > payloadEnd) { return; }
            if ((uint16_t)(bl / 3u) > JKP_CELLS_MAX) { return; }
            for (uint16_t g = 0u; (uint16_t)(g + 3u) <= bl; g = (uint16_t)(g + 3u)) {
                const uint8_t cellNo = buf[i + 2u + g];
                if (cellNo == 0u || cellNo > JKP_CELLS_MAX) {
                    char det[256];
                    snprintf(det, sizeof det,
                             "0x79 block declares cell number %u; JKP_Decode returned true "
                             "and silently dropped it (spec 7.5 requires a fault)", cellNo);
                    report("0x79 out-of-range cell number silently dropped",
                           pop, iter, buf, len, det);
                    return;
                }
            }
            if (bl % 3u != 0u) {
                char det[256];
                snprintf(det, sizeof det,
                         "0x79 blockLen=%u is not a multiple of 3; %u trailing byte(s) "
                         "ignored and the frame still accepted", bl, bl % 3u);
                report("0x79 blockLen not a multiple of 3 accepted", pop, iter, buf, len, det);
                return;
            }
            i = (uint16_t)(i + 2u + bl);
            continue;
        }
        break;  /* only inspect a leading 0x79 run; deeper walk is the decoder's job */
    }
}

/* Independently walk an accepted frame and pull out the raws the decoder used,
   so the harness can recompute a field in wide arithmetic and prove a
   narrowing rather than guess at a "plausible" band. */
typedef struct {
    bool     have84, haveC0, have83, haveAA, haveB9;
    uint16_t raw84, raw83;
    uint32_t rawAA, rawB9;
    uint8_t  verC0;
} Raws_t;

static uint8_t harnessIdentLen(uint8_t id)
{
    static const uint8_t k1[] = {
        0x85,0x86,0x9D,0xA9,0xAB,0xAC,0xAE,0xAF,0xB1,0xB3,0xB8,0xBB,0xBC,0xBD,0xC0 };
    static const uint8_t k2[] = {
        0x80,0x81,0x82,0x83,0x84,0x87,0x8A,0x8B,0x8C,0x8E,0x8F,0x90,0x91,0x92,0x93,
        0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9E,0x9F,0xA0,0xA1,0xA2,0xA3,
        0xA4,0xA5,0xA6,0xA7,0xA8,0xAD,0xB0,0xBE,0xBF };
    static const uint8_t k4[] = { 0x89,0xAA,0xB5,0xB6,0xB9 };
    for (size_t k = 0; k < sizeof k1; k++) { if (k1[k] == id) { return 1u; } }
    for (size_t k = 0; k < sizeof k2; k++) { if (k2[k] == id) { return 2u; } }
    for (size_t k = 0; k < sizeof k4; k++) { if (k4[k] == id) { return 4u; } }
    if (id == 0xB4u) { return 8u; }
    if (id == 0xB2u) { return 10u; }
    if (id == 0xB7u) { return 15u; }
    if (id == 0xBAu) { return 24u; }
    return 0u;
}

static void collectRaws(const uint8_t *buf, uint16_t len, Raws_t *r)
{
    memset(r, 0, sizeof *r);
    const uint16_t payloadEnd = (uint16_t)(len - 9u);
    uint16_t i = 11u;
    while (i < payloadEnd) {
        const uint8_t id = buf[i];
        if (id == 0x79u) {
            if ((uint16_t)(i + 2u) > payloadEnd) { return; }
            const uint8_t bl = buf[i + 1u];
            if ((uint16_t)(i + 2u + bl) > payloadEnd) { return; }
            if ((uint16_t)(bl / 3u) > JKP_CELLS_MAX) { return; }
            i = (uint16_t)(i + 2u + bl);
            continue;
        }
        const uint8_t dl = harnessIdentLen(id);
        if (dl == 0u || (uint16_t)(i + 1u + dl) > payloadEnd) { return; }
        if (id == 0x84u) { r->have84 = true; r->raw84 = (uint16_t)((buf[i+1u] << 8) | buf[i+2u]); }
        if (id == 0x83u) { r->have83 = true; r->raw83 = (uint16_t)((buf[i+1u] << 8) | buf[i+2u]); }
        if (id == 0xC0u) { r->haveC0 = true; r->verC0 = buf[i + 1u]; }
        if (id == 0xAAu) { r->haveAA = true;
            r->rawAA = ((uint32_t)buf[i+1u] << 24) | ((uint32_t)buf[i+2u] << 16)
                     | ((uint32_t)buf[i+3u] << 8) | buf[i+4u]; }
        if (id == 0xB9u) { r->haveB9 = true;
            r->rawB9 = ((uint32_t)buf[i+1u] << 24) | ((uint32_t)buf[i+2u] << 16)
                     | ((uint32_t)buf[i+3u] << 8) | buf[i+4u]; }
        i = (uint16_t)(i + 1u + dl);
    }
}

/* The publishing contract, i.e. what the CAN_DB signals are defined to carry:
   BMSMaster_JK_PackCurrent +/-300.00 A (also the L01Z300S05 Hall sensor and
   the pack, docs/notionSpec.md) and BMSMaster_JK_PackVoltage 0..100.00 V.
   A 21S Li-ion pack sits at 63.00-88.20 V in normal use, but a deeply
   discharged or overcharged pack is a real reading the vehicle needs, not a
   frame to drop, so the signal range - not the operating range - is the gate. */
#define AMPS_ABS_MAX  (30000)
#define VOLTS_MIN     (0u)
#define VOLTS_MAX     (10000u)

static void checkDerivedFields(const char *pop, unsigned long iter,
                               const uint8_t *buf, uint16_t len, const JK_Data_t *d)
{
    Raws_t r;
    collectRaws(buf, len, &r);
    char det[320];

    if (r.have84) {
        /* Recompute in int32: no narrowing, no wrap. */
        int32_t want;
        if (r.haveC0 && r.verC0 == 0x01u) {
            const int32_t mag = (int32_t)(r.raw84 & 0x7FFFu);
            want = (r.raw84 & 0x8000u) ? -mag : mag;
        } else {
            want = (int32_t)r.raw84 - 10000;
        }
        if (want != (int32_t)d->packCentiamps) {
            snprintf(det, sizeof det,
                     "0x84 raw=0x%04x (0xc0 version %s%u) -> exact %ld centiamps, "
                     "but packCentiamps is %d: the int16_t cast wrapped",
                     r.raw84, r.haveC0 ? "" : "absent, treated as ", r.haveC0 ? r.verC0 : 0u,
                     (long)want, d->packCentiamps);
            report("0x84 current narrowed/wrapped into int16_t", pop, iter, buf, len, det);
        }
        if (d->packCentiamps > AMPS_ABS_MAX || d->packCentiamps < -AMPS_ABS_MAX) {
            snprintf(det, sizeof det,
                     "packCentiamps=%d exceeds the +/-300 A the pack and the Hall "
                     "sensor can carry", d->packCentiamps);
            report("packCentiamps physically impossible", pop, iter, buf, len, det);
        }
    }
    /* The SOH clamp turns an absurd capacity pair into "perfect health". */
    if (r.haveAA && r.haveB9 && r.rawAA > 0u && r.rawAA <= 100000u && r.rawB9 <= 100000u
        && d->soh == 100u && (uint64_t)r.rawB9 * 100u > (uint64_t)r.rawAA * 120u) {
        snprintf(det, sizeof det,
                 "0xb9 actual=%u Ah vs 0xaa set=%u Ah is %.0f%% of rated; the clamp "
                 "published soh=100 (perfect health) for an impossible capacity pair",
                 r.rawB9, r.rawAA, 100.0 * (double)r.rawB9 / (double)r.rawAA);
        report("SOH clamp turns an absurd capacity pair into 100 %", pop, iter, buf, len, det);
    }
    if (r.have83 && (d->packCentivolts < VOLTS_MIN || d->packCentivolts > VOLTS_MAX)) {
        snprintf(det, sizeof det,
                 "0x83 raw=%u published as packCentivolts=%u (%u.%02u V); "
                 "BMSMaster_JK_PackVoltage carries 0.00-100.00 V",
                 r.raw83, d->packCentivolts,
                 d->packCentivolts / 100u, d->packCentivolts % 100u);
        report("packCentivolts impossible for a 21S pack", pop, iter, buf, len, det);
    }
}

/* Detect a walk that stopped early on an unknown/short identifier yet still
   returned true. Reported once as an informational class. */
static void checkTruncatedWalkAccepted(const char *pop, unsigned long iter,
                                       const uint8_t *buf, uint16_t len)
{
    static const uint8_t known1[] = {
        0x85,0x86,0x9D,0xA9,0xAB,0xAC,0xAE,0xAF,0xB1,0xB3,0xB8,0xBB,0xBC,0xBD,0xC0 };
    static const uint8_t known2[] = {
        0x80,0x81,0x82,0x83,0x84,0x87,0x8A,0x8B,0x8C,0x8E,0x8F,0x90,0x91,0x92,0x93,
        0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9E,0x9F,0xA0,0xA1,0xA2,0xA3,
        0xA4,0xA5,0xA6,0xA7,0xA8,0xAD,0xB0,0xBE,0xBF };
    static const uint8_t known4[] = { 0x89,0xAA,0xB5,0xB6,0xB9 };

    const uint16_t payloadEnd = (uint16_t)(len - 9u);
    uint16_t i = 11u;
    while (i < payloadEnd) {
        const uint8_t id = buf[i];
        uint8_t dlen = 0u;
        if (id == 0x79u) {
            if ((uint16_t)(i + 2u) > payloadEnd) { goto stopped; }
            const uint8_t bl = buf[i + 1u];
            if ((uint16_t)(i + 2u + bl) > payloadEnd) { goto stopped; }
            if ((uint16_t)(bl / 3u) > JKP_CELLS_MAX) { return; }
            i = (uint16_t)(i + 2u + bl);
            continue;
        }
        for (size_t k = 0; k < sizeof known1; k++) { if (known1[k] == id) { dlen = 1u; } }
        for (size_t k = 0; k < sizeof known2; k++) { if (known2[k] == id) { dlen = 2u; } }
        for (size_t k = 0; k < sizeof known4; k++) { if (known4[k] == id) { dlen = 4u; } }
        if (id == 0xB4u) { dlen = 8u; }
        if (id == 0xB2u) { dlen = 10u; }
        if (id == 0xB7u) { dlen = 15u; }
        if (id == 0xBAu) { dlen = 24u; }
        if (dlen == 0u || (uint16_t)(i + 1u + dlen) > payloadEnd) { goto stopped; }
        i = (uint16_t)(i + 1u + dlen);
    }
    return;
stopped:
    {
        char det[256];
        snprintf(det, sizeof det,
                 "TLV walk stopped at payload offset %u of %u (unknown identifier 0x%02x "
                 "or a length that overruns the payload); JKP_Decode still returned true "
                 "with the remaining fields left at 0",
                 (unsigned)(i - 11u), (unsigned)(payloadEnd - 11u), buf[i]);
        report("truncated TLV walk reported as a good decode", pop, iter, buf, len, det);
    }
}

/* ------------------------------------------------------------ the driver */
static void runOne(const char *pop, unsigned long iter, const uint8_t *src, uint16_t len)
{
    /* exact-size heap copy: any read past len is an ASan heap-buffer-overflow */
    uint8_t *b = (uint8_t *)malloc(len ? len : 1u);
    if (len) { memcpy(b, src, len); }

    const bool valid = JKP_Validate(b, len);

    /* Contract probe: does a rejected frame leave *out alone? A caller that
       reuses one struct across polls would otherwise keep half-written data. */
    JK_Data_t marked;
    memset(&marked, 0x3C, sizeof marked);
    if (!JKP_Decode(b, len, &marked)) {
        JK_Data_t untouched;
        memset(&untouched, 0x3C, sizeof untouched);
        if (memcmp(&marked, &untouched, sizeof marked) != 0) {
            report("JKP_Decode==false still wrote to *out", pop, iter, b, len,
                   "the caller's JK_Data_t was modified by a rejected frame");
        }
    }

    JK_Data_t d1, d2;
    memset(&d1, 0xAA, sizeof d1);
    memset(&d2, 0x55, sizeof d2);
    const bool ok1 = JKP_Decode(b, len, &d1);
    const bool ok2 = JKP_Decode(b, len, &d2);

    /* determinism */
    if (ok1 != ok2 || (ok1 && memcmp(&d1, &d2, sizeof d1) != 0)) {
        report("JKP_Decode is not deterministic", pop, iter, b, len,
               "two identical calls produced different results");
    }
    /* Decode must never accept what Validate rejected */
    if (ok1 && !valid) {
        report("JKP_Decode accepted a frame JKP_Validate rejected", pop, iter, b, len, "");
    }
    if (ok1) {
        checkInvariants(pop, iter, b, len, &d1);
        checkDerivedFields(pop, iter, b, len, &d1);
        checkCellOverflowDropped(pop, iter, b, len);
        checkTruncatedWalkAccepted(pop, iter, b, len);
    }

    /* Decode must be memory-safe even reached out of order (a future refactor
       could call it without the validate gate). Exercise the walk directly on
       a frame whose header is forced valid but whose payload is the fuzz input. */
    if (!valid && len >= JKP_OVERHEAD_PROBE) {
        uint8_t *f = (uint8_t *)malloc(len);
        memcpy(f, b, len);
        f[0] = 0x4Eu; f[1] = 0x57u;
        f[2] = (uint8_t)(((len - 2u) >> 8) & 0xFFu);
        f[3] = (uint8_t)((len - 2u) & 0xFFu);
        f[len - 5u] = 0x68u;
        f[len - 4u] = 0x00u; f[len - 3u] = 0x00u;   /* the disabled CRC16 slot */
        fixSum(f, len);
        JK_Data_t d3;
        if (JKP_Decode(f, len, &d3)) {
            checkInvariants(pop, iter, f, len, &d3);
            checkDerivedFields(pop, iter, f, len, &d3);
            checkCellOverflowDropped(pop, iter, f, len);
        }
        free(f);
    }

    /* NULL handling must not fault */
    (void)JKP_Validate(NULL, len);
    (void)JKP_Decode(b, len, NULL);
    (void)JKP_Decode(NULL, len, &d1);

    free(b);
}

/* -------------------------------------------------- population 1: noise */
static void popRandom(unsigned long iters)
{
    uint8_t buf[601];
    for (unsigned long n = 0u; n < iters; n++) {
        const uint16_t len = (uint16_t)rnd(601u);
        for (uint16_t i = 0u; i < len; i++) { buf[i] = rndByte(); }
        /* half the time, pin the magic so more inputs survive to the walk */
        if (len >= 2u && (rnd64() & 1u)) { buf[0] = 0x4Eu; buf[1] = 0x57u; }
        runOne("1 random bytes", n, buf, len);
    }
}

/* ------------------------- population 2: valid frame, random TLV payload */
static void popValidRandomPayload(unsigned long iters)
{
    uint8_t pay[580], frame[601];
    for (unsigned long n = 0u; n < iters; n++) {
        const uint16_t pl = (uint16_t)rnd(400u);
        for (uint16_t i = 0u; i < pl; i++) { pay[i] = rndByte(); }
        const uint16_t len = buildFrame(frame, pay, pl,
                                        rndByte(), rndByte(), (uint8_t)rnd(3u));
        runOne("2 valid frame / random payload", n, frame, len);
    }
}

/* --------------------------- population 3: mutation of a known-good frame */
static void popMutateGood(unsigned long iters)
{
    uint8_t good[601], m[601];
    const uint16_t glen = goodFrame(good);

    for (unsigned long n = 0u; n < iters; n++) {
        memcpy(m, good, glen);
        uint16_t len = glen;
        const uint32_t kind = rnd(7u);

        switch (kind) {
        case 0: {                                        /* bit flips */
            const uint32_t nflip = 1u + rnd(6u);
            for (uint32_t f = 0u; f < nflip; f++) {
                const uint16_t pos = (uint16_t)rnd(len);
                m[pos] ^= (uint8_t)(1u << rnd(8u));
            }
            break;
        }
        case 1:                                          /* truncate */
            len = (uint16_t)(1u + rnd(glen));
            break;
        case 2:                                          /* extend */
            for (uint16_t i = len; i < len + 1u + rnd(40u) && i < 600u; i++) {
                m[i] = rndByte();
                len = (uint16_t)(i + 1u);
            }
            break;
        case 3:                                          /* corrupt checksum only */
            m[len - 1u] ^= (uint8_t)(1u + rnd(255u));
            break;
        case 4: {                                        /* corrupt a TLV length */
            m[11] = 0x79u;
            m[12] = rndByte();
            break;
        }
        case 5: {                                        /* corrupt LENGTH */
            m[2] = rndByte(); m[3] = rndByte();
            break;
        }
        default: {                                       /* random byte splat */
            const uint16_t pos = (uint16_t)rnd(len);
            m[pos] = rndByte();
            break;
        }
        }
        /* Half the mutants get a repaired sum so they reach the TLV walk. */
        if (kind != 3u && (rnd64() & 1u)) { fixSum(m, len); }
        runOne("3 mutated good frame", n, m, len);
    }
}

/* ---------------------- population 4: adversarial LENGTH and TLV lengths */
static const uint16_t kEdgeLen[] = {
    0u, 1u, 2u, 3u, 9u, 10u, 11u, 18u, 19u, 20u, 21u, 22u, 23u,
    0x00FFu, 0x0100u, 0x01FFu, 0x0200u, 0x0201u, 0x7FFFu, 0x8000u,
    0xFFF6u, 0xFFF7u, 0xFFFBu, 0xFFFCu, 0xFFFDu, 0xFFFEu, 0xFFFFu
};
static const uint8_t kEdgeTlv[] = {
    0u, 1u, 2u, 3u, 4u, 8u, 10u, 11u, 15u, 19u, 20u, 21u, 22u,
    62u, 63u, 64u, 65u, 66u, 0x7Fu, 0x80u, 0xFDu, 0xFEu, 0xFFu
};

static void popAdversarialLengths(unsigned long iters)
{
    uint8_t pay[600], frame[601];
    const size_t nL = sizeof kEdgeLen / sizeof kEdgeLen[0];
    const size_t nT = sizeof kEdgeTlv / sizeof kEdgeTlv[0];

    for (unsigned long n = 0u; n < iters; n++) {
        /* build a short frame then force LENGTH and the caller's len to edges */
        const uint16_t pl = (uint16_t)rnd(60u);
        for (uint16_t i = 0u; i < pl; i++) { pay[i] = rndByte(); }

        /* plant an edge-valued TLV length somewhere in the payload */
        if (pl >= 2u) {
            const uint16_t at = (uint16_t)rnd((uint32_t)(pl - 1u));
            pay[at]      = (rnd64() & 1u) ? 0x79u : (uint8_t)(0x80u + rnd(0x40u));
            pay[at + 1u] = kEdgeTlv[rnd((uint32_t)nT)];
        }
        uint16_t len = buildFrame(frame, pay, pl, 0x06u, 0x00u, 0x02u);

        const uint32_t mode = rnd(4u);
        if (mode == 1u) {                       /* LENGTH lies, caller tells truth */
            const uint16_t L = kEdgeLen[rnd((uint32_t)nL)];
            frame[2] = (uint8_t)(L >> 8); frame[3] = (uint8_t)(L & 0xFFu);
            fixSum(frame, len);
        } else if (mode == 2u) {                /* caller's len lies by +/-1..3 */
            const int32_t delta = (int32_t)rnd(7u) - 3;
            int32_t nl = (int32_t)len + delta;
            if (nl < 0) { nl = 0; }
            if (nl > 600) { nl = 600; }
            len = (uint16_t)nl;
        } else if (mode == 3u) {                /* both lie, LENGTH kept consistent
                                                   with a *different* len */
            const uint16_t L = kEdgeLen[rnd((uint32_t)nL)];
            frame[2] = (uint8_t)(L >> 8); frame[3] = (uint8_t)(L & 0xFFu);
            len = (uint16_t)(L + 2u);
            if (len > 600u) { len = 600u; }
            fixSum(frame, len);
        }
        runOne("4 adversarial LENGTH / TLV len", n, frame, len);
    }
}

/* ----------------------------------- population 5: 0x79 cell block abuse */
static void popCellBlocks(unsigned long iters)
{
    static const uint8_t kBlockLen[] = {
        0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u,
        59u, 60u, 61u, 62u, 63u, 64u, 65u, 66u, 67u, 68u,
        0x7Fu, 0x80u, 0xFDu, 0xFEu, 0xFFu
    };
    static const uint8_t kCellNo[] = { 0u, 1u, 2u, 20u, 21u, 22u, 23u, 0x7Fu, 0x80u, 0xFEu, 0xFFu };
    static const uint16_t kCount[] = { 0u, 1u, 20u, 21u, 22u, 255u, 256u, 0x7FFFu, 0xFFFFu };

    uint8_t pay[600], frame[601];
    const size_t nB = sizeof kBlockLen / sizeof kBlockLen[0];
    const size_t nC = sizeof kCellNo  / sizeof kCellNo[0];
    const size_t nK = sizeof kCount   / sizeof kCount[0];

    for (unsigned long n = 0u; n < iters; n++) {
        uint16_t pl = 0u;
        const uint8_t bl = kBlockLen[rnd((uint32_t)nB)];

        pay[pl++] = 0x79u;
        pay[pl++] = bl;
        /* Real block bytes: sometimes fewer than declared, so the declared
           length overruns what the payload actually carries. */
        const uint8_t actual = (rnd64() & 3u) ? bl : (uint8_t)rnd((uint32_t)bl + 1u);
        for (uint8_t g = 0u; g + 3u <= actual; g = (uint8_t)(g + 3u)) {
            const uint16_t mv = (uint16_t)rnd64();
            pay[pl++] = kCellNo[rnd((uint32_t)nC)];
            pay[pl++] = (uint8_t)(mv >> 8);
            pay[pl++] = (uint8_t)(mv & 0xFFu);
        }
        for (uint8_t r = (uint8_t)(actual - (uint8_t)(actual / 3u) * 3u); r > 0u; r--) {
            pay[pl++] = rndByte();
        }
        /* pair it with an 0x8a declared count, often disagreeing with the block */
        if (rnd64() & 1u) {
            const uint16_t c = kCount[rnd((uint32_t)nK)];
            pay[pl++] = 0x8Au;
            pay[pl++] = (uint8_t)(c >> 8);
            pay[pl++] = (uint8_t)(c & 0xFFu);
        }
        /* and some temperature / soc edges to exercise the scalar decoders */
        if (rnd64() & 1u) {
            pay[pl++] = 0x85u; pay[pl++] = rndByte();
            pay[pl++] = 0x80u; pay[pl++] = rndByte(); pay[pl++] = rndByte();
            pay[pl++] = 0x81u; pay[pl++] = rndByte(); pay[pl++] = rndByte();
            pay[pl++] = 0x8Cu; pay[pl++] = rndByte(); pay[pl++] = rndByte();
            pay[pl++] = 0xC0u; pay[pl++] = (uint8_t)rnd(3u);
            pay[pl++] = 0x84u; pay[pl++] = rndByte(); pay[pl++] = rndByte();
            pay[pl++] = 0xAAu;
            pay[pl++] = rndByte(); pay[pl++] = rndByte(); pay[pl++] = rndByte(); pay[pl++] = rndByte();
            pay[pl++] = 0xB9u;
            pay[pl++] = rndByte(); pay[pl++] = rndByte(); pay[pl++] = rndByte(); pay[pl++] = rndByte();
        }
        /* sometimes truncate the payload so the block's declared length
           overruns payloadEnd itself */
        uint16_t use = pl;
        if ((rnd64() & 7u) == 0u) { use = (uint16_t)rnd((uint32_t)pl + 1u); }

        const uint16_t len = buildFrame(frame, pay, use, 0x06u, 0x00u, 0x01u);
        runOne("5 0x79 cell blocks", n, frame, len);
    }
}

/* ---------------------------------------- JKP_BuildRequest, all 256 cmds */
#define GUARD_FILL 0xC7u
static void checkBuildRequest(void)
{
    for (unsigned c = 0u; c < 256u; c++) {
        uint8_t big[128];
        memset(big, GUARD_FILL, sizeof big);
        const uint16_t wrote = JKP_BuildRequest(big, (uint8_t)c);

        char det[256];
        if (wrote != JKP_REQUEST_LEN) {
            snprintf(det, sizeof det, "cmd=0x%02x returned %u, expected %u",
                     c, wrote, JKP_REQUEST_LEN);
            report("JKP_BuildRequest returned wrong length", "0 BuildRequest", c, big, 32, det);
        }
        for (size_t i = JKP_REQUEST_LEN; i < sizeof big; i++) {
            if (big[i] != GUARD_FILL) {
                snprintf(det, sizeof det,
                         "cmd=0x%02x wrote guard byte at offset %zu (0x%02x)", c, i, big[i]);
                report("JKP_BuildRequest wrote past JKP_REQUEST_LEN",
                       "0 BuildRequest", c, big, 32, det);
                break;
            }
        }
        /* the request it builds must satisfy its own validator */
        if (!JKP_Validate(big, JKP_REQUEST_LEN)) {
            snprintf(det, sizeof det, "cmd=0x%02x built a frame its own JKP_Validate rejects", c);
            report("JKP_BuildRequest output fails JKP_Validate",
                   "0 BuildRequest", c, big, JKP_REQUEST_LEN, det);
        }
        /* and it must decode as an all-zero JK_Data_t (spec 7.3) */
        JK_Data_t d;
        if (JKP_Decode(big, JKP_REQUEST_LEN, &d)) {
            JK_Data_t z;
            memset(&z, 0, sizeof z);
            if (memcmp(&d, &z, sizeof z) != 0) {
                snprintf(det, sizeof det, "cmd=0x%02x request decodes to non-zero data", c);
                report("JKP_BuildRequest output decodes to non-zero data",
                       "0 BuildRequest", c, big, JKP_REQUEST_LEN, det);
            }
        }
        /* an exact-size allocation catches a one-past write ASan would miss above */
        uint8_t *tight = (uint8_t *)malloc(JKP_REQUEST_LEN);
        (void)JKP_BuildRequest(tight, (uint8_t)c);
        free(tight);
    }
}

/* The good frame must decode to exactly the values the protocol encodes -
   a smoke oracle, so a harness that finds nothing is not finding nothing
   because it never reached the decoder. */
static void checkGoodFrameOracle(void)
{
    uint8_t good[601];
    const uint16_t len = goodFrame(good);
    JK_Data_t d;
    if (!JKP_Validate(good, len)) {
        report("synthesised good frame fails JKP_Validate", "0 oracle", 0, good, len, "");
        return;
    }
    if (!JKP_Decode(good, len, &d)) {
        report("synthesised good frame fails JKP_Decode", "0 oracle", 0, good, len, "");
        return;
    }
    char det[256];
    if (d.soc != 76u || d.cellCount != 21u || d.packCentivolts != 0x1D4Cu
        || d.cycles != 42u || d.protocolVersion != 1u
        || d.mosTempC != 30 || d.balTempC != -5 || d.soh != 90u
        || d.cellMillivolts[0] != 3601u || d.cellMillivolts[20] != 3621u) {
        snprintf(det, sizeof det,
                 "soc=%u cells=%u cV=%u cyc=%u ver=%u mos=%d bal=%d soh=%u c1=%u c21=%u",
                 d.soc, d.cellCount, d.packCentivolts, d.cycles, d.protocolVersion,
                 d.mosTempC, d.balTempC, d.soh, d.cellMillivolts[0], d.cellMillivolts[20]);
        report("good-frame oracle mismatch", "0 oracle", 0, good, len, det);
    }
    printf("oracle: good frame len=%u soc=%u cells=%u soh=%u mos=%d bal=%d "
           "cA=%d cV=%u cyc=%u ver=%u c1=%u c21=%u\n",
           len, d.soc, d.cellCount, d.soh, d.mosTempC, d.balTempC,
           d.packCentiamps, d.packCentivolts, d.cycles, d.protocolVersion,
           d.cellMillivolts[0], d.cellMillivolts[20]);
}

/* ------------------------------------ population 6: exhaustive sweeps */
/* Random sampling can miss a single arithmetic edge. These loops enumerate
   every value of each untrusted field the decoder branches on, so "no crash"
   is a statement about the whole domain of those fields, not a sample. */
static unsigned long popExhaustive(void)
{
    unsigned long n = 0u;
    uint8_t pay[600], frame[601];

    /* (a) every caller-declared length 0..600, header forced valid for that len */
    for (uint16_t L = 0u; L <= 600u; L++) {
        memset(frame, 0x5Au, sizeof frame);
        if (L >= 2u) { frame[0] = 0x4Eu; frame[1] = 0x57u; }
        if (L >= 4u) { frame[2] = (uint8_t)(((L - 2u) >> 8) & 0xFFu);
                       frame[3] = (uint8_t)((L - 2u) & 0xFFu); }
        if (L >= 5u) { frame[L - 5u] = 0x68u; fixSum(frame, L); }
        runOne("6 exhaustive", n++, frame, L);
    }

    /* (b) every identifier x every data length 0..28 x three fill patterns */
    static const uint8_t fills[] = { 0x00u, 0xFFu, 0x55u };
    for (unsigned id = 0u; id < 256u; id++) {
        for (unsigned dl = 0u; dl <= 28u; dl++) {
            for (unsigned f = 0u; f < 3u; f++) {
                uint16_t pl = 0u;
                pay[pl++] = (uint8_t)id;
                for (unsigned k = 0u; k < dl; k++) { pay[pl++] = fills[f]; }
                pay[pl++] = 0x85u; pay[pl++] = 50u;  /* a sentinel behind it */
                const uint16_t len = buildFrame(frame, pay, pl, 0x06u, 0x00u, 0x01u);
                runOne("6 exhaustive", n++, frame, len);
            }
        }
    }

    /* (c) 0x79: every declared blockLen 0..255 x every actual byte count 0..bl */
    for (unsigned bl = 0u; bl < 256u; bl++) {
        for (unsigned actual = 0u; actual <= bl; actual += (bl > 32u ? 7u : 1u)) {
            uint16_t pl = 0u;
            pay[pl++] = 0x79u;
            pay[pl++] = (uint8_t)bl;
            for (unsigned k = 0u; k < actual; k++) {
                pay[pl++] = (uint8_t)((k % 3u == 0u) ? (k / 3u + 1u) : 0x0Eu);
            }
            const uint16_t len = buildFrame(frame, pay, pl, 0x06u, 0x00u, 0x01u);
            runOne("6 exhaustive", n++, frame, len);
        }
    }

    /* (d) every 0x8a declared cell count 0..65535 */
    for (unsigned c = 0u; c < 65536u; c++) {
        const uint8_t p[3] = { 0x8Au, (uint8_t)(c >> 8), (uint8_t)(c & 0xFFu) };
        const uint16_t len = buildFrame(frame, p, 3u, 0x06u, 0x00u, 0x01u);
        runOne("6 exhaustive", n++, frame, len);
    }

    /* (e) every 0x80 / 0x81 temperature raw 0..65535 */
    for (unsigned t = 0u; t < 65536u; t++) {
        const uint8_t p[6] = { 0x80u, (uint8_t)(t >> 8), (uint8_t)(t & 0xFFu),
                               0x81u, (uint8_t)(t >> 8), (uint8_t)(t & 0xFFu) };
        const uint16_t len = buildFrame(frame, p, 6u, 0x06u, 0x00u, 0x01u);
        runOne("6 exhaustive", n++, frame, len);
    }

    /* (f) every 0x85 SOC byte, and every 0x8b / 0x8c raw */
    for (unsigned v = 0u; v < 65536u; v++) {
        const uint8_t p[8] = { 0x85u, (uint8_t)(v & 0xFFu),
                               0x8Bu, (uint8_t)(v >> 8), (uint8_t)(v & 0xFFu),
                               0x8Cu, (uint8_t)(v >> 8), (uint8_t)(v & 0xFFu) };
        const uint16_t len = buildFrame(frame, p, 8u, 0x06u, 0x00u, 0x01u);
        runOne("6 exhaustive", n++, frame, len);
    }

    /* (g) every 0x84 current raw x 0xc0 in {0, 1, 2} */
    for (unsigned ver = 0u; ver < 3u; ver++) {
        for (unsigned c = 0u; c < 65536u; c++) {
            const uint8_t p[5] = { 0xC0u, (uint8_t)ver,
                                   0x84u, (uint8_t)(c >> 8), (uint8_t)(c & 0xFFu) };
            const uint16_t len = buildFrame(frame, p, 5u, 0x06u, 0x00u, 0x01u);
            runOne("6 exhaustive", n++, frame, len);
        }
    }

    /* (h) 0xaa / 0xb9 capacity pairs around the SOH bounds and the wrap point */
    static const uint32_t caps[] = {
        0u, 1u, 2u, 99u, 100u, 99999u, 100000u, 100001u,
        0x00FFFFFFu, 0x01000000u, 0x028F5C28u, 0x028F5C29u,  /* 100*x wraps above here */
        0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFEu, 0xFFFFFFFFu };
    const size_t nCap = sizeof caps / sizeof caps[0];
    for (size_t a = 0u; a < nCap; a++) {
        for (size_t b = 0u; b < nCap; b++) {
            const uint8_t p[10] = {
                0xAAu, (uint8_t)(caps[a] >> 24), (uint8_t)(caps[a] >> 16),
                       (uint8_t)(caps[a] >> 8),  (uint8_t)caps[a],
                0xB9u, (uint8_t)(caps[b] >> 24), (uint8_t)(caps[b] >> 16),
                       (uint8_t)(caps[b] >> 8),  (uint8_t)caps[b] };
            const uint16_t len = buildFrame(frame, p, 10u, 0x06u, 0x00u, 0x01u);
            runOne("6 exhaustive", n++, frame, len);
        }
    }
    return n;
}

/* -------------------------------------------------- minimal reproducers */
/* One hand-built shortest frame per violation class, so the report carries an
   exact, self-verifying repro rather than a 600-byte random blob. */
static void showRepro(const char *what, const uint8_t *payload, uint16_t pl, const char *expect)
{
    uint8_t f[601];
    const uint16_t len = buildFrame(f, payload, pl, 0x06u, 0x00u, 0x02u);
    JK_Data_t d;
    memset(&d, 0, sizeof d);
    const bool v = JKP_Validate(f, len);
    const bool k = JKP_Decode(f, len, &d);
    printf("\n-- %s\n   frame (%u B): ", what, len);
    for (uint16_t i = 0u; i < len; i++) { printf("%02x ", f[i]); }
    printf("\n   Validate=%s Decode=%s\n", v ? "true" : "FALSE", k ? "true" : "FALSE");
    printf("   soc=%u soh=%u cells=%u mos=%d bal=%d mode=0x%02x status=0x%02x "
           "cV=%u cA=%d cyc=%u ver=%u cell1=%u cell21=%u\n",
           d.soc, d.soh, d.cellCount, d.mosTempC, d.balTempC, d.modeFlags,
           d.statusFlags, d.packCentivolts, d.packCentiamps, d.cycles,
           d.protocolVersion, d.cellMillivolts[0], d.cellMillivolts[20]);
    printf("   expected: %s\n", expect);
}

static void minimalReproducers(void)
{
    printf("\n==================== minimal reproducers ====================");

    { const uint8_t p[] = { 0x85u, 0xFFu };
      showRepro("R1 SOC unclamped (0x85)", p, sizeof p,
                "reject or clamp: SOC is a percent, 0..100"); }

    { const uint8_t p[] = { 0x80u, 0x00u, 0xE4u };            /* raw 228 */
      showRepro("R2 mos temperature narrowing (0x80 raw=228)", p, sizeof p,
                "raw>200 is outside the 0x80 encoding; got -128 C"); }

    { const uint8_t p[] = { 0x81u, 0x7Fu, 0xFFu };            /* raw 32767 */
      showRepro("R3 bal temperature narrowing (0x81 raw=32767)", p, sizeof p,
                "raw 32767 should be rejected, not folded into the +/-128 C range"); }

    { const uint8_t p[] = { 0x8Cu, 0x00u, 0xFFu };
      showRepro("R4 modeFlags undefined bits (0x8c)", p, sizeof p,
                "spec 7.4: only bits 0..3 are defined"); }

    { const uint8_t p[] = { 0x79u, 0x03u, 22u, 0x0Eu, 0x10u };
      showRepro("R5 0x79 cell number 22 silently dropped", p, sizeof p,
                "spec 7.5: a cell above 21 must raise a fault; all 21 cells read 0 mV "
                "and Decode says true"); }

    { const uint8_t p[] = { 0x79u, 0x03u, 0x00u, 0x0Eu, 0x10u };
      showRepro("R6 0x79 cell number 0 silently dropped", p, sizeof p,
                "cell 0 does not exist; frame accepted with no cells"); }

    { const uint8_t p[] = { 0x79u, 0x01u, 0x00u };
      showRepro("R7 0x79 blockLen=1 (not a multiple of 3) accepted", p, sizeof p,
                "spec 7.1: 3 bytes per cell; a partial group is a malformed block"); }

    { uint8_t p[68]; uint16_t n = 0u;
      p[n++] = 0x79u; p[n++] = 65u;
      for (uint8_t c = 1u; c <= 21u; c++) { p[n++] = c; p[n++] = 0x0Eu; p[n++] = 0x10u; }
      p[n++] = 0xAAu; p[n++] = 0xBBu;                     /* 2 trailing bytes */
      showRepro("R8 0x79 blockLen=65 passes the /3 <= 21 gate", p, n,
                "65/3 == 21 so the gate passes and 2 bytes are dropped silently"); }

    { const uint8_t p[] = { 0x88u, 0x85u, 99u, 0x8Au, 0x00u, 21u };
      showRepro("R9 unknown identifier truncates the walk, Decode still true", p, sizeof p,
                "0x88 has no table entry, so SOC=99 and cellCount=21 that follow it are "
                "lost and published as 0 - the same all-zero hazard spec 7.3 guards "
                "against for the activation reply"); }

    { const uint8_t p[] = { 0x85u, 0x63u, 0xC0u };
      showRepro("R10 payload ends mid-TLV, Decode still true", p, sizeof p,
                "trailing 0xc0 has no room for its 1 data byte; walk breaks, true returned"); }

    { const uint8_t p[] = { 0xC0u, 0x00u, 0x84u, 0xFFu, 0xFFu };
      showRepro("R11 0x84 v0 encoding wraps int16 (raw=65535)", p, sizeof p,
                "10000 - 65535 = -55535, negated = 55535, does not fit int16"); }

    { const uint8_t p[] = { 0x85u, 0x64u };
      showRepro("R12 control: SOC=100 accepted (sanity)", p, sizeof p, "soc=100, no violation"); }

    /* A rejected frame that has already written to *out. */
    { uint8_t f[64]; const uint8_t p[] = { 0x85u, 0x5Au, 0x8Au, 0x00u, 22u };
      const uint16_t len = buildFrame(f, p, sizeof p, 0x06u, 0x00u, 0x02u);
      JK_Data_t d; memset(&d, 0x3Cu, sizeof d);
      const bool k = JKP_Decode(f, len, &d);
      printf("\n-- R15 rejected frame still scribbled on the caller's struct\n   frame (%u B): ", len);
      for (uint16_t i = 0u; i < len; i++) { printf("%02x ", f[i]); }
      printf("\n   Decode=%s (0x8a count 22 > 21, correctly rejected), but *out was "
             "memset and partially filled first: soc=%u (0x3c == 60 would mean "
             "untouched)\n", k ? "true" : "false", d.soc); }

    { uint8_t f[64]; uint8_t p[70]; uint16_t n = 0u;
      p[n++] = 0x85u; p[n++] = 0x5Au;
      p[n++] = 0x79u; p[n++] = 66u;                   /* 66/3 == 22 > 21 */
      for (uint8_t c = 0u; c < 22u; c++) { p[n++] = (uint8_t)(c + 1u); p[n++] = 0x0Eu; p[n++] = 0x10u; }
      uint8_t big[128];
      const uint16_t len = buildFrame(big, p, n, 0x06u, 0x00u, 0x02u);
      (void)f;
      JK_Data_t d; memset(&d, 0x3Cu, sizeof d);
      const bool k = JKP_Decode(big, len, &d);
      printf("\n-- R16 same via the 0x79 blockLen gate (blockLen=66)\n"
             "   Decode=%s, soc=%u (0x3c == 60 would mean untouched), len=%u\n",
             k ? "true" : "false", d.soc, len); }

    /* Non-zero CRC16 slot: spec 7.1 says those 2 bytes are zero. */
    { uint8_t f[64]; const uint8_t p[] = { 0x85u, 0x32u };
      const uint16_t len = buildFrame(f, p, sizeof p, 0x06u, 0x00u, 0x02u);
      f[len - 4u] = 0xDEu; f[len - 3u] = 0xADu;   /* not covered by the sum */
      JK_Data_t d;
      printf("\n-- R13 non-zero disabled-CRC16 slot accepted\n   frame (%u B): ", len);
      for (uint16_t i = 0u; i < len; i++) { printf("%02x ", f[i]); }
      printf("\n   Validate=%s Decode=%s (spec 7.1 declares those 2 bytes zero; they sit "
             "outside the checksum, so unchecked they are 16 malleable bits)\n",
             JKP_Validate(f, len) ? "true" : "false",
             JKP_Decode(f, len, &d) ? "true" : "false"); }

    /* Byte-swap malleability of the accumulated sum. */
    { uint8_t f[64]; const uint8_t p[] = { 0x85u, 0x32u, 0x87u, 0x00u, 0x2Au };
      const uint16_t len = buildFrame(f, p, sizeof p, 0x06u, 0x00u, 0x02u);
      JK_Data_t a, b2;
      (void)JKP_Decode(f, len, &a);
      const uint8_t t = f[15]; f[15] = f[16]; f[16] = t;   /* swap two payload bytes */
      printf("\n-- R14 accumulated sum is order-blind\n   swapped two payload bytes; "
             "Validate=%s, cycles %u -> %u\n",
             JKP_Validate(f, len) ? "still true" : "false", a.cycles,
             JKP_Decode(f, len, &b2) ? b2.cycles : 0u); }
}

int main(int argc, char **argv)
{
    unsigned long iters = 400000u;
    g_seed = 0xC0FFEEu;

    for (int a = 1; a < argc; a++) {
        if (strcmp(argv[a], "-x") == 0)      { g_abortFirst = 1; }
        else if (iters == 400000u && a == 1) { iters = strtoul(argv[a], NULL, 0); }
        else                                 { g_seed = strtoull(argv[a], NULL, 0); }
    }
    g_rngState = g_seed ? g_seed : 1u;

    printf("fuzz_jk: seed=%llu iterations/population=%lu  (reproduce: "
           "./fuzz_jk %lu %llu)\n",
           (unsigned long long)g_seed, iters, iters, (unsigned long long)g_seed);
    printf("sizeof(JK_Data_t)=%zu sizeof(int)=%zu sizeof(void*)=%zu\n",
           sizeof(JK_Data_t), sizeof(int), sizeof(void *));

    checkGoodFrameOracle();
    checkBuildRequest();
    minimalReproducers();
    printf("\n");
    printf("pop 0 BuildRequest              : 256 cmds\n");

    popRandom(iters);
    printf("pop 1 random bytes              : %lu\n", iters);
    popValidRandomPayload(iters);
    printf("pop 2 valid frame/random payload: %lu\n", iters);
    popMutateGood(iters);
    printf("pop 3 mutated good frame        : %lu\n", iters);
    popAdversarialLengths(iters);
    printf("pop 4 adversarial lengths       : %lu\n", iters);
    popCellBlocks(iters);
    printf("pop 5 0x79 cell blocks          : %lu\n", iters);
    { const unsigned long ex = popExhaustive();
      printf("pop 6 exhaustive field sweeps   : %lu\n", ex); }

    if (g_violN == 0) {
        printf("\nno invariant violations\n");
        return 0;
    }
    printf("\n==================== %d violation class(es), seed=%llu "
           "====================\n", g_violN, (unsigned long long)g_seed);
    for (int i = 0; i < g_violN; i++) {
        printf("\n[%d] %s   (hit %lu times)\n%s",
               i + 1, g_viol[i].name, g_viol[i].count, g_viol[i].witness);
    }
    return 1;
}

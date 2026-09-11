#include "jk_protocol.h"
#include <string.h>

#define JKP_OVERHEAD      (20u)   /* everything but the TLV payload */
#define JKP_PAYLOAD_START (11u)
#define JKP_END_FLAG      (0x68u)
#define JKP_SRC_PC        (0x03u) /* frame source: PC upper computer */
/* Sanity bound on untrusted 0xaa/0xb9 capacity registers (Ah), not a
   calibration limit - real packs are two orders of magnitude below this;
   this only guards the SOH multiply against a corrupt-but-checksummed frame. */
#define JKP_CAPACITY_MAX_AH (100000u)

/* Publishing bounds: the narrower of the JK register range and the CAN_DB
   signal range. A value outside them is a corrupt register, not a reading,
   so the frame is rejected rather than clamped into something plausible. */
#define JKP_TEMP_RAW_MAX    (140u)    /* 0x80/0x81, 140 = -40 degC            */
#define JKP_SOC_MAX_PCT     (100u)    /* 0x85                                 */
#define JKP_CENTIVOLTS_MAX  (10000u)  /* 0x83, BMSMaster_JK_PackVoltage       */
#define JKP_CENTIAMPS_MAX   (30000)   /* 0x84, BMSMaster_JK_PackCurrent       */
#define JKP_STRINGS_MIN     (3u)      /* 0x8a, protocol range is 3..32        */
#define JKP_MODE_FLAGS_MASK (0x0Fu)   /* 0x8c, bits 4..15 are reserved        */
#define JKP_SOH_MAX_PCT     (110u)    /* slightly over rated is real, far over is not */

/*
 * Data length for each identifier, excluding the identifier byte itself.
 * 0 means "unknown", which rejects the frame: without a length there is no way
 * to find the next identifier. Older firmware omits fields, so a payload that
 * ends early is normal; one that stops mid-walk is not.
 */
static uint8_t identLen(uint8_t ident)
{
    switch (ident) {
    case 0x85u: case 0x86u: case 0x9Du: case 0xA9u: case 0xABu: case 0xACu:
    case 0xAEu: case 0xAFu: case 0xB1u: case 0xB3u: case 0xB8u: case 0xBBu:
    case 0xBCu: case 0xBDu: case 0xC0u:
        return 1u;
    case 0x80u: case 0x81u: case 0x82u: case 0x83u: case 0x84u: case 0x87u:
    case 0x8Au: case 0x8Bu: case 0x8Cu: case 0x8Eu: case 0x8Fu: case 0x90u:
    case 0x91u: case 0x92u: case 0x93u: case 0x94u: case 0x95u: case 0x96u:
    case 0x97u: case 0x98u: case 0x99u: case 0x9Au: case 0x9Bu: case 0x9Cu:
    case 0x9Eu: case 0x9Fu: case 0xA0u: case 0xA1u: case 0xA2u: case 0xA3u:
    case 0xA4u: case 0xA5u: case 0xA6u: case 0xA7u: case 0xA8u: case 0xADu:
    case 0xB0u: case 0xBEu: case 0xBFu:
        return 2u;
    case 0x89u: case 0xAAu: case 0xB5u: case 0xB6u: case 0xB9u:
        return 4u;
    case 0xB4u: return 8u;
    case 0xB2u: return 10u;
    case 0xB7u: return 15u;
    case 0xBAu: return 24u;
    default:    return 0u;
    }
}

static uint16_t be16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }
static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

/* 0-100 is positive degC directly; 101-140 is negative, 101 = -1. Any higher
   raw is undefined and would truncate into int8_t, half of it as a positive. */
static bool decodeTemp(uint16_t raw, int8_t *out)
{
    if (raw > JKP_TEMP_RAW_MAX) {
        return false;
    }
    *out = (raw > 100u) ? (int8_t)-(int16_t)(raw - 100u) : (int8_t)raw;
    return true;
}

/*
 * 0x8b defines 14 warning bits but JK_StatusFlags is only 8, and truncating
 * to the low byte would drop b10 and b11 - monomer over- and under-voltage,
 * the cell-level protections that matter most. Fold by category so nothing
 * safety-relevant is lost.
 */
static uint8_t curateWarnings(uint16_t w)
{
    uint8_t f = 0u;
    if (w & ((1u << 2) | (1u << 10)))              { f |= 0x01u; }  /* over-voltage      */
    if (w & ((1u << 3) | (1u << 11)))              { f |= 0x02u; }  /* under-voltage     */
    if (w & ((1u << 1) | (1u << 4) | (1u << 8)))   { f |= 0x04u; }  /* over-temperature  */
    if (w & (1u << 9))                             { f |= 0x08u; }  /* low temperature   */
    if (w & ((1u << 5) | (1u << 6)))               { f |= 0x10u; }  /* over-current      */
    if (w & (1u << 7))                             { f |= 0x20u; }  /* cell pressure diff */
    if (w & (1u << 0))                             { f |= 0x40u; }  /* low capacity      */
    if (w & ((1u << 12) | (1u << 13)))             { f |= 0x80u; }  /* 309_A / 309_B     */
    return f;
}

uint16_t JKP_BuildRequest(uint8_t *out, uint8_t cmd)
{
    memset(out, 0, JKP_REQUEST_LEN);
    out[0]  = 0x4Eu;
    out[1]  = 0x57u;
    out[2]  = 0x00u;
    out[3]  = (uint8_t)(JKP_REQUEST_LEN - 2u);   /* LENGTH includes itself and the checksum */
    out[8]  = cmd;
    out[9]  = JKP_SRC_PC;
    out[10] = 0x00u;                             /* transmission type: request */
    out[11] = 0x00u;                             /* identifier 0 = read all    */
    out[16] = JKP_END_FLAG;

    uint16_t sum = 0u;
    for (uint16_t i = 0u; i <= 16u; i++) { sum = (uint16_t)(sum + out[i]); }
    out[17] = 0u;                                /* CRC16 slot, disabled */
    out[18] = 0u;
    out[19] = (uint8_t)(sum >> 8);
    out[20] = (uint8_t)(sum & 0xFFu);
    return JKP_REQUEST_LEN;
}

bool JKP_Validate(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len < JKP_OVERHEAD) {
        return false;
    }
    if (buf[0] != 0x4Eu || buf[1] != 0x57u) {
        return false;
    }
    if ((uint16_t)(be16(&buf[2]) + 2u) != len) {
        return false;
    }
    if (buf[len - 5u] != JKP_END_FLAG) {
        return false;
    }
    /* The CRC16 slot is disabled and declared zero. It sits outside the sum,
       so leaving it unchecked is 16 freely malleable bits. */
    if (buf[len - 4u] != 0u || buf[len - 3u] != 0u) {
        return false;
    }

    uint16_t sum = 0u;
    for (uint16_t i = 0u; i <= (uint16_t)(len - 5u); i++) {
        sum = (uint16_t)(sum + buf[i]);
    }
    return be16(&buf[len - 2u]) == sum;
}

bool JKP_Decode(const uint8_t *buf, uint16_t len, JK_Data_t *out)
{
    if (out == NULL || !JKP_Validate(buf, len)) {
        return false;
    }

    /* Filled locally and published only on success, so a rejected frame
       leaves the caller's struct alone. */
    JK_Data_t d;
    memset(&d, 0, sizeof d);

    uint32_t capacitySet = 0u, capacityActual = 0u;
    bool haveSet = false, haveActual = false;
    uint16_t currentRaw = 0u;
    bool haveCurrent = false;

    const uint16_t payloadEnd = (uint16_t)(len - 9u);   /* record number starts here */
    uint16_t i = JKP_PAYLOAD_START;

    while (i < payloadEnd) {
        const uint8_t ident = buf[i];

        if (ident == 0x79u) {                    /* length-prefixed cell block */
            if ((uint16_t)(i + 2u) > payloadEnd) { return false; }
            const uint8_t blockLen = buf[i + 1u];
            if ((uint16_t)(i + 2u + blockLen) > payloadEnd) { return false; }
            /* 3 bytes per cell and a pack is 21S. A partial group, an
               over-long block or a cell number outside 1..21 means cells the
               vehicle would never see, so reject the frame rather than
               silently drop them - 0 mV is also the link-down state. */
            if ((blockLen % 3u) != 0u || (blockLen / 3u) > JKP_CELLS_MAX) {
                return false;
            }
            for (uint8_t g = 0u; (uint16_t)(g + 3u) <= blockLen; g = (uint8_t)(g + 3u)) {
                const uint8_t cellNo = buf[i + 2u + g];
                if (cellNo < 1u || cellNo > JKP_CELLS_MAX) { return false; }
                d.cellMillivolts[cellNo - 1u] = be16(&buf[i + 3u + g]);
            }
            i = (uint16_t)(i + 2u + blockLen);
            continue;
        }

        const uint8_t dlen = identLen(ident);
        if (dlen == 0u || (uint16_t)(i + 1u + dlen) > payloadEnd) {
            /* Unknown identifier, or one whose data runs past the payload: the
               walk cannot continue, and every field behind it would publish 0.
               0 mV is the link-down state, so that reads as a dead link on a
               frame we just called good. */
            return false;
        }
        const uint8_t *v = &buf[i + 1u];

        switch (ident) {
        case 0x80u: if (!decodeTemp(be16(v), &d.mosTempC)) { return false; } break;
        case 0x81u: if (!decodeTemp(be16(v), &d.balTempC)) { return false; } break;
        case 0x83u: {
            const uint16_t centivolts = be16(v);
            if (centivolts > JKP_CENTIVOLTS_MAX) { return false; }
            d.packCentivolts = centivolts;
            break;
        }
        /* 0.01 A since protocol V20200508; older JK firmware reports 0.1 A - a silent 10x. */
        case 0x84u: currentRaw = be16(v); haveCurrent = true; break;
        case 0x85u:
            if (v[0] > JKP_SOC_MAX_PCT) { return false; }
            d.soc = v[0];
            break;
        case 0x87u: d.cycles = be16(v); break;
        case 0x8Au: {
            /* Protocol range is 3..32 strings; over 21 means cells the vehicle
               cannot see (spec 7.5). Either way the frame is rejected. */
            const uint16_t count = be16(v);
            if (count < JKP_STRINGS_MIN || count > JKP_CELLS_MAX) { return false; }
            d.cellCount = (uint8_t)count;
            break;
        }
        case 0x8Bu: d.statusFlags = curateWarnings(be16(v)); break;
        case 0x8Cu: d.modeFlags = (uint8_t)(be16(v) & JKP_MODE_FLAGS_MASK); break;
        case 0xAAu: capacitySet = be32(v); haveSet = true; break;
        case 0xB9u: capacityActual = be32(v); haveActual = true; break;
        case 0xC0u: d.protocolVersion = v[0]; break;
        default: break;                          /* known length, not needed */
        }
        i = (uint16_t)(i + 1u + dlen);
    }

    /*
     * Current, negated so positive means DISCHARGING - matching the CAN
     * signal and the Hall sensor, so the two current sources can be compared
     * directly. The JK itself uses the opposite sign.
     * 0xc0 selects between the two encodings and is indistinguishable at low
     * current, which is why it must be parsed before this runs.
     * Computed in int32_t: the offset encoding reaches +55535 centiamps, which
     * wraps to a charging current if narrowed straight to int16_t.
     */
    if (haveCurrent) {
        int32_t centiamps;
        if (d.protocolVersion == 0x01u) {
            const int32_t mag = (int32_t)(currentRaw & 0x7FFFu);
            centiamps = (currentRaw & 0x8000u) ? -mag : mag;
        } else {
            centiamps = (int32_t)currentRaw - 10000;
        }
        if (centiamps < -JKP_CENTIAMPS_MAX || centiamps > JKP_CENTIAMPS_MAX) {
            return false;
        }
        d.packCentiamps = (int16_t)centiamps;
    }

    /* The protocol has no SOH register; this is the only health figure it exposes.
       Bound both operands first: an unbounded capacityActual*100u can wrap
       uint32_t and land <=100 after the clamp below, publishing a
       plausible-but-wrong SOH instead of an obvious failure. */
    if (haveSet && haveActual && capacitySet > 0u
        && capacitySet <= JKP_CAPACITY_MAX_AH && capacityActual <= JKP_CAPACITY_MAX_AH) {
        const uint32_t soh = (capacityActual * 100u + capacitySet / 2u) / capacitySet;  /* rounded */
        /* A little over rated is a real measurement and clamps to 100; far over
           is a corrupt register, and clamping it would publish perfect health. */
        if (soh > JKP_SOH_MAX_PCT) { return false; }
        d.soh = (uint8_t)((soh > 100u) ? 100u : soh);
    }

    *out = d;
    return true;
}

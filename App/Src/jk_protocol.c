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

/*
 * Data length for each identifier, excluding the identifier byte itself.
 * 0 means "unknown", which stops the walk: without a length there is no way
 * to find the next identifier. Older firmware omits fields, so a short
 * payload is normal, not an error.
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

/* 0-100 is positive degC directly; above 100 the value is negative, 101 = -1. */
static int8_t decodeTemp(uint16_t raw)
{
    return (raw > 100u) ? (int8_t)-(int16_t)(raw - 100u) : (int8_t)raw;
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
    memset(out, 0, sizeof *out);

    uint32_t capacitySet = 0u, capacityActual = 0u;
    bool haveSet = false, haveActual = false;
    uint16_t currentRaw = 0u;
    bool haveCurrent = false;

    const uint16_t payloadEnd = (uint16_t)(len - 9u);   /* record number starts here */
    uint16_t i = JKP_PAYLOAD_START;

    while (i < payloadEnd) {
        const uint8_t ident = buf[i];

        if (ident == 0x79u) {                    /* length-prefixed cell block */
            if ((uint16_t)(i + 2u) > payloadEnd) { break; }
            const uint8_t blockLen = buf[i + 1u];
            if ((uint16_t)(i + 2u + blockLen) > payloadEnd) { break; }
            /* A pack is 21S; a reported count above that is a fault - those
               cells would be invisible to the vehicle, so reject the frame
               rather than silently drop the overflow. */
            if ((uint16_t)(blockLen / 3u) > JKP_CELLS_MAX) {
                return false;
            }
            for (uint8_t g = 0u; (uint16_t)(g + 3u) <= blockLen; g = (uint8_t)(g + 3u)) {
                const uint8_t cellNo = buf[i + 2u + g];
                if (cellNo >= 1u && cellNo <= JKP_CELLS_MAX) {
                    out->cellMillivolts[cellNo - 1u] = be16(&buf[i + 3u + g]);
                }
            }
            i = (uint16_t)(i + 2u + blockLen);
            continue;
        }

        const uint8_t dlen = identLen(ident);
        if (dlen == 0u || (uint16_t)(i + 1u + dlen) > payloadEnd) {
            break;                               /* unknown length: cannot continue */
        }
        const uint8_t *v = &buf[i + 1u];

        switch (ident) {
        case 0x80u: out->mosTempC = decodeTemp(be16(v)); break;
        case 0x81u: out->balTempC = decodeTemp(be16(v)); break;
        case 0x83u: out->packCentivolts = be16(v); break;
        case 0x84u: currentRaw = be16(v); haveCurrent = true; break;
        case 0x85u: out->soc = v[0]; break;
        case 0x87u: out->cycles = be16(v); break;
        case 0x8Au: out->cellCount = (uint8_t)be16(v); break;
        case 0x8Bu: out->statusFlags = curateWarnings(be16(v)); break;
        case 0x8Cu: out->modeFlags = (uint8_t)(be16(v) & 0xFFu); break;
        case 0xAAu: capacitySet = be32(v); haveSet = true; break;
        case 0xB9u: capacityActual = be32(v); haveActual = true; break;
        case 0xC0u: out->protocolVersion = v[0]; break;
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
     */
    if (haveCurrent) {
        if (out->protocolVersion == 0x01u) {
            const int16_t mag = (int16_t)(currentRaw & 0x7FFFu);
            out->packCentiamps = (currentRaw & 0x8000u) ? (int16_t)-mag : mag;
        } else {
            out->packCentiamps = (int16_t)((int32_t)currentRaw - 10000);
        }
    }

    /* The protocol has no SOH register; this is the only health figure it exposes.
       Bound both operands first: an unbounded capacityActual*100u can wrap
       uint32_t and land <=100 after the clamp below, publishing a
       plausible-but-wrong SOH instead of an obvious failure. */
    if (haveSet && haveActual && capacitySet > 0u
        && capacitySet <= JKP_CAPACITY_MAX_AH && capacityActual <= JKP_CAPACITY_MAX_AH) {
        uint32_t soh = (capacityActual * 100u + capacitySet / 2u) / capacitySet;  /* rounded */
        if (soh > 100u) { soh = 100u; }
        out->soh = (uint8_t)soh;
    }
    return true;
}

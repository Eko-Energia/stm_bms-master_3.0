#include "test_runner.h"
#include "jk_protocol.h"
#include <string.h>

/* Build a well-formed response carrying the given TLV payload. */
static uint16_t makeResponse(uint8_t *buf, const uint8_t *payload, uint16_t payloadLen)
{
    uint16_t n = 0;
    buf[n++] = 0x4Eu; buf[n++] = 0x57u;
    const uint16_t total = (uint16_t)(20u + payloadLen);
    buf[n++] = (uint8_t)((total - 2u) >> 8);        /* LENGTH, big-endian */
    buf[n++] = (uint8_t)((total - 2u) & 0xFFu);
    buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u;   /* terminal id */
    buf[n++] = 0x06u;                                /* command word */
    buf[n++] = 0x00u;                                /* frame source: BMS */
    buf[n++] = 0x01u;                                /* transmission type: reply */
    memcpy(&buf[n], payload, payloadLen); n = (uint16_t)(n + payloadLen);
    buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u;   /* record number */
    buf[n++] = 0x68u;                                /* end flag */
    uint16_t sum = 0u;
    for (uint16_t i = 0; i < n; i++) { sum = (uint16_t)(sum + buf[i]); }
    buf[n++] = 0u; buf[n++] = 0u;                    /* CRC16 slot, disabled */
    buf[n++] = (uint8_t)(sum >> 8);
    buf[n++] = (uint8_t)(sum & 0xFFu);
    return n;
}

TEST(read_all_request_matches_the_documented_bytes)
{
    uint8_t req[JKP_REQUEST_LEN];
    CHECK_EQ(JKP_BuildRequest(req, JKP_CMD_READ_ALL), JKP_REQUEST_LEN);

    const uint8_t expect[JKP_REQUEST_LEN] = {
        0x4Eu, 0x57u, 0x00u, 0x13u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x06u, 0x03u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x68u, 0x00u, 0x00u, 0x01u, 0x29u
    };
    CHECK_EQ(memcmp(req, expect, JKP_REQUEST_LEN), 0);
}

TEST(activation_request_differs_only_in_command_and_checksum)
{
    uint8_t req[JKP_REQUEST_LEN];
    JKP_BuildRequest(req, JKP_CMD_ACTIVATE);
    CHECK_EQ(req[8], 0x01u);
    CHECK_EQ(req[19], 0x01u);
    CHECK_EQ(req[20], 0x24u);       /* 0x129 - 6 + 1 = 0x124 */
}

TEST(validate_rejects_malformed_frames)
{
    uint8_t buf[64];
    const uint8_t payload[3] = { 0x85u, 0x00u, 0x55u };
    uint16_t len = makeResponse(buf, payload, sizeof payload);
    CHECK(JKP_Validate(buf, len));

    buf[0] = 0x00u;  CHECK(!JKP_Validate(buf, len));            /* bad magic */
    buf[0] = 0x4Eu;
    CHECK(!JKP_Validate(buf, (uint16_t)(len - 1u)));            /* length mismatch */
    buf[len - 5u] = 0x00u; CHECK(!JKP_Validate(buf, len));      /* bad end flag */
    buf[len - 5u] = 0x68u;
    buf[len - 1u] ^= 0xFFu; CHECK(!JKP_Validate(buf, len));     /* bad checksum */
    CHECK(!JKP_Validate(buf, 10u));                             /* too short */
}

TEST(pack_voltage_and_soc_decode_directly)
{
    uint8_t buf[64];
    /* 0x83 total voltage 10 mV/LSB: 6800 = 68.00 V. 0x85 SOC = 85 %. */
    const uint8_t payload[6] = { 0x83u, 0x1Au, 0x90u, 0x85u, 0x55u, 0x00u };
    const uint16_t len = makeResponse(buf, payload, 5u);
    (void)payload;
    JK_Data_t d;
    CHECK(JKP_Decode(buf, len, &d));
    CHECK_EQ(d.packCentivolts, 6800u);
    CHECK_EQ(d.soc, 85u);
}

TEST(current_offset_encoding_is_negated_to_positive_equals_discharging)
{
    uint8_t buf[64];
    JK_Data_t d;

    /* 0xC0 = 0: JK reports 11000 for 10 A discharge. Positive-discharging
       means we must publish +1000 centiamps. */
    const uint8_t discharge[6] = { 0xC0u, 0x00u, 0x84u, 0x2Au, 0xF8u, 0x00u };
    CHECK(JKP_Decode(buf, makeResponse(buf, discharge, 5u), &d));
    CHECK_EQ(d.packCentiamps, 1000);

    /* JK reports 9500 for 5 A charge -> -500 centiamps. */
    const uint8_t charge[6] = { 0xC0u, 0x00u, 0x84u, 0x25u, 0x1Cu, 0x00u };
    CHECK(JKP_Decode(buf, makeResponse(buf, charge, 5u), &d));
    CHECK_EQ(d.packCentiamps, -500);
}

TEST(current_sign_bit_encoding_is_also_negated)
{
    uint8_t buf[64];
    JK_Data_t d;

    /* 0xC0 = 1: 0x07D0 = 2000 with bit15 clear = 20 A discharge -> +2000. */
    const uint8_t discharge[6] = { 0xC0u, 0x01u, 0x84u, 0x07u, 0xD0u, 0x00u };
    CHECK(JKP_Decode(buf, makeResponse(buf, discharge, 5u), &d));
    CHECK_EQ(d.packCentiamps, 2000);

    /* 0x87D0 = bit15 set = 20 A charge -> -2000. */
    const uint8_t charge[6] = { 0xC0u, 0x01u, 0x84u, 0x87u, 0xD0u, 0x00u };
    CHECK(JKP_Decode(buf, makeResponse(buf, charge, 5u), &d));
    CHECK_EQ(d.packCentiamps, -2000);
}

TEST(cell_voltages_walk_the_length_prefixed_block)
{
    uint8_t buf[128];
    /* 0x79, length 9 = three cells, each {cellNo, mV big-endian}. */
    const uint8_t payload[11] = {
        0x79u, 0x09u,
        0x01u, 0x0Cu, 0xE4u,        /* cell 1 = 3300 mV */
        0x02u, 0x0Cu, 0xE5u,        /* cell 2 = 3301 mV */
        0x15u, 0x0Cu, 0xE6u         /* cell 21 = 3302 mV */
    };
    JK_Data_t d;
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.cellMillivolts[0], 3300u);
    CHECK_EQ(d.cellMillivolts[1], 3301u);
    CHECK_EQ(d.cellMillivolts[20], 3302u);
}

TEST(cell_voltages_accept_a_single_cell)
{
    uint8_t buf[64];
    /* 0x79, length 3 = one cell only; the rest of the array stays 0. */
    const uint8_t payload[5] = { 0x79u, 0x03u, 0x01u, 0x0Cu, 0xE4u };
    JK_Data_t d;
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.cellMillivolts[0], 3300u);
    CHECK_EQ(d.cellMillivolts[1], 0u);
    CHECK_EQ(d.cellMillivolts[20], 0u);
}

TEST(cell_voltages_accept_the_full_21_cell_pack)
{
    uint8_t buf[128];
    /* 0x79 length 63 = the full 21S pack; every slot must land at its own index. */
    uint8_t payload[2 + 21 * 3];
    payload[0] = 0x79u;
    payload[1] = (uint8_t)(21u * 3u);
    for (uint8_t c = 0u; c < 21u; c++) {
        payload[2u + c * 3u] = (uint8_t)(c + 1u);
        payload[3u + c * 3u] = 0x0Cu;
        payload[4u + c * 3u] = (uint8_t)(0xE4u + c);
    }
    JK_Data_t d;
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    for (uint8_t c = 0u; c < 21u; c++) {
        CHECK_EQ(d.cellMillivolts[c], (uint16_t)(3300u + c));
    }
}

TEST(cell_voltages_reject_a_count_above_the_series_taps)
{
    uint8_t buf[128];
    /* 0x79 length 66 = 22 entries. A pack is 21S: 22 reported cells means one
       would be invisible to the vehicle, so the whole frame is rejected. */
    uint8_t payload[2 + 22 * 3];
    payload[0] = 0x79u;
    payload[1] = (uint8_t)(22u * 3u);
    for (uint8_t c = 0u; c < 22u; c++) {
        payload[2u + c * 3u] = (uint8_t)(c + 1u);
        payload[3u + c * 3u] = 0x0Cu;
        payload[4u + c * 3u] = (uint8_t)(0xE4u + c);
    }
    JK_Data_t d;
    CHECK(!JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
}

TEST(a_reported_cell_count_above_the_series_taps_rejects_the_frame)
{
    uint8_t buf[64];
    JK_Data_t d;

    /* 0x8a is the JK's own runtime count and is published straight to frame
       148. 21 is the pack, so 21 decodes and 24 is a fault (spec 7.5) - three
       cells the vehicle would look for and never find. */
    uint8_t payload[3] = { 0x8Au, 0x00u, 21u };
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.cellCount, 21u);

    payload[2] = 24u;
    CHECK(!JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));

    /* A count that only overflows the uint8_t truncation must not slip past. */
    payload[1] = 0x01u; payload[2] = 0x15u;              /* 277 -> 21 if truncated */
    CHECK(!JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));

    /* Below 21 stays legal: the absent cells publish 0 mV. */
    payload[1] = 0x00u; payload[2] = 16u;
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.cellCount, 16u);
}

TEST(temperatures_use_the_offset_above_one_hundred_encoding)
{
    uint8_t buf[64];
    /* 0x80 = 45 -> +45 degC. 0x81 = 105 -> -5 degC. */
    const uint8_t payload[6] = { 0x80u, 0x00u, 0x2Du, 0x81u, 0x00u, 0x69u };
    JK_Data_t d;
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.mosTempC, 45);
    CHECK_EQ(d.balTempC, -5);
}

TEST(soh_is_derived_from_actual_over_configured_capacity)
{
    uint8_t buf[64];
    /* 0xaa capacity setting = 100 Ah, 0xb9 actual = 90 Ah -> 90 %. */
    const uint8_t payload[10] = {
        0xAAu, 0x00u, 0x00u, 0x00u, 0x64u,
        0xB9u, 0x00u, 0x00u, 0x00u, 0x5Au
    };
    JK_Data_t d;
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.soh, 90u);
}

TEST(soh_stays_at_default_when_capacity_setting_is_zero)
{
    uint8_t buf[64];
    /* 0xaa capacity setting = 0 is a divide-by-zero guard, not a real reading;
       SOH must stay at its zero default rather than derive from garbage. */
    const uint8_t payload[10] = {
        0xAAu, 0x00u, 0x00u, 0x00u, 0x00u,
        0xB9u, 0x00u, 0x00u, 0x00u, 0x5Au
    };
    JK_Data_t d;
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.soh, 0u);
}

TEST(warning_bits_fold_into_the_curated_summary)
{
    uint8_t buf[64];
    JK_Data_t d;

    /* b10 monomer over-voltage must survive: it would be lost by truncation. */
    const uint8_t monomerOv[3] = { 0x8Bu, 0x04u, 0x00u };
    CHECK(JKP_Decode(buf, makeResponse(buf, monomerOv, sizeof monomerOv), &d));
    CHECK_EQ(d.statusFlags & 0x01u, 0x01u);

    /* b11 monomer under-voltage -> bit 1. */
    const uint8_t monomerUv[3] = { 0x8Bu, 0x08u, 0x00u };
    CHECK(JKP_Decode(buf, makeResponse(buf, monomerUv, sizeof monomerUv), &d));
    CHECK_EQ(d.statusFlags & 0x02u, 0x02u);

    /* b1 MOS over-temperature -> bit 2. */
    const uint8_t mosOt[3] = { 0x8Bu, 0x00u, 0x02u };
    CHECK(JKP_Decode(buf, makeResponse(buf, mosOt, sizeof mosOt), &d));
    CHECK_EQ(d.statusFlags & 0x04u, 0x04u);
}

TEST(an_unknown_identifier_rejects_the_frame)
{
    uint8_t buf[64];
    JK_Data_t d;

    /* 0x7B has no documented length, so the walk cannot continue past it and
       every field behind it would publish 0 - which for cells is also the
       link-down state. A frame that cannot be read whole is not trustworthy. */
    const uint8_t payload[6] = { 0x85u, 0x42u, 0x7Bu, 0x00u, 0x00u, 0x00u };
    CHECK(!JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));

    /* The captured frame: 0x88 hides SOC 99 and cellCount 21 behind it. */
    const uint8_t captured[26] = {
        0x4Eu, 0x57u, 0x00u, 0x18u, 0x00u, 0x00u, 0x00u, 0x00u, 0x06u,
        0x00u, 0x02u, 0x88u, 0x85u, 0x63u, 0x8Au, 0x00u, 0x15u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x68u, 0x00u, 0x00u, 0x03u, 0x3Cu
    };
    CHECK(JKP_Validate(captured, sizeof captured));
    CHECK(!JKP_Decode(captured, sizeof captured, &d));
}

TEST(a_payload_ending_mid_identifier_rejects_the_frame)
{
    uint8_t buf[64];
    /* The trailing 0xC0 has no room for its data byte. */
    const uint8_t payload[3] = { 0x85u, 0x63u, 0xC0u };
    JK_Data_t d;
    CHECK(!JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
}

TEST(a_cell_number_outside_the_pack_rejects_the_frame)
{
    uint8_t buf[64];
    JK_Data_t d;
    /* Cell 0 does not exist and cell 22 is past a 21S pack. Dropping either
       silently reports 21 cells at 0 mV as a good decode. */
    const uint8_t zero[5] = { 0x79u, 0x03u, 0x00u, 0x0Cu, 0xE4u };
    CHECK(!JKP_Decode(buf, makeResponse(buf, zero, sizeof zero), &d));
    const uint8_t over[5] = { 0x79u, 0x03u, 22u, 0x0Cu, 0xE4u };
    CHECK(!JKP_Decode(buf, makeResponse(buf, over, sizeof over), &d));
}

TEST(a_cell_block_length_that_is_not_a_multiple_of_three_rejects_the_frame)
{
    uint8_t buf[128];
    JK_Data_t d;
    /* 64 and 65 both pass a /3 <= 21 gate and then drop the trailing bytes. */
    uint8_t payload[2u + 65u];
    payload[0] = 0x79u;
    for (uint8_t bl = 64u; bl <= 65u; bl++) {
        payload[1] = bl;
        for (uint8_t k = 0u; k < bl; k++) {
            payload[2u + k] = (uint8_t)((k % 3u == 0u) ? (k / 3u + 1u) : 0x0Eu);
        }
        CHECK(!JKP_Decode(buf, makeResponse(buf, payload, (uint16_t)(2u + bl)), &d));
    }
    const uint8_t tiny[3] = { 0x79u, 0x01u, 0x00u };
    CHECK(!JKP_Decode(buf, makeResponse(buf, tiny, sizeof tiny), &d));
}

TEST(a_temperature_raw_outside_the_encoding_rejects_the_frame)
{
    uint8_t buf[64];
    JK_Data_t d;
    /* 140 is the lowest defined raw, -40 degC. */
    const uint8_t edge[6] = { 0x80u, 0x00u, 140u, 0x81u, 0x00u, 140u };
    CHECK(JKP_Decode(buf, makeResponse(buf, edge, sizeof edge), &d));
    CHECK_EQ(d.mosTempC, -40);
    CHECK_EQ(d.balTempC, -40);

    /* Undefined raws: 229 truncates to +127 degC and 65535 to +101 degC. */
    const uint8_t over[3]  = { 0x80u, 0x00u, 141u };
    const uint8_t trunc[3] = { 0x80u, 0x00u, 0xE5u };
    const uint8_t wide[3]  = { 0x81u, 0xFFu, 0xFFu };
    CHECK(!JKP_Decode(buf, makeResponse(buf, over, sizeof over), &d));
    CHECK(!JKP_Decode(buf, makeResponse(buf, trunc, sizeof trunc), &d));
    CHECK(!JKP_Decode(buf, makeResponse(buf, wide, sizeof wide), &d));
}

TEST(a_current_outside_the_pack_range_rejects_the_frame)
{
    uint8_t buf[64];
    JK_Data_t d;
    /* Offset encoding: raw 40000 is exactly +300.00 A, the sensor's limit. */
    const uint8_t edge[5] = { 0xC0u, 0x00u, 0x84u, 0x9Cu, 0x40u };
    CHECK(JKP_Decode(buf, makeResponse(buf, edge, sizeof edge), &d));
    CHECK_EQ(d.packCentiamps, 30000);

    /* raw 0xffff is +55535 centiamps; narrowed to int16_t it publishes
       -10001, a 100 A discharge reported as a charge. */
    const uint8_t wrap[5] = { 0xC0u, 0x00u, 0x84u, 0xFFu, 0xFFu };
    CHECK(!JKP_Decode(buf, makeResponse(buf, wrap, sizeof wrap), &d));

    /* Sign-bit encoding: 32767 fits int16_t but not a +/-300 A pack. */
    const uint8_t big[5] = { 0xC0u, 0x01u, 0x84u, 0x7Fu, 0xFFu };
    CHECK(!JKP_Decode(buf, makeResponse(buf, big, sizeof big), &d));
}

TEST(a_state_of_charge_above_one_hundred_rejects_the_frame)
{
    uint8_t buf[64];
    JK_Data_t d;
    const uint8_t full[2] = { 0x85u, 100u };
    CHECK(JKP_Decode(buf, makeResponse(buf, full, sizeof full), &d));
    CHECK_EQ(d.soc, 100u);
    const uint8_t over[2] = { 0x85u, 0xFFu };
    CHECK(!JKP_Decode(buf, makeResponse(buf, over, sizeof over), &d));
}

TEST(mode_flags_keep_only_the_four_defined_bits)
{
    uint8_t buf[64];
    JK_Data_t d;
    /* 0x8c defines charge MOS, discharge MOS, balancing and battery dropped;
       bits 4..15 are reserved and must not reach the bus. */
    const uint8_t payload[3] = { 0x8Cu, 0x00u, 0xFFu };
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.modeFlags, 0x0Fu);
}

TEST(a_pack_voltage_beyond_the_can_signal_rejects_the_frame)
{
    uint8_t buf[64];
    JK_Data_t d;
    const uint8_t edge[3] = { 0x83u, 0x27u, 0x10u };    /* 100.00 V, the signal max */
    CHECK(JKP_Decode(buf, makeResponse(buf, edge, sizeof edge), &d));
    CHECK_EQ(d.packCentivolts, 10000u);
    const uint8_t over[3] = { 0x83u, 0xFFu, 0xFFu };    /* 655.35 V */
    CHECK(!JKP_Decode(buf, makeResponse(buf, over, sizeof over), &d));
}

TEST(a_string_count_below_the_protocol_minimum_rejects_the_frame)
{
    uint8_t buf[64];
    JK_Data_t d;
    uint8_t payload[3] = { 0x8Au, 0x00u, 3u };          /* the register is 3..32 */
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.cellCount, 3u);
    payload[2] = 2u;
    CHECK(!JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    payload[2] = 0u;
    CHECK(!JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
}

TEST(a_rejected_frame_leaves_the_callers_struct_alone)
{
    uint8_t buf[64];
    JK_Data_t d, untouched;
    memset(&d, 0x3Cu, sizeof d);
    memset(&untouched, 0x3Cu, sizeof untouched);
    /* 0x8a = 22 is rejected, but only after the walk has published SOC. */
    const uint8_t payload[5] = { 0x85u, 0x5Au, 0x8Au, 0x00u, 22u };
    CHECK(!JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(memcmp(&d, &untouched, sizeof d), 0);
}

TEST(an_absurd_capacity_pair_is_rejected_instead_of_clamped)
{
    uint8_t buf[64];
    JK_Data_t d;
    /* 105 Ah against 100 Ah rated is a real new-pack reading, so it clamps. */
    uint8_t payload[10] = {
        0xAAu, 0x00u, 0x00u, 0x00u, 100u,
        0xB9u, 0x00u, 0x00u, 0x00u, 105u
    };
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.soh, 100u);

    /* 218 Ah against 100 Ah rated is not health, and soh=100 would hide it. */
    payload[9] = 218u;
    CHECK(!JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
}

TEST(a_non_zero_disabled_crc16_slot_is_rejected)
{
    uint8_t buf[64];
    const uint8_t payload[2] = { 0x85u, 0x32u };
    const uint16_t len = makeResponse(buf, payload, sizeof payload);
    CHECK(JKP_Validate(buf, len));
    /* Those 2 bytes are declared zero and sit outside the sum: left unchecked
       they are 16 freely malleable bits in an already weak checksum. */
    buf[len - 4u] = 0xDEu;
    CHECK(!JKP_Validate(buf, len));
    buf[len - 4u] = 0x00u; buf[len - 3u] = 0xADu;
    CHECK(!JKP_Validate(buf, len));
    JK_Data_t d;
    CHECK(!JKP_Decode(buf, len, &d));
}

TEST(a_byte_swap_the_order_blind_sum_cannot_see_is_caught_by_the_range_gate)
{
    uint8_t buf[64];
    JK_Data_t d;
    /* The accumulated sum ignores byte order, so a swap inside the payload
       keeps the frame valid. Only the value gate can catch it. */
    const uint8_t payload[4] = { 0x85u, 0x64u, 0x86u, 0x02u };
    const uint16_t len = makeResponse(buf, payload, sizeof payload);
    CHECK(JKP_Decode(buf, len, &d));
    CHECK_EQ(d.soc, 100u);

    const uint8_t t = buf[12]; buf[12] = buf[13]; buf[13] = t;
    CHECK(JKP_Validate(buf, len));
    CHECK(!JKP_Decode(buf, len, &d));       /* SOC would have been 134 */
}

TEST(unsolicited_type_0x02_frame_still_decodes)
{
    uint8_t buf[64];
    uint16_t n = 0;
    buf[n++] = 0x4Eu; buf[n++] = 0x57u;
    const uint8_t payload[2] = { 0x85u, 0x32u };
    const uint16_t total = (uint16_t)(20u + sizeof payload);
    buf[n++] = (uint8_t)((total - 2u) >> 8);
    buf[n++] = (uint8_t)((total - 2u) & 0xFFu);
    buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u;
    buf[n++] = 0x06u;
    buf[n++] = 0x00u;
    buf[n++] = 0x02u;                                /* transmission type: unsolicited */
    memcpy(&buf[n], payload, sizeof payload); n = (uint16_t)(n + sizeof payload);
    buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u;
    buf[n++] = 0x68u;
    uint16_t sum = 0u;
    for (uint16_t i = 0; i < n; i++) { sum = (uint16_t)(sum + buf[i]); }
    buf[n++] = 0u; buf[n++] = 0u;
    buf[n++] = (uint8_t)(sum >> 8);
    buf[n++] = (uint8_t)(sum & 0xFFu);

    CHECK(JKP_Validate(buf, n));
    JK_Data_t d;
    CHECK(JKP_Decode(buf, n, &d));
    CHECK_EQ(d.soc, 0x32u);
}

TEST(zero_length_frame_is_rejected)
{
    uint8_t buf[64];
    CHECK(!JKP_Validate(buf, 0u));
    JK_Data_t d;
    CHECK(!JKP_Decode(buf, 0u, &d));
}

TEST(truncated_frame_is_rejected)
{
    uint8_t buf[64];
    const uint8_t payload[3] = { 0x85u, 0x00u, 0x55u };
    uint16_t len = makeResponse(buf, payload, sizeof payload);
    /* Cut the frame short: fewer bytes were actually received than LENGTH claims. */
    CHECK(!JKP_Validate(buf, (uint16_t)(len - 10u)));
}

TEST(length_field_overrunning_the_buffer_is_rejected)
{
    uint8_t buf[64];
    const uint8_t payload[3] = { 0x85u, 0x00u, 0x55u };
    uint16_t len = makeResponse(buf, payload, sizeof payload);
    /* LENGTH claims a much larger frame than what actually arrived. */
    buf[2] = 0xFFu; buf[3] = 0xFFu;
    CHECK(!JKP_Validate(buf, len));
}

int main(void)
{
    RUN(read_all_request_matches_the_documented_bytes);
    RUN(activation_request_differs_only_in_command_and_checksum);
    RUN(validate_rejects_malformed_frames);
    RUN(pack_voltage_and_soc_decode_directly);
    RUN(current_offset_encoding_is_negated_to_positive_equals_discharging);
    RUN(current_sign_bit_encoding_is_also_negated);
    RUN(cell_voltages_walk_the_length_prefixed_block);
    RUN(cell_voltages_accept_a_single_cell);
    RUN(cell_voltages_accept_the_full_21_cell_pack);
    RUN(cell_voltages_reject_a_count_above_the_series_taps);
    RUN(a_reported_cell_count_above_the_series_taps_rejects_the_frame);
    RUN(temperatures_use_the_offset_above_one_hundred_encoding);
    RUN(soh_is_derived_from_actual_over_configured_capacity);
    RUN(soh_stays_at_default_when_capacity_setting_is_zero);
    RUN(warning_bits_fold_into_the_curated_summary);
    RUN(an_unknown_identifier_rejects_the_frame);
    RUN(a_payload_ending_mid_identifier_rejects_the_frame);
    RUN(a_cell_number_outside_the_pack_rejects_the_frame);
    RUN(a_cell_block_length_that_is_not_a_multiple_of_three_rejects_the_frame);
    RUN(a_temperature_raw_outside_the_encoding_rejects_the_frame);
    RUN(a_current_outside_the_pack_range_rejects_the_frame);
    RUN(a_state_of_charge_above_one_hundred_rejects_the_frame);
    RUN(mode_flags_keep_only_the_four_defined_bits);
    RUN(a_pack_voltage_beyond_the_can_signal_rejects_the_frame);
    RUN(a_string_count_below_the_protocol_minimum_rejects_the_frame);
    RUN(a_rejected_frame_leaves_the_callers_struct_alone);
    RUN(an_absurd_capacity_pair_is_rejected_instead_of_clamped);
    RUN(a_non_zero_disabled_crc16_slot_is_rejected);
    RUN(a_byte_swap_the_order_blind_sum_cannot_see_is_caught_by_the_range_gate);
    RUN(unsolicited_type_0x02_frame_still_decodes);
    RUN(zero_length_frame_is_rejected);
    RUN(truncated_frame_is_rejected);
    RUN(length_field_overrunning_the_buffer_is_rejected);
    return TEST_SUMMARY();
}

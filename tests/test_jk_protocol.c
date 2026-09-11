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

TEST(an_unknown_identifier_stops_the_walk_without_corrupting_earlier_fields)
{
    uint8_t buf[64];
    /* 0x7B has no documented length, so the walk cannot continue past it. */
    const uint8_t payload[6] = { 0x85u, 0x42u, 0x7Bu, 0x00u, 0x00u, 0x00u };
    JK_Data_t d;
    CHECK(JKP_Decode(buf, makeResponse(buf, payload, sizeof payload), &d));
    CHECK_EQ(d.soc, 0x42u);
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
    RUN(temperatures_use_the_offset_above_one_hundred_encoding);
    RUN(soh_is_derived_from_actual_over_configured_capacity);
    RUN(soh_stays_at_default_when_capacity_setting_is_zero);
    RUN(warning_bits_fold_into_the_curated_summary);
    RUN(an_unknown_identifier_stops_the_walk_without_corrupting_earlier_fields);
    RUN(unsolicited_type_0x02_frame_still_decodes);
    RUN(zero_length_frame_is_rejected);
    RUN(truncated_frame_is_rejected);
    RUN(length_field_overrunning_the_buffer_is_rejected);
    return TEST_SUMMARY();
}

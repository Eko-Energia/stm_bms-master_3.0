#include "test_runner.h"
#include "app_adc.h"
#include "app_can.h"
#include "app_therm.h"
#include "app_jk.h"
#include "bms_calib.h"
#include "bms_errors.h"
#include "error_handler.h"
#include "CAN_DB.h"
#include <stdio.h>
#include <string.h>

/*
 * Dumps every frame app_can.c transmits, packed from known, non-trivial
 * inputs, to oracle/vectors.txt. tests/oracle/check_packing.py decodes each
 * line with cantools against the pinned DBC - an independent check that our
 * packing agrees with the database, not just with this file's own arithmetic.
 */

static CAN_InstanceTypeDef i1, i2;
static CAN_HandleTypeDef h1 = { &i1, { DISABLE } }, h2 = { &i2, { DISABLE } };
static UART_InstanceTypeDef uinst;
static UART_HandleTypeDef huart = { &uinst };
static EH_HandleTypeDef eh;
static volatile uint16_t adcBuf[3];

/* Appends a big-endian TLV field: 1 id byte + len value bytes. */
static uint16_t putField(uint8_t *buf, uint16_t n, uint8_t ident, uint32_t v, uint8_t len)
{
    buf[n++] = ident;
    for (uint8_t k = len; k > 0u; k--) { buf[n++] = (uint8_t)(v >> (8u * (k - 1u))); }
    return n;
}

/* Builds a well-formed JK read-all response with distinct, non-symmetric,
   sign-exercising values in every field app_can.c relays onto CAN. */
static uint16_t makeJkResponse(uint8_t *buf)
{
    uint8_t payload[128];
    uint16_t p = 0;
    p = putField(payload, p, 0xC0u, 0x01u, 1u);        /* protocol: sign-bit current */
    p = putField(payload, p, 0x84u, 0x87D0u, 2u);      /* raw 2000, sign bit set -> charging */
    p = putField(payload, p, 0x83u, 7256u, 2u);        /* 72.56 V */
    p = putField(payload, p, 0x85u, 77u, 1u);           /* SOC 77% */
    p = putField(payload, p, 0xAAu, 100u, 4u);          /* capacity setting 100 Ah */
    p = putField(payload, p, 0xB9u, 88u, 4u);           /* capacity actual 88 Ah -> SOH 88% */
    p = putField(payload, p, 0x8Au, 21u, 2u);           /* cell count 21 */
    p = putField(payload, p, 0x8Bu, 0x0004u, 2u);       /* monomer OV -> statusFlags bit0 */
    p = putField(payload, p, 0x8Cu, 0x00ABu, 2u);       /* modeFlags 0xab -> 0x0b, b4+ reserved */
    p = putField(payload, p, 0x80u, 45u, 2u);           /* mosTemp +45 C */
    p = putField(payload, p, 0x81u, 105u, 2u);          /* balTemp -5 C (offset-above-100) */
    p = putField(payload, p, 0x87u, 1234u, 2u);         /* 1234 cycles */

    payload[p++] = 0x79u;                               /* cell block: all 21 taps, distinct mV */
    payload[p++] = (uint8_t)(21u * 3u);
    for (uint8_t c = 0u; c < 21u; c++) {
        payload[p++] = (uint8_t)(c + 1u);
        const uint16_t mv = (uint16_t)(3300u + (c * 7u));
        payload[p++] = (uint8_t)(mv >> 8);
        payload[p++] = (uint8_t)(mv & 0xFFu);
    }

    uint16_t n = 0;
    buf[n++] = 0x4Eu; buf[n++] = 0x57u;
    const uint16_t total = (uint16_t)(20u + p);
    buf[n++] = (uint8_t)((total - 2u) >> 8);
    buf[n++] = (uint8_t)((total - 2u) & 0xFFu);
    buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u;
    buf[n++] = 0x06u; buf[n++] = 0x00u; buf[n++] = 0x01u;
    memcpy(&buf[n], payload, p); n = (uint16_t)(n + p);
    buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u; buf[n++] = 0u;
    buf[n++] = 0x68u;
    uint16_t sum = 0u;
    for (uint16_t i = 0; i < n; i++) { sum = (uint16_t)(sum + buf[i]); }
    buf[n++] = 0u; buf[n++] = 0u;
    buf[n++] = (uint8_t)(sum >> 8);
    buf[n++] = (uint8_t)(sum & 0xFFu);
    return n;
}

TEST(dump_vectors_for_the_oracle)
{
    Fake_Reset();
    i1.MCR = 0; i2.MCR = 0;
    memset(&eh, 0, sizeof eh);
    CAN_App_Init(&h1, &h2, &eh);
    EH_init(&eh, &h1, BMSMASTER_NODE_FRAME_ID, CAN_App_Scheduler());
    ADC_Init(adcBuf, &eh);
    THERM_Init(&eh);
    JK_Init(&huart, &eh);

    /* An active fault so the NODE frame's Error_Code is non-zero. Not one of
       the codes app_adc.c/app_therm.c/app_jk.c auto-clear on a healthy
       reading, or this test's own valid inputs would clear it right back. */
    const uint8_t blob[5] = { 0x34u, 0x12u, 0u, 0u, 0u };
    EH_reportEx(&eh, BMS_ERR_FATAL_INIT, ERROR_SEVERITY_ERROR, blob, 2u);

    /* Master voltage/current/temperature, from calibrated ADC counts. */
    for (int i = 0; i < 10; i++) {
        adcBuf[0] = calibNtcCount[37]; adcBuf[1] = 2208u; adcBuf[2] = 3200u;
        ADC_OnConvComplete();
        ADC_Task(0u);
    }

    /* Nine thermistor frames x seven modules, every (module, therm) distinct. */
    for (int p = 0; p < 10; p++) {
        for (uint32_t id = 211u; id <= 279u; id++) {
            if ((id % 10u) == 0u) { continue; }
            THERM_OnFrame(id, (uint8_t)(60u + (id % 7u)));
        }
        THERM_Task();
    }

    /* The boot poll is JKP_CMD_ACTIVATE and its reply carries no data, so it
       is acknowledged and discarded; the read-all a second later is what
       publishes. Same all-zero ack frame the transport test uses. */
    uint8_t ack[20];
    memset(ack, 0, sizeof ack);
    ack[0] = 0x4Eu; ack[1] = 0x57u; ack[3] = 0x12u;
    ack[8] = JKP_CMD_ACTIVATE; ack[10] = 0x01u; ack[15] = 0x68u;
    uint16_t ackSum = 0u;
    for (uint16_t i = 0; i <= 15u; i++) { ackSum = (uint16_t)(ackSum + ack[i]); }
    ack[18] = (uint8_t)(ackSum >> 8); ack[19] = (uint8_t)(ackSum & 0xFFu);

    Fake_SetTick(0u);
    JK_Task(0u);
    Fake_QueueUartRx(ack, sizeof ack);
    JK_OnTxComplete();
    JK_OnRxEvent((uint16_t)sizeof ack);
    JK_Task(0u);
    CHECK(!JK_Valid());                 /* an activation ack is not a reading */

    /* One full JK exchange with a crafted, non-trivial response: negative
       pack current, distinct multi-byte cell voltages, signed temperatures. */
    Fake_SetTick(1000u);
    JK_Task(1000u);
    uint8_t reply[200];
    const uint16_t replyLen = makeJkResponse(reply);
    Fake_QueueUartRx(reply, replyLen);
    JK_OnTxComplete();
    JK_OnRxEvent(replyLen);
    JK_Task(1000u);
    CHECK(JK_Valid());

    for (uint32_t t = 0u; t <= 6000u; t += 100u) {
        Fake_SetTick(t);
        CAN_App_Task();
    }

    FILE *f = fopen("oracle/vectors.txt", "w");
    CHECK(f != NULL);
    const uint32_t ids[] = {
        BMSMASTER_NODE_FRAME_ID,
        BMSMASTER_MASTERVOLTCURRTEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM1TEMP_FRAME_ID, BMSMASTER_PCBSTHERM2TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM3TEMP_FRAME_ID, BMSMASTER_PCBSTHERM4TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM5TEMP_FRAME_ID, BMSMASTER_PCBSTHERM6TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM7TEMP_FRAME_ID, BMSMASTER_PCBSTHERM8TEMP_FRAME_ID,
        BMSMASTER_PCBSTHERM9TEMP_FRAME_ID,
        BMSMASTER_JK_PACK_FRAME_ID,
        BMSMASTER_JK_CELLS_1_4_FRAME_ID, BMSMASTER_JK_CELLS_5_8_FRAME_ID,
        BMSMASTER_JK_CELLS_9_12_FRAME_ID, BMSMASTER_JK_CELLS_13_16_FRAME_ID,
        BMSMASTER_JK_CELLS_17_20_FRAME_ID, BMSMASTER_JK_CELLS_21_FRAME_ID,
        BMSMASTER_JK_TEMP_FRAME_ID, BMSMASTER_JK_CYCLESTATS_FRAME_ID,
        BMSMASTER_END_FRAME_ID
    };
    /* Every frame app_can.c schedules, plus the NODE frame - not a sample. */
    CHECK_EQ(sizeof ids / sizeof ids[0], 21u);
    for (size_t k = 0; k < sizeof ids / sizeof ids[0]; k++) {
        uint8_t d[8];
        CHECK(Fake_FindTx(ids[k], d, NULL));
        fprintf(f, "%03X ", (unsigned)ids[k]);
        for (int b = 0; b < 8; b++) { fprintf(f, "%02X", d[b]); }
        fprintf(f, "\n");
    }
    fclose(f);

    /* Echo the values the oracle must reproduce. */
    printf("  voltage=%u dV current=%d dA temp=%u cdegC\n",
           ADC_PackDecivolts(), ADC_PackDeciamps(), ADC_TempCenti());
    printf("  jk pack=%u cV current=%d cA soc=%u soh=%u cycles=%u\n",
           JK_Data()->packCentivolts, JK_Data()->packCentiamps,
           JK_Data()->soc, JK_Data()->soh, JK_Data()->cycles);
}

int main(void)
{
    RUN(dump_vectors_for_the_oracle);
    return TEST_SUMMARY();
}

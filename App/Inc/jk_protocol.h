#ifndef JK_PROTOCOL_H
#define JK_PROTOCOL_H
#include <stdbool.h>
#include <stdint.h>

#define JKP_REQUEST_LEN   (21u)
#define JKP_RX_BUF_LEN    (512u)    /* worst-case response is about 339 bytes */
#define JKP_CELLS_MAX     (21u)     /* the pack is 21S */

#define JKP_CMD_ACTIVATE  (0x01u)
#define JKP_CMD_READ_ALL  (0x06u)

/** Decoded JK state. Scales match the CAN_DB signals so packing is a copy. */
typedef struct {
    uint16_t packCentivolts;                 /* 0x83, 10 mV/LSB           */
    int16_t  packCentiamps;                  /* 0x84, positive = DISCHARGING */
    uint8_t  soc;                            /* 0x85, %                   */
    uint8_t  soh;                            /* derived from 0xb9 / 0xaa  */
    uint8_t  statusFlags;                    /* curated summary of 0x8b   */
    uint8_t  modeFlags;                      /* 0x8c low byte             */
    uint16_t cellMillivolts[JKP_CELLS_MAX];  /* 0x79                      */
    uint8_t  cellCount;                      /* 0x8a, true runtime count  */
    int8_t   mosTempC;                       /* 0x80                      */
    int8_t   balTempC;                       /* 0x81, battery-box         */
    uint16_t cycles;                         /* 0x87                      */
    uint8_t  protocolVersion;                /* 0xc0, selects 0x84 encoding */
} JK_Data_t;

/** @brief Build a request. Returns bytes written, always JKP_REQUEST_LEN. */
uint16_t JKP_BuildRequest(uint8_t *out, uint8_t cmd);

/** @brief Check magic, LENGTH, end flag and the accumulated checksum. */
bool JKP_Validate(const uint8_t *buf, uint16_t len);

/** @brief Walk the TLV payload into out. Validates first. */
bool JKP_Decode(const uint8_t *buf, uint16_t len, JK_Data_t *out);

#endif /* JK_PROTOCOL_H */

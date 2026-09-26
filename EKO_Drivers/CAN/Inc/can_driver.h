/**
 * @file can_driver.h
 * @brief CAN bus driver for PERLA
 * @author AGH EKO-ENERGIA
 * @author Kacper Lasota
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "can_id_list.h"
#include "main.h"
#include <stdio.h>
#include "string.h"

/**
 * Defines
 */

#define CAN_MAX_DLC (8)
#define CAN_MAX_MSG (28)                    /* 21 CAN1 TX frames plus headroom */

/* bxCAN on STM32F105 has 28 filter banks shared between the two peripherals.
   Banks below CAN_SLAVE_START_FILTER_BANK belong to CAN1, at or above it to
   CAN2. Programming a CAN2 filter into a CAN1 bank silently does nothing. */
#define CAN_FILTER_BANK_COUNT       (28)
#define CAN_SLAVE_START_FILTER_BANK (14)

/**
 * Automatic retransmission (bxCAN NART bit), applied by CAN_Init().
 *
 * 1 = enabled (default) - the peripheral retries a frame that lost arbitration
 *                         or hit a bus error until it is acknowledged.
 * 0 = disabled          - a frame is transmitted once; a failed frame is dropped
 *                         and its mailbox is released immediately.
 *
 * @note With retransmission enabled a frame that is never acknowledged (no other
 * node on the bus, missing termination, bit timing mismatch) occupies its mailbox
 * indefinitely. bxCAN has only 3 TX mailboxes, so three such frames block every
 * further transmission. CAN_HandleScheduled() recovers via CAN_TX_FAIL_LIMIT.
 */
#define CAN_AUTO_RETRANSMISSION (0U)

/**
 * Consecutive failed enqueue attempts of a single scheduled message before
 * CAN_HandleScheduled() aborts all pending TX requests to unblock the mailboxes.
 * Counts periods, not loop iterations. Set to 0 to disable the recovery.
 */
#define CAN_TX_FAIL_LIMIT (3U)

/**
 * @brief Generic macro to swap endianness based on variable type.~
 * 
 * Endiannes should be handlend in GetData function of every 
 * * usage: 
 * uint32_t val = 0x12345678;
 * val = SWAP_ENDIANNESS(val); // Becomes 0x78563412
 */
#define SWAP_ENDIANNESS(x) _Generic((x),       \
    uint8_t:  (x),                             \
    int8_t:   (x),                             \
    uint16_t: __builtin_bswap16(x),                  \
    int16_t:  __builtin_bswap16(x),                  \
    uint32_t: __builtin_bswap32(x),                  \
    int32_t:  __builtin_bswap32(x),                  \
    uint64_t: __builtin_bswap64(x),                  \
    int64_t:  __builtin_bswap64(x)                   \
)

/**
 * @brief Extracts the n-th byte from variable x.
 * @warning Do not pass expressions with side effects (e.g., x++) as arguments,
 * as they may be evaluated multiple times.
 * @param x The source variable (uint8_t, uint16_t, or uint32_t).
 * @param n The byte index (0 for LSB).
 */
#define GET_BYTE(x, n) ((uint8_t)(((x) >> ((n) * 8u)) & 0xFFu))

/**
 * Periodic CAN message
 */
struct CAN_scheduledMsg
{
	CAN_TxHeaderTypeDef header;     // frame header
	uint32_t periodMs;              // period of this message
	uint32_t lastTick;              // time stamp of the last message
	void (*getData)(uint8_t *data, void *context); // fetches data
	void *context;                  // user callback context
	uint32_t txFailCount;           // consecutive periods this message could not be enqueued, managed by the driver
};

/**
 * Periodic CAN message list used for automation
 */
struct CAN_scheduledMsgList
{
	struct CAN_scheduledMsg list[CAN_MAX_MSG];
	uint8_t size;
	uint32_t txMailbox;
};

/**
 * Incoming CAN message
 */
struct CAN_IncomingMsg
{
	CAN_RxHeaderTypeDef header;
	uint8_t data[CAN_MAX_DLC];
};

/**
 * Incoming CAN message buffer
 */
struct CAN_IncomingMsgList
{
	struct CAN_IncomingMsg list[CAN_MAX_MSG];
	volatile uint8_t count;         /* written by the RX ISR, read by the main loop */
	volatile uint8_t receiveFlag;
	volatile uint8_t head;          /* owned by the ISR */
	volatile uint8_t tail;          /* owned by the main loop */
};

/**
 * Setup functions
 */

/**
 * @brief Configure a 16-bit list-mode filter accepting up to four standard IDs.
 *        Repeat an ID to fill unused slots. Call before CAN_Init().
 */
HAL_StatusTypeDef CAN_ConfigFilterList16(CAN_HandleTypeDef *hcan, uint8_t bank, const uint16_t stdIds[4]);

/**
 * @brief Configure a 32-bit mask-mode filter on standard IDs. Call before CAN_Init().
 */
HAL_StatusTypeDef CAN_ConfigFilterMask32(CAN_HandleTypeDef *hcan, uint8_t bank, uint16_t stdId, uint16_t stdMask);

/**
 * @brief Initialize CAN peripheral
 *
 * @param hcanPtr   Pointer to CAN handle
 */
HAL_StatusTypeDef CAN_Init(CAN_HandleTypeDef *hcan);

/**
 * Functions for scheduled messages
 */

 /**
 * @brief Process all scheduled CAN messages (call in main loop)
 *
 * @param hcanPtr      Pointer to CAN handle
 * @param scheduler    Pointer to the message scheduler
 */
void CAN_HandleScheduled(CAN_HandleTypeDef *hcanPtr, struct CAN_scheduledMsgList *scheduler);

/**
 * @brief Add new message to the periodic buffer
 *
 * @param msg      Pointer to the message to add
 * @param buffer   Pointer to the buffer that holds messages
 * @retval HAL_StatusTypeDef   State of the operation
 */
HAL_StatusTypeDef CAN_AddScheduledMsg(struct CAN_scheduledMsg *msg, struct CAN_scheduledMsgList *buffer);

/**
 * @brief Remove message from the periodic buffer
 *
 * @param id       ID of the message to remove
 * @param buffer   Pointer to the buffer that holds messages
 * @retval HAL_StatusTypeDef   State of the operation
 */
HAL_StatusTypeDef CAN_RemoveScheduledMsg(uint32_t id, struct CAN_scheduledMsgList *buffer);

/* Incoming CAN message buffer */

/**
 * @brief Add incoming CAN message to the buffer
 *
 * @param header  Pointer to received CAN header
 * @param data    Pointer to received CAN payload
 * @retval HAL_StatusTypeDef   State of the operation
 */
HAL_StatusTypeDef CAN_AddIncomingMsg(struct CAN_IncomingMsgList *buffer, CAN_RxHeaderTypeDef *header, uint8_t *data);

/**
 * @brief Read and remove the oldest pending message (FIFO order by arrival, not by CAN ID)
 *
 * @param msg  Pointer to storage for the received message
 * @retval HAL_StatusTypeDef   State of the operation
 */
HAL_StatusTypeDef CAN_GetLatestMessage(struct CAN_IncomingMsgList *buffer, struct CAN_IncomingMsg *msg);

#endif /* CAN_DRIVER_H */

/**
  * @file can_driver.c
  * @brief CAN bus driver for PERLA
  * @author AGH EKO-ENERGIA
  * @author Kacper Lasota
  */

/*
 * TODO
 *
 * Error handling both on bus and generic error messages
 * Filter configuration
 * Received messages handling
 *
 */
#include "can_driver.h"

/* Include error handler if available */
#if __has_include("error_handler.h")
#include "error_handler.h"
#define ERROR_HANDLER_AVAILABLE (1)
#else
#define ERROR_HANDLER_AVAILABLE (0)
#endif

HAL_StatusTypeDef CAN_Init(CAN_HandleTypeDef *hcanPtr)
{
	if (hcanPtr == NULL)
	{
		return HAL_ERROR;
	}

	/*
	 * Single-CAN configuration (CAN2 removed).
	 * Accept-all mask on bank 0 → every incoming ID lands in FIFO0.
	 * SlaveStartFilterBank stays at 14 for backwards-compatibility with any
	 * future re-introduction of CAN2; it has no effect when CAN2 is disabled.
	 */
	CAN_FilterTypeDef filterConfig = {0};

	filterConfig.FilterBank            = 0;
	filterConfig.FilterMode            = CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale           = CAN_FILTERSCALE_32BIT;
	filterConfig.FilterIdHigh          = 0x0000;
	filterConfig.FilterIdLow           = 0x0000;
	filterConfig.FilterMaskIdHigh      = 0x0000;
	filterConfig.FilterMaskIdLow       = 0x0000;
	filterConfig.FilterFIFOAssignment  = CAN_RX_FIFO0;
	filterConfig.FilterActivation      = ENABLE;
	filterConfig.SlaveStartFilterBank  = 14;

	if (HAL_CAN_ConfigFilter(hcanPtr, &filterConfig) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (HAL_CAN_Start(hcanPtr) != HAL_OK)
	{
		return HAL_ERROR;
	}

	/* RX FIFO0 pending IRQ — thermistor / safe-state frames handled in HAL_CAN_RxFifo0MsgPendingCallback */
	if (HAL_CAN_ActivateNotification(hcanPtr, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

HAL_StatusTypeDef CAN_AddScheduledMsg(struct CAN_scheduledMsg *msg, struct CAN_scheduledMsgList *buffer)
{
	// basic error checking
	if (buffer->size >= CAN_MAX_MSG)
	{
		return HAL_ERROR;
	}
	if (msg->periodMs == 0)
	{
		return HAL_ERROR;
	}

	struct CAN_scheduledMsg tempMsg = *msg;
	tempMsg.lastTick = HAL_GetTick();
	tempMsg.txFailCount = 0;

	// check if id already exists in the buffer
	for (uint8_t i = 0; i < buffer->size; i++)
	{
		if ((buffer->list[i].header.IDE == CAN_ID_STD && buffer->list[i].header.StdId == tempMsg.header.StdId) ||
			(buffer->list[i].header.IDE == CAN_ID_EXT && buffer->list[i].header.ExtId == tempMsg.header.ExtId))
		{
			return HAL_ERROR;
		}
	}

	buffer->list[buffer->size] = tempMsg;
	buffer->size++;
	return HAL_OK;
}

HAL_StatusTypeDef CAN_RemoveScheduledMsg(uint32_t id, struct CAN_scheduledMsgList *buffer)
{
	for (uint8_t i = 0; i < buffer->size; i++)
	{
		if ((buffer->list[i].header.IDE == CAN_ID_STD && buffer->list[i].header.StdId == id) ||
			(buffer->list[i].header.IDE == CAN_ID_EXT && buffer->list[i].header.ExtId == id))
		{
			while (i + 1 < buffer->size)
			{
				buffer->list[i] = buffer->list[i + 1];
				i++;
			}
			buffer->size--;
			return HAL_OK;
		}
	}

	return HAL_ERROR;
}

void CAN_HandleScheduled(CAN_HandleTypeDef *hcanPtr, struct CAN_scheduledMsgList *scheduler)
{
	if (hcanPtr == NULL || scheduler == NULL)
	{
		return;
	}

	uint32_t currentTick = HAL_GetTick();
	for (uint8_t i = 0; i < scheduler->size; i++)
	{
		struct CAN_scheduledMsg *msg = &scheduler->list[i];
		// unsigned difference stays correct across the HAL_GetTick() wrap
		if ((currentTick - msg->lastTick) >= msg->periodMs)
		{
			uint8_t data[CAN_MAX_DLC];
			// Initialize data to 0 to be safe
			for (uint8_t k = 0; k < CAN_MAX_DLC; k++)
			{
				data[k] = 0;
			}
			
			if (msg->getData != NULL)
			{
				msg->getData(data, msg->context);
			}
			
			if (HAL_CAN_AddTxMessage(hcanPtr, &msg->header, data, &scheduler->txMailbox) != HAL_OK)
			{
				/*
				 * No free mailbox, or the peripheral is not ready. Re-arm so this
				 * message waits a full period before trying again - leaving
				 * lastTick stale would make the branch above true on every main
				 * loop iteration and turn the period into a busy retry. Skip only
				 * this message, so one blocked frame cannot starve the rest.
				 */
				msg->lastTick = currentTick;

				// saturate rather than wrap, so CAN_TX_FAIL_LIMIT stays
				// usable over its whole range instead of being capped by the
				// width of the counter
				if (msg->txFailCount < UINT32_MAX)
				{
					msg->txFailCount++;
				}

				/*
				 * All three mailboxes stuck for several periods in a row. With
				 * automatic retransmission enabled an unacknowledged frame is
				 * retried forever and never releases its mailbox, so drop the
				 * pending requests to let the queue drain. On a mailbox that is
				 * mid-transmission the abort takes effect at the end of the
				 * current attempt.
				 */
				if ((CAN_TX_FAIL_LIMIT != 0U) && (msg->txFailCount >= CAN_TX_FAIL_LIMIT))
				{
					HAL_CAN_AbortTxRequest(hcanPtr, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
					msg->txFailCount = 0;
				}

				continue;
			}

			msg->txFailCount = 0;

			/*
			 * Advance by whole periods so the cadence does not drift with the
			 * execution time of the send, and resynchronise if we fell more than
			 * one period behind, to avoid a catch-up burst after a long stall.
			 */
			msg->lastTick += msg->periodMs;
			if ((currentTick - msg->lastTick) >= msg->periodMs)
			{
				msg->lastTick = currentTick;
			}
		}
	}
}

HAL_StatusTypeDef CAN_AddIncomingMsg(struct CAN_IncomingMsgList *buffer, CAN_RxHeaderTypeDef *header, uint8_t *data)
{
	if (buffer == NULL || header == NULL || data == NULL)
	{
		return HAL_ERROR;
	}

	if (buffer->count >= CAN_MAX_MSG)
	{
		return HAL_ERROR;
	}

	struct CAN_IncomingMsg *dst = &buffer->list[buffer->head];
	dst->header = *header;
	memcpy(dst->data, data, CAN_MAX_DLC);

	buffer->head = (buffer->head + 1) % CAN_MAX_MSG;
	buffer->count++;
	buffer->receiveFlag = 1;

	return HAL_OK;
}

HAL_StatusTypeDef CAN_GetLatestMessage(struct CAN_IncomingMsgList *buffer, struct CAN_IncomingMsg *msg)
{
	if (buffer == NULL || msg == NULL)
	{
		return HAL_ERROR;
	}

	if (buffer->count == 0)
	{
		return HAL_ERROR;
	}

	*msg = buffer->list[buffer->tail];
	buffer->tail = (buffer->tail + 1) % CAN_MAX_MSG;
	buffer->count--;

	if (buffer->count == 0)
	{
		buffer->receiveFlag = 0;
	}

	return HAL_OK;
}

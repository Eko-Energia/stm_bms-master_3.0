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

static HAL_StatusTypeDef applyFilter(CAN_HandleTypeDef *hcanPtr, CAN_FilterTypeDef *f, uint8_t bank)
{
	if (hcanPtr == NULL || bank >= CAN_FILTER_BANK_COUNT)
	{
		return HAL_ERROR;
	}
	f->FilterBank            = bank;
	f->FilterFIFOAssignment  = CAN_RX_FIFO0;
	f->FilterActivation      = ENABLE;
	f->SlaveStartFilterBank  = CAN_SLAVE_START_FILTER_BANK;
	return HAL_CAN_ConfigFilter(hcanPtr, f);
}

HAL_StatusTypeDef CAN_ConfigFilterList16(CAN_HandleTypeDef *hcanPtr, uint8_t bank, const uint16_t stdIds[4])
{
	if (stdIds == NULL)
	{
		return HAL_ERROR;
	}

	/* In 16-bit list mode each of the four registers holds one standard ID,
	   left-aligned at bit 5 (STID[10:0] occupies bits 15:5). */
	CAN_FilterTypeDef f = {0};
	f.FilterMode      = CAN_FILTERMODE_IDLIST;
	f.FilterScale     = CAN_FILTERSCALE_16BIT;
	f.FilterIdHigh     = (uint16_t)(stdIds[0] << 5);
	f.FilterIdLow      = (uint16_t)(stdIds[1] << 5);
	f.FilterMaskIdHigh = (uint16_t)(stdIds[2] << 5);
	f.FilterMaskIdLow  = (uint16_t)(stdIds[3] << 5);
	return applyFilter(hcanPtr, &f, bank);
}

HAL_StatusTypeDef CAN_ConfigFilterMask32(CAN_HandleTypeDef *hcanPtr, uint8_t bank, uint16_t stdId, uint16_t stdMask)
{
	CAN_FilterTypeDef f = {0};
	f.FilterMode      = CAN_FILTERMODE_IDMASK;
	f.FilterScale     = CAN_FILTERSCALE_32BIT;
	f.FilterIdHigh     = (uint16_t)(stdId << 5);
	f.FilterIdLow      = 0u;
	f.FilterMaskIdHigh = (uint16_t)(stdMask << 5);
	f.FilterMaskIdLow  = 0u;      /* IDE and RTR unmasked; callers bounds-check in software */
	return applyFilter(hcanPtr, &f, bank);
}

HAL_StatusTypeDef CAN_Init(CAN_HandleTypeDef *hcanPtr)
{
	if (hcanPtr == NULL)
	{
		return HAL_ERROR;
	}

	/*
	 * NART is only writable while the peripheral is still in initialisation
	 * mode, that is after HAL_CAN_Init() and before HAL_CAN_Start(), so this
	 * must stay in this order. Init.AutoRetransmission is kept in sync so a
	 * later HAL_CAN_Init() does not silently revert it.
	 */
#if (CAN_AUTO_RETRANSMISSION != 0U)
	CLEAR_BIT(hcanPtr->Instance->MCR, CAN_MCR_NART);
	hcanPtr->Init.AutoRetransmission = ENABLE;
#else
	SET_BIT(hcanPtr->Instance->MCR, CAN_MCR_NART);
	hcanPtr->Init.AutoRetransmission = DISABLE;
#endif

	if (HAL_CAN_Start(hcanPtr) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_CAN_ActivateNotification(hcanPtr, CAN_IT_RX_FIFO0_MSG_PENDING);
}

HAL_StatusTypeDef CAN_AddScheduledMsg(struct CAN_scheduledMsg *msg, struct CAN_scheduledMsgList *buffer)
{
	if (msg == NULL || buffer == NULL)
	{
		return HAL_ERROR;
	}

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
				 * No free mailbox. Leave lastTick alone so the frame stays due
				 * and retries on the next pass; re-arming it would skip the slot
				 * and silence the frame for a whole period. Only three mailboxes
				 * exist, so a burst drains across successive passes.
				 */

				// count missed periods, not passes: ordinary contention frees a
				// mailbox in microseconds and must not reach CAN_TX_FAIL_LIMIT
				const uint32_t missedPeriods = (currentTick - msg->lastTick) / msg->periodMs;
				if (missedPeriods > msg->txFailCount)
				{
					msg->txFailCount = missedPeriods;

					// blocked for CAN_TX_FAIL_LIMIT periods: with automatic
					// retransmission an unacknowledged frame keeps its mailbox
					// forever, so drop the pending requests. The compare above
					// limits this to one abort per period.
					if ((CAN_TX_FAIL_LIMIT != 0U) && (msg->txFailCount >= CAN_TX_FAIL_LIMIT))
					{
						HAL_CAN_AbortTxRequest(hcanPtr, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
					}
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

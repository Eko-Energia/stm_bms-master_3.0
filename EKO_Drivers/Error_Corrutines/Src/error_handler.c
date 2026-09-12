/**
 * @file error_handler.c
 * @brief Error handling and reporting implementation for PERLA CAN network
 * @author Antoni Wozniak (atomwoz)
 *
 * @details Implements error reporting via CAN frames.
 *          Supports bxCAN architecture.
 */

#include "error_handler.h"
#include <string.h>

/* ============================================================================
 * Private Variables
 * ============================================================================ */

/* Global variables removed - replaced by context struct */

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void haltNode(EH_HandleTypeDef *hehandler);
static void getData_HeightbeatOK(uint8_t *data, void *context);
static void getData_Error(uint8_t *data, void *context);
static void updateTransmissionInterval(EH_HandleTypeDef *hehandler);
static void setNodeFrameSource(EH_HandleTypeDef *hehandler,
                               void (*getData)(uint8_t *, void *), uint32_t periodMs);

/* ============================================================================
 * Initialization Functions
 * ============================================================================ */

/**
 * @brief Initialize error handler with bxCAN
 *
 * @param hcanPtr     Pointer to CAN handle
 * @param nodeIdVal   Node ID (used as error frame ID)
 * @param schedulerPtr Pointer to the CAN scheduled message list
 */
void EH_init(EH_HandleTypeDef *hehandler, CAN_HandleTypeDef *hcanPtr, uint16_t nodeIdVal, struct CAN_scheduledMsgList *schedulerPtr)
{
	if (hehandler == NULL || hcanPtr == NULL || schedulerPtr == NULL)
	{
		return;
	}

	hehandler->phcan = hcanPtr;
	hehandler->nodeId = nodeIdVal;
	// Error frame ID is node ID on most 6 bits of 11 bit CAN ID and messageID=0
	hehandler->errorFrameId = hehandler->nodeId;
	hehandler->scheduler = schedulerPtr;
	hehandler->isInitialized = 1;
	hehandler->isHalted = 0;
	
	hehandler->activeErrorCount = 0;
	hehandler->currentTransmitIndex = 0;

	// Add Heartbeat OK message to scheduler (1000ms period)
	struct CAN_scheduledMsg heartbeatMsg;
	heartbeatMsg.header.StdId = hehandler->errorFrameId;
	heartbeatMsg.header.ExtId = 0;
	heartbeatMsg.header.IDE = CAN_ID_STD;
	heartbeatMsg.header.RTR = CAN_RTR_DATA;
	heartbeatMsg.header.DLC = ERROR_FRAME_DLC;
	heartbeatMsg.header.TransmitGlobalTime = DISABLE;
	heartbeatMsg.periodMs = HEARTBEAT_INTERVAL;
	heartbeatMsg.lastTick = 0;
	heartbeatMsg.getData = getData_HeightbeatOK;
	heartbeatMsg.context = hehandler;

	CAN_AddScheduledMsg(&heartbeatMsg, hehandler->scheduler);
}

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Report an error without stopping the node
 * @param errorCode    Unique error code
 * @param severity     Error severity level
 */
void EH_report(EH_HandleTypeDef *hehandler, uint16_t errorCode, errorSeverity_e severity)
{
	EH_reportEx(hehandler, errorCode, severity, NULL, 0);
}

/**
 * @brief Report an error and halt the node
 * @param errorCode    Unique error code
 * @param severity     Error severity level
 */
void EH_stop(EH_HandleTypeDef *hehandler, uint16_t errorCode, errorSeverity_e severity)
{
	EH_stopEx(hehandler, errorCode, severity, NULL, 0);
}

/**
 * @brief Trigger system-wide Safe State
 * @param reason    Reason code
 */
void EH_triggerSafeState(EH_HandleTypeDef *hehandler, uint16_t reason)
{
	// Send error frame with ERROR_SEVERITY_SAFE_STATE and reason as error code
	EH_reportEx(hehandler, reason, ERROR_SEVERITY_SAFE_STATE, NULL, 0);
}

/**
 * @brief Report an error with additional diagnostic data
 * @param errorCode    Unique error code
 * @param severity     Error severity level
 * @param data         Specific data pointer
 * @param dataLen      Length of data
 */
void EH_reportEx(EH_HandleTypeDef *hehandler, uint16_t errorCode, errorSeverity_e severity, const uint8_t *data, uint8_t dataLen)
{
	if (hehandler == NULL || !hehandler->isInitialized)
	{
		return;
	}

	int8_t existingIndex = -1;
	for (uint8_t i = 0; i < hehandler->activeErrorCount; i++) {
		if (hehandler->activeErrors[i].errorCode == errorCode) {
			existingIndex = i;
			break;
		}
	}

	// Latched before the insert path below reassigns existingIndex
	uint8_t isNewError = (existingIndex < 0);

	uint8_t dataL = (dataLen > ERROR_SPECIFIC_DATA_SIZE) ? ERROR_SPECIFIC_DATA_SIZE : dataLen;

	if (existingIndex >= 0) {
		// Update existing
		hehandler->activeErrors[existingIndex].pendingClear = 0;
		hehandler->activeErrors[existingIndex].severity = severity;
		hehandler->activeErrors[existingIndex].specificDataLen = dataL;
		if (data != NULL && dataL > 0) {
			memcpy(hehandler->activeErrors[existingIndex].specificData, data, dataL);
		} else {
			memset(hehandler->activeErrors[existingIndex].specificData, 0, ERROR_SPECIFIC_DATA_SIZE);
		}
	} else {
		// Insert new
		if (hehandler->activeErrorCount < MAX_ACTIVE_ERRORS) {
			existingIndex = hehandler->activeErrorCount;
			hehandler->activeErrors[existingIndex].sent = 0;
			hehandler->activeErrors[existingIndex].pendingClear = 0;
			hehandler->activeErrorCount++;
		} else {
			// Find lowest severity (highest numerical enum value)
			int8_t lowestIndex = -1;
			errorSeverity_e lowestSev = (errorSeverity_e)-1; 
			// Enum values: SAFE_STATE=0, ERROR=1, WARNING=2, INFO=3
			for (uint8_t i = 0; i < hehandler->activeErrorCount; i++) {
				if (hehandler->activeErrors[i].severity > lowestSev || lowestIndex == -1) {
					lowestSev = hehandler->activeErrors[i].severity;
					lowestIndex = i;
				}
			}
			// If new is more severe or equally severe than the lowest severity error in queue
			if (severity <= lowestSev) {
				existingIndex = lowestIndex;
			} else {
				return; // Reject because buffer full and new error has lower priority
			}
		}

		if (existingIndex >= 0) {
			hehandler->activeErrors[existingIndex].errorCode = errorCode;
			hehandler->activeErrors[existingIndex].severity = severity;
			hehandler->activeErrors[existingIndex].specificDataLen = dataL;
			if (data != NULL && dataL > 0) {
				memcpy(hehandler->activeErrors[existingIndex].specificData, data, dataL);
			} else {
				memset(hehandler->activeErrors[existingIndex].specificData, 0, ERROR_SPECIFIC_DATA_SIZE);
			}
		}
	}

	// First error inserted: Setup error scheduler message.
	// Repeat reports must not re-install it, that would reset lastTick and starve the frame.
	if (isNewError && hehandler->activeErrorCount == 1 && existingIndex == 0) {
		setNodeFrameSource(hehandler, getData_Error, ERROR_INTERVAL);
	}

	updateTransmissionInterval(hehandler);
}

/**
 * @brief Report an error with diagnostic data and halt the node
 * @param errorCode    Unique error code
 * @param severity     Error severity level
 * @param data         Specific data pointer
 * @param dataLen      Length of data
 */
void EH_stopEx(EH_HandleTypeDef *hehandler, uint16_t errorCode, errorSeverity_e severity, const uint8_t *data, uint8_t dataLen)
{
	// First report the error to switch scheduler state
	EH_reportEx(hehandler, errorCode, severity, data, dataLen);
	haltNode(hehandler);
}

/**
 * @brief Clear a specific error from the active error state
 *
 * Checks if there are no more active errors. If so, returns to Heartbeat OK.
 * @param errorCode The error code to clear
 */
void EH_clear(EH_HandleTypeDef *hehandler, uint16_t errorCode)
{
	if (hehandler == NULL || !hehandler->isInitialized)
	{
		return;
	}

	int8_t foundIndex = -1;
	for (uint8_t i = 0; i < hehandler->activeErrorCount; i++) {
		if (hehandler->activeErrors[i].errorCode == errorCode) {
			foundIndex = i;
			break;
		}
	}

	if (foundIndex >= 0 && !hehandler->activeErrors[foundIndex].sent) {
		// Never reached the bus. Hold it so the next node frame carries it once.
		hehandler->activeErrors[foundIndex].pendingClear = 1;
		return;
	}

	if (foundIndex >= 0) {
		for (uint8_t i = foundIndex; i < hehandler->activeErrorCount - 1; i++) {
			hehandler->activeErrors[i] = hehandler->activeErrors[i + 1];
		}
		hehandler->activeErrorCount--;

		if (hehandler->currentTransmitIndex >= hehandler->activeErrorCount && hehandler->activeErrorCount > 0) {
			hehandler->currentTransmitIndex = 0;
		}

		if (hehandler->activeErrorCount == 0) {
			setNodeFrameSource(hehandler, getData_HeightbeatOK, HEARTBEAT_INTERVAL);
		} else {
			updateTransmissionInterval(hehandler);
		}
	}
}

/**
 * @brief Get the configured Node ID
 * @return Current node ID
 */
uint8_t EH_getActiveCount(EH_HandleTypeDef *hehandler)
{
	if (hehandler == NULL) return 0;

	uint8_t n = 0;
	for (uint8_t i = 0; i < hehandler->activeErrorCount; i++) {
		if (!hehandler->activeErrors[i].pendingClear) { n++; }
	}
	return n;
}

uint16_t EH_getNodeId(EH_HandleTypeDef *hehandler)
{
	if (hehandler == NULL) return 0;
	return hehandler->nodeId;
}

/**
 * @brief Check if error handler is initialized
 * @return 1 if initialized, 0 otherwise
 */
uint8_t EH_isInitialized(EH_HandleTypeDef *hehandler)
{
	if (hehandler == NULL) return 0;
	return hehandler->isInitialized;
}

/* ============================================================================
 * Internal Sizing Callback
 * ============================================================================ */
/* Swap the node frame's payload source without re-adding it. Removing and
   re-adding resets lastTick, and a fault that flutters would then restart the
   period on every transition and starve the frame. */
static void setNodeFrameSource(EH_HandleTypeDef *hehandler,
                               void (*getData)(uint8_t *, void *), uint32_t periodMs)
{
	if (hehandler == NULL || hehandler->scheduler == NULL) return;

	for (uint8_t i = 0; i < hehandler->scheduler->size; i++) {
		if (hehandler->scheduler->list[i].header.StdId == hehandler->errorFrameId) {
			hehandler->scheduler->list[i].getData  = getData;
			hehandler->scheduler->list[i].periodMs = periodMs;
			return;
		}
	}
}

static void updateTransmissionInterval(EH_HandleTypeDef *hehandler)
{
	if (hehandler == NULL || hehandler->scheduler == NULL || hehandler->activeErrorCount == 0) return;
	
	// fixed cadence: a faulting node must not add bus load
	for (uint8_t i = 0; i < hehandler->scheduler->size; i++) {
		if (hehandler->scheduler->list[i].header.StdId == hehandler->errorFrameId) {
			hehandler->scheduler->list[i].periodMs = ERROR_INTERVAL;
			break;
		}
	}
}

/* ============================================================================
 * Diagnostic API Getter Functions
 * ============================================================================ */
uint8_t EH_GetAllActive(EH_HandleTypeDef *hehandler, EH_ActiveError *outArray, uint8_t maxLen)
{
	if (hehandler == NULL || outArray == NULL) return 0;

	uint8_t copied = 0;
	for (uint8_t i = 0; i < hehandler->activeErrorCount && copied < maxLen; i++) {
		outArray[copied++] = hehandler->activeErrors[i];
	}
	return copied;
}

uint8_t EH_GetAllActiveErrors(EH_HandleTypeDef *hehandler, EH_ActiveError *outArray, uint8_t maxLen)
{
	if (hehandler == NULL || outArray == NULL) return 0;

	uint8_t copied = 0;
	for (uint8_t i = 0; i < hehandler->activeErrorCount && copied < maxLen; i++) {
		if (hehandler->activeErrors[i].severity == ERROR_SEVERITY_ERROR ||
			hehandler->activeErrors[i].severity == ERROR_SEVERITY_SAFE_STATE) 
		{
			outArray[copied++] = hehandler->activeErrors[i];
		}
	}
	return copied;
}

uint8_t EH_GetAllActiveWarnings(EH_HandleTypeDef *hehandler, EH_ActiveError *outArray, uint8_t maxLen)
{
	if (hehandler == NULL || outArray == NULL) return 0;

	uint8_t copied = 0;
	for (uint8_t i = 0; i < hehandler->activeErrorCount && copied < maxLen; i++) {
		if (hehandler->activeErrors[i].severity == ERROR_SEVERITY_WARNING) 
		{
			outArray[copied++] = hehandler->activeErrors[i];
		}
	}
	return copied;
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Callback to generate Heartbeat OK payload
 * @param data Pointer to data buffer (8 bytes)
 */
static void getData_HeightbeatOK(uint8_t *data, void *context)
{
	UNUSED(context); // Not needed for simple heartbeat, but good practice
	
	// Construct Heartbeat OK payload (Error Code 0xFFFF, Severity INFO)
	uint16_t errorCode = HEARTBEAT_ERROR_CODE;
	errorSeverity_e severity = ERROR_SEVERITY_INFO;
	uint8_t halted = 0;

	// Bytes 0-1: Error code (Little Endian)
	data[0] = (uint8_t)(errorCode & 0xFF);
	data[1] = (uint8_t)((errorCode >> 8) & 0xFF);

	// Byte 2: Flags
	// DBC BMSMaster_NODE: Reserved 16|4, Severity 20|3, Halted 23|1 - so within
	// byte 2 that is reserved 0-3, severity 4-6, halted 7.
	data[2] = (uint8_t)(((severity & 0x07) << 4) | ((halted & 0x01) << 7));

	// Bytes 3-7: Zero
	memset(&data[3], 0, 5);
}

/**
 * @brief Callback to multiplex and generate Error payload
 * @param data Pointer to data buffer (8 bytes)
 */
static void getData_Error(uint8_t *data, void *context)
{
	EH_HandleTypeDef *hehandler = (EH_HandleTypeDef*)context;
	if (hehandler == NULL || hehandler->activeErrorCount == 0) return;

	uint8_t idx = hehandler->currentTransmitIndex;

	uint16_t errorToSend = hehandler->activeErrors[idx].errorCode;
	errorSeverity_e sev = hehandler->activeErrors[idx].severity;

	// Bytes 0-1: Error code
	data[0] = (uint8_t)(errorToSend & 0xFF);
	data[1] = (uint8_t)((errorToSend >> 8) & 0xFF);

	// Byte 2: Flags
	data[2] = (uint8_t)(((sev & 0x07) << 4) | ((hehandler->isHalted & 0x01) << 7));

	// Bytes 3-7: Specific data
	memcpy(&data[3], hehandler->activeErrors[idx].specificData, ERROR_SPECIFIC_DATA_SIZE);

	hehandler->activeErrors[idx].sent = 1;
	hehandler->currentTransmitIndex = (idx + 1) % hehandler->activeErrorCount;

	// data[] is already packed, so dropping entries here cannot affect this frame
	for (uint8_t i = 0; i < hehandler->activeErrorCount; ) {
		if (hehandler->activeErrors[i].pendingClear && hehandler->activeErrors[i].sent) {
			for (uint8_t j = i; j + 1 < hehandler->activeErrorCount; j++) {
				hehandler->activeErrors[j] = hehandler->activeErrors[j + 1];
			}
			hehandler->activeErrorCount--;
		} else {
			i++;
		}
	}

	if (hehandler->activeErrorCount == 0) {
		hehandler->currentTransmitIndex = 0;
		setNodeFrameSource(hehandler, getData_HeightbeatOK, HEARTBEAT_INTERVAL);
	} else if (hehandler->currentTransmitIndex >= hehandler->activeErrorCount) {
		hehandler->currentTransmitIndex = 0;
	}
}

/**
 * @brief Enter infinite loop, handling only CAN communication
 * @note This function does not return
 */
static void haltNode(EH_HandleTypeDef *hehandler)
{
	if (hehandler == NULL) return;
	hehandler->isHalted = 1;

	// Infinite loop - node is halted, only CAN is processed
	while (1)
	{
		// Process queued messages (Heartbeat/Error)
		// We need to support multiple CAN controllers here?
		// Since we have hehandler, we use it.
		if (hehandler->scheduler != NULL && hehandler->phcan != NULL)
		{
			CAN_HandleScheduled(hehandler->phcan, hehandler->scheduler);
		}
	}
}

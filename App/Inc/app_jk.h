#ifndef APP_JK_H
#define APP_JK_H
#include <stdbool.h>
#include <stdint.h>
#include "error_handler.h"
#include "jk_protocol.h"
#include "main.h"

void JK_Init(UART_HandleTypeDef *huart, EH_HandleTypeDef *eh);

/** @brief Drive the RS485 exchange. Takes the tick rather than reading it. */
void JK_Task(uint32_t nowMs);

/** @brief From HAL_UART_TxCpltCallback, which fires on TC - the last stop bit. */
void JK_OnTxComplete(void);

/** @brief From HAL_UARTEx_RxEventCallback. ISR context. */
void JK_OnRxEvent(uint16_t size);

bool JK_Valid(void);
const JK_Data_t *JK_Data(void);

#endif /* APP_JK_H */

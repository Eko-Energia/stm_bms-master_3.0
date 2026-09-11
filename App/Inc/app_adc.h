#ifndef APP_ADC_H
#define APP_ADC_H
#include <stdbool.h>
#include <stdint.h>
#include "error_handler.h"

/**
 * @brief Bind the caller's DMA buffer and error handle, and reset the filter.
 *        Calibration and HAL_ADC_Start_DMA are started by app.c. The buffer is
 *        owned by the caller, which makes this module testable without hardware.
 * @param dmaBuf 3 halfwords: rank 1 = PC0 temp, 2 = PC1 current, 3 = PC2 voltage
 */
void ADC_Init(volatile uint16_t *dmaBuf, EH_HandleTypeDef *eh);

/** @brief Called from HAL_ADC_ConvCpltCallback. ISR context. */
void ADC_OnConvComplete(void);

/**
 * @brief Consume a completed scan into the filter, or report a stalled stream.
 * @param nowMs current tick; only used to detect a stopped conversion stream
 */
void ADC_Task(uint32_t nowMs);

/** @brief True once the window has filled and scans are still arriving. */
bool     ADC_Ready(void);
uint16_t ADC_PackDecivolts(void);
int16_t  ADC_PackDeciamps(void);
uint16_t ADC_TempCenti(void);

#endif /* APP_ADC_H */

#ifndef APP_CONTACTOR_H
#define APP_CONTACTOR_H
#include <stdbool.h>
#include <stdint.h>
#include "main.h"

void CONTACTOR_Init(TIM_HandleTypeDef *htim);

/** @brief Advance the safe-state machine. Takes the tick rather than reading it. */
void CONTACTOR_Task(uint32_t nowMs);

/** @brief Note a SafeState frame. ISR context; only IDs 1 and 3 matter. */
void CONTACTOR_OnSafeStateFrame(uint32_t stdId);

/** @brief Drop the duty to 0 %. For Error_Handler. */
void CONTACTOR_ForceOpen(void);

bool CONTACTOR_IsClosed(void);

#endif /* APP_CONTACTOR_H */

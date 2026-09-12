#ifndef APP_THERM_H
#define APP_THERM_H
#include <stdint.h>
#include "error_handler.h"

#define THERM_MODULES     (7u)
#define THERM_PER_MODULE  (9u)

/* PCBCells boards send raw = (degC + 49) / 0.39216 and wrap above 51 degC.
 * CAN_App_OnRx2 undoes both, giving the (0.39216, 0) the databases specify.
 * Set to 0 only when the boards are reflashed: a corrected byte de-biased
 * twice is wrong, and nothing detects the mismatch at runtime. */
#define THERM_LEGACY_DEBIAS  (1)
#define THERM_LEGACY_BIAS    (125u)   /* 49 degC / 0.39216, rounded */
#define THERM_LEGACY_LOWEST  (124u)   /* what a legacy board sends for 0 degC */

void THERM_Init(EH_HandleTypeDef *eh);

/** @brief Ingest one PCBCells thermistor frame. ISR context: two byte stores. */
void THERM_OnFrame(uint32_t stdId, uint8_t raw);

/** @brief Advance the 10-sample window and recompute. Caller owns the 1 Hz cadence. */
void THERM_Task(void);

/** @brief Filtered raw count. module 1..7, therm 1..9. Out of range returns 0. */
uint8_t THERM_Filtered(uint8_t module, uint8_t therm);

/** @brief Hottest filtered count across all 63, for the overtemperature check. */
uint8_t THERM_MaxRaw(void);

/** @brief Module 1..7 the THERM_MaxRaw() count came from. */
uint8_t THERM_MaxModule(void);

/** @brief Thermistor 1..9 the THERM_MaxRaw() count came from. */
uint8_t THERM_MaxTherm(void);

/** @brief Count of received thermistors pinned at either end of the lookup. */
uint8_t THERM_SaturatedCount(void);

#endif /* APP_THERM_H */

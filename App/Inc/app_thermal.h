#ifndef APP_THERMAL_H
#define APP_THERMAL_H
#include <stdint.h>
#include "error_handler.h"

/*
 * Pack thermistor limits, in whole degrees C. Change them here and nowhere
 * else: the raw counts the comparison uses are derived from these.
 *
 * Both judge the hottest thermistor in the pack. Positions on
 * THERM_DISABLED_LIST report 0 and never reach that maximum, so they are
 * excluded without a second test.
 */
#define THERMAL_WARN_DEGC   (48u)   /* BMS_ERR_CAN2_TEMP_HIGH, warning  */
#define THERMAL_ERROR_DEGC  (52u)   /* BMS_ERR_CAN2_TEMP_EXTREME, error */

void THERMAL_Init(EH_HandleTypeDef *eh);

/**
 * @brief Judge the hottest pack thermistor against both limits. No hysteresis:
 *        a reading that falls back clears on the same call.
 * @param packMaxRaw hottest pack thermistor, raw count at 0.39216 degC/LSB
 * @param packModule module 1..7 that reading came from
 * @param packTherm  thermistor 1..9 that reading came from
 */
void THERMAL_Evaluate(uint8_t packMaxRaw, uint8_t packModule, uint8_t packTherm);

#endif /* APP_THERMAL_H */

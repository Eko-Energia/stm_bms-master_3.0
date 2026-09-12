#ifndef APP_THERMAL_H
#define APP_THERMAL_H
#include <stdbool.h>
#include <stdint.h>
#include "error_handler.h"

void THERMAL_Init(EH_HandleTypeDef *eh);

/**
 * @brief Judge both temperature sources against the 60 degC limit.
 * @param boardValid false while the ADC filter is still filling
 * @param boardCenti on-board NTC, 0.01 degC/LSB
 * @param packMaxRaw hottest pack thermistor, raw count at 0.39216 degC/LSB
 * @param packModule module 1..7 that reading came from
 * @param packTherm  thermistor 1..9 that reading came from
 */
void THERMAL_Evaluate(bool boardValid, uint16_t boardCenti,
                      uint8_t packMaxRaw, uint8_t packModule, uint8_t packTherm);

#endif /* APP_THERMAL_H */

#include "app_thermal.h"
#include "bms_errors.h"

/* 60 degC limit from notionSpec, with hysteresis so the fault cannot chatter.
   Pack thermistors are raw counts at 0.39216 degC/LSB: 153 counts = 60.0 degC. */
#define TEMP_LIMIT_CENTI   (6000u)
#define TEMP_CLEAR_CENTI   (5700u)
#define THERM_LIMIT_RAW    (153u)
#define THERM_CLEAR_RAW    (145u)

static EH_HandleTypeDef *ehandler;
static bool boardHot;
static bool packHot;

void THERMAL_Init(EH_HandleTypeDef *eh)
{
    ehandler = eh;
    boardHot = false;
    packHot = false;
}

void THERMAL_Evaluate(bool boardValid, uint16_t boardCenti,
                      uint8_t packMaxRaw, uint8_t packModule, uint8_t packTherm)
{
    if (ehandler == NULL) { return; }

    if (boardValid) {
        if (!boardHot && boardCenti > TEMP_LIMIT_CENTI) {
            const uint8_t blob[5] = { (uint8_t)(boardCenti & 0xFFu),
                                      (uint8_t)(boardCenti >> 8), 0u, 0u, 0u };
            EH_reportEx(ehandler, BMS_ERR_TEMP_HIGH, ERROR_SEVERITY_ERROR, blob, 2u);
            boardHot = true;
        } else if (boardHot && boardCenti < TEMP_CLEAR_CENTI) {
            EH_clear(ehandler, BMS_ERR_TEMP_HIGH);
            boardHot = false;
        }
    }

    if (!packHot && packMaxRaw > THERM_LIMIT_RAW) {
        /* Spec 6.4: pack, thermistor, raw count - a technician needs to know
           which of the 63 is overheating, not just that one is. */
        const uint8_t blob[5] = { packModule, packTherm, packMaxRaw, 0u, 0u };
        EH_reportEx(ehandler, BMS_ERR_CAN2_TEMP_HIGH, ERROR_SEVERITY_ERROR, blob, 3u);
        packHot = true;
    } else if (packHot && packMaxRaw < THERM_CLEAR_RAW) {
        EH_clear(ehandler, BMS_ERR_CAN2_TEMP_HIGH);
        packHot = false;
    }
}

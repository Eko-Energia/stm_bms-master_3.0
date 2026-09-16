#include "app_thermal.h"
#include "bms_errors.h"

/* Pack thermistors are raw counts at 0.39216 degC/LSB, and 0.39216 is 100/255,
   so raw = degC * 255 / 100. Rounded up, so a limit trips at or above itself
   rather than a fraction of a degree under: 48 degC is raw 123 (48.24 degC),
   52 degC is raw 133 (52.16 degC). */
#define THERMAL_RAW_AT(degC)  ((uint8_t)(((((degC) * 255u) + 99u) / 100u)))
#define THERMAL_WARN_RAW      THERMAL_RAW_AT(THERMAL_WARN_DEGC)
#define THERMAL_ERROR_RAW     THERMAL_RAW_AT(THERMAL_ERROR_DEGC)

_Static_assert(THERMAL_WARN_RAW < THERMAL_ERROR_RAW,
               "the warning must trip before the error");

static EH_HandleTypeDef *ehandler;

void THERMAL_Init(EH_HandleTypeDef *eh)
{
    ehandler = eh;
}

void THERMAL_Evaluate(uint8_t packMaxRaw, uint8_t packModule, uint8_t packTherm)
{
    if (ehandler == NULL) { return; }

    /* Spec 6.4: a technician needs to know which of the 63 is hot, not just
       that one is. */
    const uint8_t blob[5] = { packModule, packTherm, packMaxRaw, 0u, 0u };

    /* Escalating, so exactly one of the two stands at a time: above the error
       limit the warning is redundant and would cost a heartbeat slot of its
       own. No hysteresis anywhere - both follow the pack directly. */
    if (packMaxRaw >= THERMAL_ERROR_RAW) {
        EH_clear(ehandler, BMS_ERR_CAN2_TEMP_HIGH);
        EH_reportEx(ehandler, BMS_ERR_CAN2_TEMP_EXTREME, ERROR_SEVERITY_ERROR, blob, 3u);
    } else if (packMaxRaw >= THERMAL_WARN_RAW) {
        EH_clear(ehandler, BMS_ERR_CAN2_TEMP_EXTREME);
        EH_reportEx(ehandler, BMS_ERR_CAN2_TEMP_HIGH, ERROR_SEVERITY_WARNING, blob, 3u);
    } else {
        EH_clear(ehandler, BMS_ERR_CAN2_TEMP_EXTREME);
        EH_clear(ehandler, BMS_ERR_CAN2_TEMP_HIGH);
    }
}

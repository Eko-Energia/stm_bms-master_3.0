#include "app_adc.h"
#include "bms_calib.h"
#include "bms_errors.h"
#include <string.h>

#define ADC_CHANNELS      (3u)
#define ADC_WINDOW        (10u)      /* samples per channel; min and max are trimmed */
#define CH_TEMP           (0u)
#define CH_CURRENT        (1u)
#define CH_VOLTAGE        (2u)

/* DBC signal ranges. Values outside are clamped and reported. */
#define VOLT_MIN_DV       (630u)
#define VOLT_MAX_DV       (870u)
#define CURRENT_MAX_DA    (3000)
#define TEMP_MAX_CENTI    (10000u)

/* Guard bands. The table spans counts 1092..3728, so a reading far outside it
   is a wiring fault rather than a temperature: an open NTC pulls the divider
   toward 0, a short toward full scale. Between the open band and the table
   start the reading is genuinely below 0 degC. */
#define NTC_OPEN_BELOW    (200u)
#define NTC_SHORT_ABOVE   (4000u)

/* Liveness. A scan completes every ~84 us, so 100 ms without one is ~1200
   missed conversions - a stopped DMA, not jitter. Reported before the next
   500 ms publish of frame 130. */
#define ADC_STALE_MS      (100u)

static volatile uint16_t *adcBuf;
static EH_HandleTypeDef  *ehandler;
static volatile uint8_t   convCplt;

static uint16_t window[ADC_CHANNELS][ADC_WINDOW];
static uint8_t  windowIdx;
static uint8_t  windowFill;

static uint16_t packDecivolts;
static int16_t  packDeciamps;
static uint16_t tempCenti;

static uint32_t lastScanMs;
static bool     scanClockSet;   /* lastScanMs seeded from the first Task call */
static bool     stalled;

/* Mean of the window with the single lowest and single highest sample removed. */
static uint16_t trimmedMean(const uint16_t *samples, uint8_t fill)
{
    if (fill == 0u) { return 0u; }            /* the divisors below are fill and fill-2 */

    uint32_t sum = 0u;
    uint16_t lo = 0xFFFFu, hi = 0u;

    for (uint8_t i = 0u; i < fill; i++) {
        const uint16_t s = samples[i];
        sum += s;
        if (s < lo) { lo = s; }
        if (s > hi) { hi = s; }
    }

    if (fill < 3u) {                          /* too short to trim meaningfully */
        return (uint16_t)((sum + (fill / 2u)) / fill);
    }
    sum -= ((uint32_t)lo + (uint32_t)hi);
    const uint32_t n = fill - 2u;
    return (uint16_t)((sum + (n / 2u)) / n);
}

static void report(uint16_t code, const uint8_t *data, uint8_t len)
{
    if (ehandler != NULL) {
        EH_reportEx(ehandler, code, ERROR_SEVERITY_ERROR, data, len);
    }
}

static void clear(uint16_t code)
{
    if (ehandler != NULL) {
        EH_clear(ehandler, code);
    }
}

static uint16_t countToCenti(uint16_t count)
{
    if (count >= calibNtcCount[100]) { return TEMP_MAX_CENTI; }
    if (count <= calibNtcCount[0])   { return 0u; }

    for (uint8_t i = 0u; i < 100u; i++) {
        if (count < calibNtcCount[i + 1u]) {
            const uint16_t lo   = calibNtcCount[i];
            const uint16_t span = (uint16_t)(calibNtcCount[i + 1u] - lo);
            if (span == 0u) { return (uint16_t)((uint32_t)i * 100u); }   /* table not monotonic */
            const uint32_t frac = ((uint32_t)(count - lo) * 100u) + (span / 2u);
            return (uint16_t)(((uint32_t)i * 100u) + (frac / span));
        }
    }
    return TEMP_MAX_CENTI;
}

void ADC_Init(volatile uint16_t *dmaBuf, EH_HandleTypeDef *eh)
{
    adcBuf = dmaBuf;
    ehandler = eh;
    convCplt = 0u;
    windowIdx = 0u;
    windowFill = 0u;
    packDecivolts = VOLT_MIN_DV;
    packDeciamps = 0;
    tempCenti = 0u;
    lastScanMs = 0u;
    scanClockSet = false;
    stalled = false;
    memset(window, 0, sizeof window);
}

void ADC_OnConvComplete(void) { convCplt = 1u; }

void ADC_Task(uint32_t nowMs)
{
    if (adcBuf == NULL) {
        return;
    }
    if (!scanClockSet) { lastScanMs = nowMs; scanClockSet = true; }

    if (convCplt == 0u) {
        /* The DMA is a bus master: if it stops, nothing else notices. Without
           this the last values freeze and ADC_Ready() stays latched true. */
        uint32_t idleMs = (uint32_t)(nowMs - lastScanMs);
        if (!stalled && idleMs >= ADC_STALE_MS) {
            if (idleMs > 0xFFFFu) { idleMs = 0xFFFFu; }
            const uint8_t blob[5] = { (uint8_t)(idleMs & 0xFFu),
                                      (uint8_t)((idleMs >> 8) & 0xFFu), 0u, 0u, 0u };
            report(BMS_ERR_ADC_STALLED, blob, 2u);
            stalled = true;
        }
        return;
    }
    convCplt = 0u;
    lastScanMs = nowMs;
    if (stalled) { stalled = false; clear(BMS_ERR_ADC_STALLED); }

    /*
     * No critical section: masking interrupts would not stop the DMA, which is
     * a bus master. Halfword loads are atomic on Cortex-M3 so no value tears,
     * and skew between three independent signals is harmless.
     */
    for (uint8_t ch = 0u; ch < ADC_CHANNELS; ch++) {
        window[ch][windowIdx] = (uint16_t)(adcBuf[ch] & 0x0FFFu);
    }
    windowIdx = (uint8_t)((windowIdx + 1u) % ADC_WINDOW);
    if (windowFill < ADC_WINDOW) { windowFill++; }

    const uint16_t tempCount = trimmedMean(window[CH_TEMP], windowFill);
    const uint16_t currCount = trimmedMean(window[CH_CURRENT], windowFill);
    const uint16_t voltCount = trimmedMean(window[CH_VOLTAGE], windowFill);

    /* Voltage. Max intermediate 4095 * 228554 = 936M, inside uint32. */
    uint32_t dv = (((uint32_t)voltCount * CALIB_PACK_V_NUM) + (CALIB_PACK_V_DEN / 2u))
                  / CALIB_PACK_V_DEN;
    if (dv < VOLT_MIN_DV || dv > VOLT_MAX_DV) {
        const uint8_t blob[5] = { (uint8_t)(dv & 0xFFu), (uint8_t)((dv >> 8) & 0xFFu), 0u, 0u, 0u };
        report(BMS_ERR_PACK_VOLT_RANGE, blob, 2u);
        dv = (dv < VOLT_MIN_DV) ? VOLT_MIN_DV : VOLT_MAX_DV;
    } else {
        clear(BMS_ERR_PACK_VOLT_RANGE);
    }
    packDecivolts = (uint16_t)dv;

    /* Current. Rounded away from zero so the sign is symmetric. */
    const int32_t delta = (int32_t)currCount - CALIB_CURRENT_OFFSET;
    int32_t da = delta * CALIB_CURRENT_NUM;
    da = (da >= 0) ? ((da + (CALIB_CURRENT_DEN / 2)) / CALIB_CURRENT_DEN)
                   : ((da - (CALIB_CURRENT_DEN / 2)) / CALIB_CURRENT_DEN);
    if (da > CURRENT_MAX_DA || da < -CURRENT_MAX_DA) {
        /* & 0xFF/>>8 safe: 12-bit ADC bounds da well within int16. The right
           shift of a possibly-negative da relies on GCC/ARM's arithmetic-shift
           definition, not standard C (implementation-defined) - fine on this
           fixed toolchain/target. */
        const uint8_t blob[5] = { (uint8_t)(da & 0xFF), (uint8_t)((da >> 8) & 0xFF), 0u, 0u, 0u };
        report(BMS_ERR_PACK_CURRENT_HIGH, blob, 2u);
        da = (da > 0) ? CURRENT_MAX_DA : -CURRENT_MAX_DA;
    } else {
        clear(BMS_ERR_PACK_CURRENT_HIGH);
    }
    packDeciamps = (int16_t)da;

    /* Temperature, with open and short discriminated from genuinely cold. */
    if (tempCount < NTC_OPEN_BELOW || tempCount > NTC_SHORT_ABOVE) {
        const uint8_t blob[5] = { (uint8_t)(tempCount & 0xFFu),
                                  (uint8_t)((tempCount >> 8) & 0xFFu), 0u, 0u, 0u };
        report(BMS_ERR_TEMP_SENSOR_FAULT, blob, 2u);
    } else {
        clear(BMS_ERR_TEMP_SENSOR_FAULT);
    }
    tempCenti = countToCenti(tempCount);
}

bool     ADC_Ready(void)         { return (windowFill >= ADC_WINDOW) && !stalled; }
uint16_t ADC_PackDecivolts(void) { return packDecivolts; }
int16_t  ADC_PackDeciamps(void)  { return packDeciamps; }
uint16_t ADC_TempCenti(void)     { return tempCenti; }

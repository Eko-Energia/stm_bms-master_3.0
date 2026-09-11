#ifndef BMS_CALIB_H
#define BMS_CALIB_H
#include <stdint.h>

/*
 * Measured on hardware. UNCALIBRATED: these are the reference values from
 * docs/adc.md and must be corrected at bring-up step 3 by comparing SWD reads
 * against a multimeter on PC0, PC1 and PC2. Vref and resistor tolerance
 * dominate the error budget 20-100x over any arithmetic effect.
 */
#define CALIB_PACK_V_NUM      (228554u)   /* decivolts per ADC count, scaled 1e6 */
#define CALIB_PACK_V_DEN      (1000000u)
#define CALIB_CURRENT_OFFSET  (2108)      /* ADC count at 0 A */
#define CALIB_CURRENT_NUM     (5)         /* deciamps = (count - offset) * NUM / DEN */
#define CALIB_CURRENT_DEN     (2)

/*
 * Expected ADC count per degree C, 0 to 100, for the 10k NTC on PC0 with a
 * fixed 10k to ground. Monotonically increasing.
 *
 * Indexed by count rather than resistance on purpose: the divider is
 * ratiometric with VREF+ (tied to VDDA internally on LQFP64), so the supply
 * cancels out and temperature is immune to Vref tolerance. Derived from
 * Bartek's validated resistance table - 10.0k at 25 degC, implied
 * B(0/25) = 3297, B(25/100) = 3441.
 */
extern const uint16_t calibNtcCount[101];

#endif /* BMS_CALIB_H */

#ifndef BMS_CALIB_H
#define BMS_CALIB_H
#include <stdbool.h>
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

/* ADC count per degree C, 0 to 100, monotonically increasing. Provenance and
   the count-indexed rationale are in docs/adc.md. */
extern const uint16_t calibNtcCount[101];

/** @brief True when calibNtcCount is strictly increasing, which is what keeps
 *         the temperature interpolation's divisor non-zero. Checked at init. */
bool CALIB_NtcCountIsMonotonic(void);

#endif /* BMS_CALIB_H */

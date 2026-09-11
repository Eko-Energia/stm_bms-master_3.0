# ADC Calculation Logic

## Channel map

ADC1 scans three analog inputs using circular DMA:

| Channel | Pin | Signal | Meaning |
| --- | --- | --- | --- |
| ADC1_IN10 | PC0 | `TEMP` | NTC thermistor voltage |
| ADC1_IN11 | PC1 | `HALL_OUT` | Hall current sensor output |
| ADC1_IN12 | PC2 | `VOLTAGE` | Pack voltage divider output |

The reference ADC driver averages four samples per channel and validates each result against the ADC resolution.

## ADC count to voltage

For a 12-bit ADC:

$$V_{pin} = \frac{ADC_{count}}{4095} \times V_{ref}$$

The reference implementation performs this operation through `ADC_Get_PinVoltage()`.

## Pack voltage

PC2 is connected through a resistor divider. The reference BMS code reconstructs the pack voltage with:

$$V_{pack} = V_{PC2} \times 28.362637362637$$

The multiplier must match the fitted resistor values and should be calibrated on the hardware.

## NTC temperature

The reference circuit uses a 10 kOhm NTC and 10 kOhm fixed resistor, with the **NTC on the high
side** (`V_CC` to NTC to `PC0` to the fixed 10 kOhm to ground):

$$R_t = 10000 \times \left(\frac{V_{CC}}{V_{NTC}} - 1\right)$$

This project (`App/Src/app_adc.c`, `App/Inc/bms_calib.h`) does not evaluate this formula at
runtime. It converts the raw ADC count directly against a **count-indexed lookup table**,
`calibNtcCount[101]` (index = degrees C, 0 to 100), with linear interpolation between adjacent
counts:

$$T = i + \frac{count - calibNtcCount[i]}{calibNtcCount[i+1] - calibNtcCount[i]}, \quad
calibNtcCount[i] \le count < calibNtcCount[i+1]$$

Indexing by count rather than by resistance is deliberate: the divider is ratiometric with
`VREF+` (tied to `VDDA` on this LQFP64 package), so a count-based table is immune to `VREF`/supply
tolerance in a way a resistance-based one is not - the supply term cancels before it ever reaches
the lookup.

Table anchors (measured, not derived from the formula above): count **1092** at 0 degC, **2048**
at 25 degC, **3143** at 60 degC, **3728** at 100 degC. A count below 1092 clamps to 0 degC, above
3728 clamps to 100 degC (`TEMP_MAX_CENTI`). Counts far outside the table (below 200 or above 4000)
are treated as an open or shorted NTC, not a temperature, and raise `BMS_ERR_TEMP_SENSOR_FAULT`.

Sampling time for all three ADC1 channels (`PC0`, `PC1`, `PC2`) is **239.5 cycles**
(`ADC_SAMPLETIME_239CYCLES_5`, `Core/Src/adc.c`), not the CubeMX default of 1.5 cycles: 1.5
cycles cannot charge the sample-and-hold capacitor through a 10 kOhm-plus divider before the
converter samples, which would bias every channel low.

## Current

The Hall sensor on PC1 uses this reference calibration:

$$I = \frac{ADC_{count} - 2108}{4}$$

`2108` is the zero-current offset and `4` is the counts-per-ampere gain. Both values must be recalibrated if the analogue hardware changes.

```c
float voltage;
ADC_Get_PinVoltage(&hadc1, &channelConfig, &adcBuffer,
                   ADC_CHANNEL_12, &voltage);
voltage *= 28.362637362637f;
```

After engineering conversion, the original application applies CAN scaling before packing the values into frames. The clean project contains the peripheral setup but not the original `BMS_Driver/ADC` application wrapper.

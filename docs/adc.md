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

The reference circuit uses a 10 kOhm NTC and 10 kOhm fixed resistor. With the NTC on the low side:

$$R_t = 10000 \times \left(\frac{V_{CC}}{V_{NTC}} - 1\right)$$

The resistance is converted with a 0 C to 100 C lookup table and linear interpolation:

$$T = T_i + (T_{i+1} - T_i) \frac{R_t - R_i}{R_{i+1} - R_i}$$

Guard against zero input voltage and invalid resistance. The reference BMS production range is 0 C to 60 C.

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

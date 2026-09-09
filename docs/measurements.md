# Temperature, Voltage, and Current Measurements

## ADC signal map

ADC1 scans three analog inputs in this order:

| ADC channel | Pin | Signal | BMS meaning |
| --- | --- | --- | --- |
| ADC1_IN10 | PC0 | `TEMP` | NTC thermistor voltage |
| ADC1_IN11 | PC1 | `HALL_OUT` | Hall current sensor output |
| ADC1_IN12 | PC2 | `VOLTAGE` | Scaled pack voltage |

The clean project configures three conversions and circular DMA in `Core/Src/adc.c`. The engineering-unit conversions are implemented in the original project in `BMS_Driver/ADC/Src/BMS_ADC_driver.c`.

## ADC pin voltage

For a 12-bit ADC and a 3.3 V reference:

$$V_{pin} = \frac{ADC_{count}}{4095} \times 3.3$$

The original driver delegates this conversion to `ADC_Get_PinVoltage()`. Use the actual reference voltage and calibration data for production accuracy.

## Pack voltage

The pack voltage is measured through a resistor divider on PC2. The reference application reads `ADC1_IN12`, converts it to the PC2 voltage, multiplies by `28.362637362637...`, and then applies CAN scaling.

$$V_{pack} = V_{PC2} \times 28.362637362637$$

```c
float packVoltage;
ADC_Get_PinVoltage(&hadc1, &channelConfig, &adcBuffer,
                   ADC_CHANNEL_12, &packVoltage);
packVoltage *= 28.362637362637f;
```

Verify the multiplier against the fitted resistor values and calibrate it before using this value for protection decisions.

## NTC temperature

The reference application reads PC0 and assumes a 10 kOhm NTC with a 10 kOhm fixed resistor:

$$R_t = 10000 \times \left(\frac{V_{CC}}{V_{NTC}} - 1\right)$$

The resistance is converted to degrees Celsius using a 0 C to 100 C lookup table and linear interpolation.

```c
float ntcVoltage;
float thermistorResistance;
float temperatureC;

ADC_Get_PinVoltage(&hadc1, &channelConfig, &adcBuffer,
                   ADC_CHANNEL_10, &ntcVoltage);
thermistorResistance = 10000.0f * (3.3f / ntcVoltage - 1.0f);
temperatureC = BMS_ADC_NTC_GetTemperature(thermistorResistance);
```

Guard against `ntcVoltage == 0`, verify the divider orientation, and treat an invalid lookup result as a measurement fault. The source application defines 0 C to 60 C as the normal BMS temperature range.

## Current

The Hall sensor output is read from ADC1 channel 11 / PC1. The calibration in the reference application is:

$$I = \frac{ADC_{count} - 2108}{4}$$

An ADC count of 2108 represents approximately 0 A and four counts represent one ampere according to the current source code.

```c
uint16_t currentCount;
float currentA;

ADC_ReadChannel(&hadc1, &channelConfig, &adcBuffer,
                ADC_CHANNEL_11, &currentCount);
currentA = ((float)currentCount - 2108.0f) / 4.0f;
```

The offset and gain are sensor-specific and must be calibrated on the real hardware.

## Application order

```c
BMS_ADC_Init(&bms);

for (;;) {
    if (BMS_ADC_ReadValues(&bms) != HAL_OK) {
        /* Report a measurement fault and enter required safe behavior. */
    }
    /* Run CAN, relay, and protection logic here. */
}
```

The clean project does not currently contain these `BMS_*` application functions. This is the reference integration pattern.

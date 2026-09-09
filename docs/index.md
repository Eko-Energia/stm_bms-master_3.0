# STM BMS Master Documentation

This directory documents `stm_bms-master_3.0` and the reference application in `stm_bms-master`.

## Documents

- [Project overview](projectOverview.md)
- [Temperature, voltage, and current measurements](measurements.md)
- [PWM generation](pwmGeneration.md)
- [ADC calculation logic](adc.md)
- [BMS-JK communication](bmsJk.md)
- [CAN frames and driver integration](can.md)
- [PCB pinout](pcb.md)

## Project boundary

`stm_bms-master_3.0` is the clean Cube project. It contains generated STM32 startup, HAL, peripheral configuration, and the EKO CAN/PWM drivers. The application-level `BMS_Driver` sources are intentionally not copied into this directory.

The measurement formulas and BMS PWM state examples document code from the original `stm_bms-master` reference project. They describe the application layer to integrate when the clean project becomes a complete firmware application.

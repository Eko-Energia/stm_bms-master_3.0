# STM BMS Master Documentation

This directory documents `stm_bms-master_3.0` and the reference application in `stm_bms-master`.

It also holds `CAN-DATABASE/`, a git submodule pinning the team's `.dbc` files. Run
`git submodule update --init docs/CAN-DATABASE` after cloning; see [canDatabase.md](canDatabase.md).

## Documents

- [Project overview](projectOverview.md)
- [Temperature, voltage, and current measurements](measurements.md)
- [PWM generation](pwmGeneration.md)
- [ADC calculation logic](adc.md)
- [BMS-JK communication](bmsJk.md)
- [CAN frames and driver integration](can.md)
- [CAN database and C source generation](canDatabase.md)
- [PCB pinout](pcb.md)
- [stm32-mcp hardware server](stm32Mcp.md)
- [Claude Code plugins](claudePlugins.md)
- [Firmware specification](firmwareSpec.md)
- [Notion source specification](notionSpec.md)

## Project boundary

`stm_bms-master_3.0` is the clean Cube project. It contains generated STM32 startup, HAL, peripheral configuration, and the EKO CAN/PWM drivers. The application-level `BMS_Driver` sources are intentionally not copied into this directory.

The measurement formulas and BMS PWM state examples document code from the original `stm_bms-master` reference project. They describe the application layer to integrate when the clean project becomes a complete firmware application.

[notionSpec.md](notionSpec.md) is different in kind from the other documents: it is a translation of the original Notion requirements page and states intended behavior, not behavior present in this repository. Its closing section lists where it diverges from the current project.

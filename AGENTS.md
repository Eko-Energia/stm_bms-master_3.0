# AI Project Guide

This file is the navigation guide for AI assistants working on `stm_bms-master_3.0`.

## Required reading by task

| When producing or changing | Read first |
| --- | --- |
| Any hardware pin, peripheral, clock, or CubeMX configuration | `BMS-Master.ioc`, then `docs/pcb.md` |
| ADC setup, raw conversion, voltage, temperature, or current logic | `docs/adc.md`, then `docs/measurements.md` |
| Relay PWM output, duty cycle, timer configuration, or PWM input | `docs/pwmGeneration.md` |
| CAN frames, identifiers, payload packing, scheduling, or reception | `docs/can.md`, then `EKO_Drivers/CAN/Inc/can_driver.h` |
| DBC contents, node/signal names, or regenerating CAN C sources | `docs/canDatabase.md`, then `docs/CAN-DATABASE/*.dbc` |
| JK BMS, UART, RS485 direction, request/response, or received data | `docs/bmsJk.md`, then `Core/Src/usart.c` and `Core/Src/gpio.c` |
| Overall project structure or generated-versus-application code boundaries | `docs/projectOverview.md` |
| Building, flashing, serial exchanges, or memory reads driven from an AI session | `docs/stm32Mcp.md` |
| Original product requirements, intended feature set, or scope questions | `docs/notionSpec.md` |
| Documentation navigation or available project documents | `docs/index.md` |

## Project rules

1. Treat `BMS-Master.ioc` as the source of truth for the copied project's hardware configuration.
2. Verify a proposed pin or peripheral change against `docs/pcb.md` before editing generated code.
3. Keep generated CubeMX files separate from application logic. Preserve `USER CODE` sections and avoid placing application state machines in generated files.
4. The copied project is intentionally clean. Do not silently copy the original project's `BMS_Driver` application layer into it unless explicitly requested.
5. Distinguish documented reference behavior from code that is currently present in `stm_bms-master_3.0`.
6. Preserve the existing camelCase naming convention for documentation filenames and the lowercase `docs` directory.
7. After code changes, update the related documentation when the public behavior, pinout, frame format, or calculation changes.
8. CAN frame C sources are generated from `docs/CAN-DATABASE/*.dbc` by the team's cantools fork, never hand-written. Regenerate using `docs/canDatabase.md`; do not edit generated output, and do not edit the `.dbc` files from this repository.
9. Keep code simple, easily readable, not overengineered but still fast and optimized.
10. Keep comments short and meaningful, avoid essay-style and storytelling - hard facts easy to understand.
11. While using floating point operations, try to use integrated FPU if available. Always use float instead of double. If available use float functions instead of double ones, e.g. `powf()` instead of `pow()`.
12. Never use constants and variables bigger than 32 bits, try to optimize their size so that we save the memory.

## Validation expectations

Before reporting a hardware-related change as complete:

- confirm the relevant assignment in `BMS-Master.ioc`;
- check generated headers and source for matching labels and peripheral handles;
- build the STM32CubeIDE project when the toolchain is available;
- state clearly when a behavior is documented only and is not yet implemented in the clean project.

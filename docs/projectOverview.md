# Project Overview

## Purpose

This is a BMS master firmware base for an STM32F105R8Tx. STM32CubeMX generates peripheral initialization and STM32 HAL integration. The application layer is intended to read analog BMS signals, communicate over two CAN interfaces, control a relay with PWM, and report or forward data.

## Hardware configuration

The configuration is stored in `BMS-Master.ioc`.

| Peripheral | Configuration and purpose |
| --- | --- |
| ADC1 | Three-channel scan of PC0, PC1, and PC2 with circular DMA |
| CAN1 | CAN bus on PA11/PA12, 500 kbit/s |
| CAN2 | CAN bus on PB12/PB13, 500 kbit/s, receive interrupt enabled |
| TIM3 | Relay PWM output on PB0 / TIM3_CH3 |
| USART1 | Serial interface on PA9/PA10 |
| GPIO | Relay, fan, transceiver enables, standby, LEDs, and status inputs |
| SWD | Debug interface on PA13/PA14 |

The clock setup uses the internal 8 MHz HSI, divides it by two, and multiplies by nine for a 36 MHz system clock. ADC clocking is PCLK2 divided by four.

## Directory map

```text
stm_bms-master_3.0/
|-- BMS-Master.ioc                 CubeMX source configuration
|-- Core/                          generated startup and peripheral code
|-- Drivers/                      STM32F1 HAL and CMSIS files
|-- EKO_Drivers/
|   |-- CAN/                       reusable CAN scheduling driver
|   `-- PWM/                       reusable PWM input/output driver
|-- docs/                          project documentation
`-- *.project files                STM32CubeIDE project metadata
```

## Runtime startup

The generated `main()` initializes HAL, the system clock, GPIO, DMA, ADC1, CAN1, CAN2, USART1, and TIM3. It enables the configured interrupts and enters an empty loop. This is deliberate for the clean project: application objects and periodic calls still need to be added.

The original project adds the application layer under `BMS_Driver`. That layer owns the BMS object, ADC conversions, CAN scheduling, error handling, and PWM operating modes.

## Integration checklist

1. Open `stm_bms-master_3.0` as an STM32CubeIDE project.
2. Regenerate from `BMS-Master.ioc` only when generated files are expected to change.
3. Keep application code in user-code sections or separate application files.
4. Add the EKO driver include paths and source files to the build settings.
5. Start ADC DMA before consuming ADC buffer values.
6. Start CAN peripherals and configure filters before using the CAN API.
7. Initialize relay PWM before changing its duty cycle.

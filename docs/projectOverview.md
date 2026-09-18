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

The clock setup uses a **16 MHz external oscillator in BYPASS mode** (`HSE`, `PD0-OSC_IN`), not
the internal HSI, PLL-multiplied to a **72 MHz** system clock (`RCC.HSE_VALUE=16000000`,
`RCC.AHBFreq_Value=72000000` in `BMS-Master.ioc`). `PD1-OSC_OUT` is assigned but not wired -
BYPASS mode drives the oscillator input directly and does not use the MCU's inverting amplifier,
so `OSC_OUT` is reserved rather than in use; see [pcb.md](pcb.md). ADC clocking is **PCLK2 divided
by eight** (`RCC.ADCPresc=RCC_ADCPCLK2_DIV8`), giving a 9 MHz ADC clock, not PCLK2/4.

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

`main()` initializes HAL, the system clock, GPIO, DMA, ADC1, CAN1, CAN2, USART1, and TIM3, enables
the configured interrupts, and then calls **`app_main()`** (`App/Src/app.c`) instead of entering
an empty loop. `app_main()` runs module `Init` calls once and then the cooperative superloop
forever - it never returns. `main.c` gains three lines for this: `#include "app.h"`, `app_main();`
in `USER CODE BEGIN 2`, and `App_OnFatalError();` in `USER CODE BEGIN Error_Handler_Debug`.

The application layer - ADC conversions, CAN scheduling, error handling, the JK link, the
thermistor and contactor logic - lives in `App/` (`app_*.c`/`.h`), not in a copied `BMS_Driver`
tree from the original project. See [firmwareSpec.md](firmwareSpec.md) for the full module map.

## Integration checklist

1. Open `stm_bms-master_3.0` as an STM32CubeIDE project.
2. Regenerate from `BMS-Master.ioc` only when generated files are expected to change.
3. Keep application code in user-code sections or separate application files.
4. Add the EKO driver include paths and source files to the build settings.
5. Start ADC DMA before consuming ADC buffer values.
6. Start CAN peripherals and configure filters before using the CAN API.
7. Initialize relay PWM before changing its duty cycle.

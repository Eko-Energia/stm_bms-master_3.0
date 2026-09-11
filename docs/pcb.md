# PCB Pinout

This pinout is read from `BMS-Master.ioc` in `stm_bms-master_3.0`.

## Device

| Property | Value |
| --- | --- |
| MCU | STM32F105R8T6 / `STM32F105R8Tx` |
| Package | LQFP64 |
| MCU family | STM32F1 |
| Board type | Custom |
| System clock | 72 MHz, PLL from a 16 MHz `HSE` external oscillator in **BYPASS** mode (`RCC.HSE_VALUE=16000000`) - not a crystal |
| ADC clock | 9 MHz (PCLK2 / 8, `RCC.ADCPresc=RCC_ADCPCLK2_DIV8`) |
| ADC sampling time | 239.5 cycles on all three ADC1 channels (`ADC_SAMPLETIME_239CYCLES_5`) |

## Assigned pins

| MCU pin | Label / signal | Peripheral function | Direction or mode | Notes |
| --- | --- | --- | --- | --- |
| `PA0-WKUP` | `SYS_WKUP` | System wake-up input | Wake-up, locked | Wake-up pin |
| `PA7` | `HVIL` | GPIO input | Pull-down, locked | High-voltage interlock input |
| `PA8` | `CD` | GPIO input | Pull-up, locked | Carrier-detect/status input |
| `PA9` | USART1_TX | USART1 | Asynchronous TX | 115200 baud |
| `PA10` | USART1_RX | USART1 | Asynchronous RX | 115200 baud |
| `PA11` | CAN1_RX | CAN1 | CAN receive | CAN1 bus |
| `PA12` | CAN1_TX | CAN1 | CAN transmit | CAN1 bus |
| `PA13` | SWDIO | Serial Wire Debug | SWD | Debug data |
| `PA14` | SWCLK | Serial Wire Debug | SWD | Debug clock |
| `PB0` | `RELAY_CTRL` | TIM3_CH3 | Output compare/PWM channel 3 | Relay control output |
| `PB1` | `FAN_CONTROL` | GPIO output | Output | Fan control |
| `PB8` | `RED_LD` | GPIO output | Output | Red LED |
| `PB9` | `GREEN_LD` | GPIO output | Output | Green LED |
| `PB12` | CAN2_RX | CAN2 | CAN receive | CAN2 bus |
| `PB13` | CAN2_TX | CAN2 | CAN transmit | CAN2 bus |
| `PB14` | `TX_EN` | GPIO output | Output | Transmitter enable |
| `PB15` | `TRX_CE` | GPIO output | Output | Transceiver chip enable |
| `PC0` | `TEMP` | ADC1_IN10 | Analog input | Temperature sensor input |
| `PC1` | `HALL_OUT` | ADC1_IN11 | Analog input | Hall current sensor input |
| `PC2` | `VOLTAGE` | ADC1_IN12 | Analog input | Divided pack-voltage input |
| `PC4` | `RS_DIR` | GPIO output | Pull-down, high speed | RS485 direction control label |
| `PC5` | `RE_DIR` | GPIO output | Pull-down | RS485 receiver-enable label |
| `PC6` | `PWR_UP` | GPIO output | Output | Power-up control |
| `PC8` | `DR` | GPIO input | Pull-up | Data-ready/status input |
| `PC9` | `AM` | GPIO input | Pull-up | Address-match/status input |
| `PC10` | `nCAN2_Stby` | GPIO output | Pull-down | CAN2 standby control |
| `PC11` | `nCAN1_Stby` | GPIO output | Output | CAN1 standby control |
| `PD0-OSC_IN` | RCC oscillator input | RCC_OSC_IN | HSE-External-Clock-Source, locked | 16 MHz external oscillator input (BYPASS, not a crystal) |
| `PD1-OSC_OUT` | RCC oscillator output | RCC_OSC_OUT | HSE-External-Clock-Source, **not locked** | Assigned but **not wired** - reserved so a future switch to a crystal is one `.ioc` change, not a board respin |

## Analog channels

ADC1 is configured as a three-channel scan with circular DMA on `DMA1_Channel1`:

| Scan rank | ADC channel | Pin | Signal |
| ---: | --- | --- | --- |
| 1 | ADC1_IN10 | PC0 | `TEMP` |
| 2 | ADC1_IN11 | PC1 | `HALL_OUT` |
| 3 | ADC1_IN12 | PC2 | `VOLTAGE` |

The engineering calculations for these inputs are described in [adc.md](adc.md).

## CAN buses

| Bus | RX | TX | Nominal bit rate |
| --- | --- | --- | ---: |
| CAN1 | PA11 | PA12 | 500 kbit/s |
| CAN2 | PB12 | PB13 | 500 kbit/s |

The corresponding transceiver standby controls are `PC11` (`nCAN1_Stby`) for CAN1 and `PC10`
(`nCAN2_Stby`) for CAN2. **These are active-low: LOW is normal operation.** The `n` prefix in the
label is easy to misread as "not asserted when high" the way an nRESET or nCS line normally
works, but for these standby pins the opposite is true - driving either pin **HIGH puts both
transceivers into standby**, silencing all CAN traffic. `PC10` is pulled down at boot
(`GPIO_PULLDOWN`), so the reset state is the safe, normal-operation state.

## Timer output

TIM3 channel 3 is assigned to `PB0 / RELAY_CTRL` with a prescaler of 71 and period of 999. With the configured 72 MHz timer clock, the intended PWM frequency is:

$$f_{PWM} = \frac{72\,MHz}{(71 + 1)(999 + 1)} = 1\,kHz$$

The PWM behavior is described in [pwmGeneration.md](pwmGeneration.md).

## UART and RS485 note

This `.ioc` lists `USART1` only. `PA9`/`PA10` are the configured UART pins, used as the JK BMS
RS485 link over DMA (`DMA1_Channel4` TX, `DMA1_Channel5` RX), 115200 8N1. The labels `RS_DIR` and
`RE_DIR` are GPIO outputs on PC4 and PC5 that drive the SN65HVD72 transceiver's `DE`/`/RE` pins.
`USART2` is not assigned in this `.ioc` at all - `PA2`/`PA3` are not available for JK
communication. See [bmsJk.md](bmsJk.md) for the transport state machine and frame format.

## Unassigned or reserved functions

- `PA13` and `PA14` are reserved for SWD debugging.
- `PD0-OSC_IN` is the HSE oscillator input (BYPASS mode, external 16 MHz oscillator).
- `PD1-OSC_OUT` is assigned to `RCC_OSC_OUT` in the `.ioc` but is **not locked and not wired** -
  reserved so a future switch to a crystal (which needs both `OSC_IN` and `OSC_OUT`) is a `.ioc`
  and BOM change, not a board respin.
- `VP_SYS_VS_Systick` is the CubeMX virtual SysTick service and has no external package pin.
- The `.ioc` does not assign `USART2`; `PA2`/`PA3` are not configured for any peripheral.

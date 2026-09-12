# BMS Master — Notion Source Specification

English translation of the original Polish Notion page `BMS Master`. The wording is
kept close to the source, including its open questions and informal notes.

| Field | Value |
| --- | --- |
| Person | Bartek Rychlicki |
| Status | Waiting for refactor |
| Info | Being modified |
| Parent item | [STM](https://app.notion.com/p/STM-201bf9d3035d8003b185d8b50756d64d?pvs=21) |

> **Scope note.** This document is a requirements specification, not a description
> of the code in `stm_bms-master_3.0`. It states the intended behavior of the BMS
> Master board. See [projectOverview.md](projectOverview.md) for what the clean
> project currently contains, and the [Deviations](#deviations-from-the-current-project)
> section below.

## Description

A PCB located inside the large battery. Its job is to receive information from the
(off-the-shelf) [JK BMS](https://app.notion.com/p/BMS-JK-SMART-204bf9d3035d8095bdcbe1cec04a9bf7?pvs=21)
over RS485 — a UART/RS485 transceiver is connected to the STM, and the STM drives it
over UART — and from the battery packs, then forward that data onto CAN.

> 💡 Use `CAN_DRIVER` for all CAN communication, together with the Python CANtools
> (there is a fork on our GitHub).
>
> Run it with:
>
> ```
> cantools generate_c_source ./CAN_DB.dbc --node BMSMaster
> ```
>
> You only have to make sure the communication matrix is set up correctly.
>
> *(Working instructions for this: [canDatabase.md](canDatabase.md). The database is pinned in
> this repository as the `docs/CAN-DATABASE` submodule.)*

The board measures the battery voltage (pin PC2 — the divider present on the board
still has to be calculated; the STM maximum is 3.3 V) and the current drawn from or
charged into it. Current is on pin PC1
([current measurement with the L01Z300S05 device](https://app.notion.com/p/pomiar-pr-du-z-urz-dzenia-L01Z300S05-204bf9d3035d80839d71e6a49e90676a?pvs=21),
[Digi-Key part page](https://www.digikey.pl/pl/products/detail/tamura/L01Z300S05/529409)).
Temperature is on pin PC0 — Bartek already did this part and it works well, so take
it from him. The data is packed into frames.

> 💡 Do **not** use `ADC_DRIVER`.

![CAN_DB.dbc](BMS%20Master/image.png)

*CAN_DB.dbc*

## Pack thermistor aggregation

The board receives `PBCCELLS<x>_Therm<y>` frames (1 ≤ *x* ≤ 7, 1 ≤ *y* ≤ 9) on CAN2,
computes the average of *Z* measurements, and sends the result onto CAN1 in
`BMSMaster_PCBsTherm<y>Temp1` frames.

> The pinned database spells these `PCBCells<x>_Therm<y>` and `BMSMaster_PCBsTherm<y>Temp`.
> See [canDatabase.md](canDatabase.md); the database is authoritative.

![CAN2_DB.dbc](BMS%20Master/image%201.png)

*CAN2_DB.dbc*

![CAN_DB.dbc](BMS%20Master/image%202.png)

*CAN_DB.dbc*

## JK BMS link

The board communicates with the JK BMS through a UART/RS transceiver (USART1).

> 💡 Transceiver documentation — keep in mind that the communication is half-duplex:
>
> [SN65HVD72.PDF](BMS%20Master/SN65HVD72.pdf)

It receives the data and packs it into frames.

![CAN_DB.dbc](BMS%20Master/image%203.png)

*CAN_DB.dbc*

## Contactor control

In addition, pin PB0 controls the contactor. On engagement we apply 100 % PWM, and
after 2 seconds 50 % PWM. The contactor is engaged after the `SafeState_Node` frame
is received and disengaged after the `SafeState_Aktiv` frame is received (after
300 ms without a frame it returns to normal mode again).

![CAN_DB.dbc](BMS%20Master/image%204.png)

*CAN_DB.dbc*

> 💡 Use `PWM_DRIVER`.

There is also HVIL, but I still have to ask Bartek what it is about.

> ⚡ [ELECTRICAL PAGE](https://app.notion.com/p/BMS-MASTER-0091ffb519eb4c728ea63785604043a3?pvs=21)

> 💡 Below is the description of the CAN frames from the JK BMS.
>
> [BMS.Protocol.V2.5.ENGLISH.GOOGLE.Translate.pdf](BMS%20Master/BMS.Protocol.V2.5.ENGLISH.GOOGLE.Translate.pdf)

## Functions

- collecting data from the [JK BMS](https://app.notion.com/p/BMS-JK-SMART-204bf9d3035d8095bdcbe1cec04a9bf7?pvs=21) over RS485
- collecting data from the thermistors in every pack
    - current measurement with the L01Z300S05 device — [Digi-Key part page](https://www.digikey.pl/pl/products/detail/tamura/L01Z300S05/529409)
    - range from −300 A to +300 A. Positive is discharging, negative is charging
- measuring the main battery voltage
    - battery voltage range from 63 V to 87 V
    - pin voltage range after the voltage divider from 2.0221 V to 3.067 V
- thermistor temperature measurement — from 0 °C to 100 °C
- sending all data onto CAN1
- sending data over radio when the battery is outside the car:
    - radio transceiver → https://www.mouser.pl/datasheet/2/297/NRSAS00107_1-2559891.pdf
    - amplifier → https://www.analog.com/media/en/technical-documentation/data-sheets/MAX2232-MAX2233.pdf

STM32 documentation:

[STM32F303K8T6.pdf](https://www.tme.eu/Document/22f423643ca306651c3e9b4b2f6c6e2d/STM32F303K8T6.pdf)

## Deviations from the current project

These are differences between this specification and `stm_bms-master_3.0` as it
stands. They are recorded here so the spec is not mistaken for implemented behavior.

| Topic | This specification | Current project |
| --- | --- | --- |
| MCU documentation link | STM32F303K8T6 datasheet | `BMS-Master.ioc` targets STM32F105R8T6 — see [pcb.md](pcb.md) |
| JK BMS UART | USART1 | `BMS-Master.ioc` configures USART1 on PA9/PA10. [bmsJk.md](bmsJk.md) previously described USART2 on PA2/PA3, which is not assigned in the `.ioc`; it has been corrected. |
| Analog inputs | PC2 voltage, PC1 current, PC0 temperature | Matches the `.ioc`: PC0 `TEMP`, PC1 `HALL_OUT`, PC2 `VOLTAGE` |
| Contactor output | PB0 | Matches the `.ioc`: PB0 `RELAY_CTRL` on TIM3_CH3 — see [pwmGeneration.md](pwmGeneration.md) |
| Radio link | Required | No radio transceiver logic is present in the clean project |
| CAN/PWM drivers | `CAN_DRIVER`, `PWM_DRIVER` required; `ADC_DRIVER` forbidden | `EKO_Drivers/CAN` and `EKO_Drivers/PWM` are present; no ADC driver is used — see [adc.md](adc.md) |

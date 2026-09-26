# BMS Master

Battery management firmware for the **BMS Master** board in AGH Eko-Energia's solar car *Perła*.

It watches a 21S pack through three independent sources — the pack's own thermistor boards over
CAN2, a JK BMS over RS-485, and the board's own ADC channels — and publishes the result on CAN1
alongside a fault frame that the rest of the car listens to.

```
                 63 thermistors                  21 cells, V/I/SOC, 3 temps
                       |                                     |
                  CAN2 (in)                            USART1 / RS-485
                       |                                     |
                       +----------->  BMS Master  <----------+
                                           |
                            ADC1  <--------+--------> TIM3_CH3 contactor
                       (pack V, current, NTC)
                                           |
                                      CAN1 (out)
                              18 data frames + NODE faults
```

---

## Hardware

**STM32F105R8T6** — Cortex-M3, 64 KB flash, 64 KB RAM. Configuration lives in `BMS-Master.ioc`,
which is the source of truth for anything pin- or clock-related.

| Peripheral | Use |
| --- | --- |
| **CAN1** — PA11/PA12 | 500 kbit/s, the car's bus. Everything this board publishes |
| **CAN2** — PB12/PB13 | 500 kbit/s, receive only. The seven PCBCells thermistor boards |
| **USART1** — PA9/PA10 | 115200 8N1 over RS-485 to the JK BMS |
| **ADC1** — PC0/PC1/PC2 | NTC, Hall current, pack voltage divider. Scan + circular DMA |
| **TIM3_CH3** — PB0 | Contactor coil PWM |

Clocked from a **16 MHz external oscillator in BYPASS mode**, PLL'd to **72 MHz**. The ADC runs
at PCLK2/8 = 9 MHz.

---

## Layout

```text
App/                    application layer
  Inc/ Src/
    app.c                 superloop and task scheduling
    app_can.c             CAN1 frame registration and payload builders
    app_therm.c           CAN2 thermistor ingest, filtering, argmax
    app_thermal.c         pack temperature limits
    app_jk.c              JK RS-485 transport: poll, timeout, recovery
    jk_protocol.c         JK frame validation and TLV decode
    app_adc.c             ADC conversion, filtering, sensor faults
    app_contactor.c       contactor PWM and safe-state handling
    bms_calib.c           measured constants and the NTC lookup table
EKO_Drivers/            shared team drivers: CAN scheduler, PWM, error handler
  CAN/{Inc,Src}/          includes CAN_DB.* / CAN2_DB.*, generated from the database
Core/ Drivers/          CubeMX output and STM32 HAL - not hand-edited
docs/                   see docs/index.md
  CAN-DATABASE/           submodule pinning the team's .dbc files
tests/                  unit suite, oracle, and stress harnesses
```

---

## Building

STM32CubeIDE, or headless:

```bash
git submodule update --init docs/CAN-DATABASE
make -C Release        # or Debug
```

A Release build is **~23 KB flash, ~5.2 KB RAM** — roughly a third of each.

Generated CAN sources are committed and must never be hand-edited. Regenerate them from the
database instead; the procedure is in [`docs/canDatabase.md`](docs/canDatabase.md).

---

## Tests

Four gates, all runnable locally. Nothing here needs hardware.

```bash
make -C tests                      # unit suite + packing oracle
bash tests/stress/run_sweep.sh     # exhaustive input sweep
bash tests/stress/run_fuzz.sh      # JK protocol fuzzer, ASan + UBSan
bash tests/stress/run_soak.sh      # whole-system soak (~10 min)
bash tests/stress/run_soak.sh --quick
```

| Gate | Scope |
| --- | --- |
| **Unit suite** | 186 tests over every module, `-Wall -Wextra -Werror`. Includes an oracle that decodes real transmitted frames against the database with `cantools` |
| **Sweep** | Every input a conversion can receive — all 4096 ADC counts per channel, all 2048 CAN IDs, all 256 thermistor wire values |
| **Fuzzer** | 2 M mutations per population against the JK decoder |
| **Soak** | 112 simulated days, including the 32-bit tick wrap at 49.7 days, against a 64-bit reference model |

The soak is the only gate that reaches the tick wrap. Run the full version — not `--quick` —
for anything touching timing or the CAN scheduler.

---

## What it does

**Pack thermistors.** 63 positions across 7 boards, arriving on CAN2. The ID map is not monotonic
— odd packs count up, even packs count down — and is `_Static_assert`ed against the generated
frame IDs, so a database renumber breaks the build rather than silently cross-mapping sensors.
Each position gets a 10-sample trimmed mean; the hottest drives the limits.

**JK BMS.** Polled once a second over half-duplex RS-485. One request, one reply, both on DMA,
with the reply accepted only on an idle line. Decodes 21 cell voltages, pack voltage and current,
SOC, a derived SOH, curated warning flags and three temperatures.

**Faults.** Twelve codes on the `BMSMaster_NODE` frame, one per transmission at heartbeat cadence,
each guaranteed to reach the bus at least once even if the condition clears first. Codes and
payloads are in [`docs/errorCodes.csv`](docs/errorCodes.csv).

**Safety posture.** A reading is published as measured, never clamped into a plausible-looking
value; a range in the database is a check, not a transform. Where a sensor cannot be trusted, its
frame is not published at all rather than emitting a number someone might act on.

---

## Current limitations

Read these before trusting a reading from this board.

| | |
| --- | --- |
| **Two thermistors are disabled** | `PCB6Therm9` is short-circuited and `PCB3Therm6` reads offset. Both are listed in `App/Inc/bms_therm_disabled.h`, report 0 °C and are excluded from every check — so **the cells behind them have no thermal protection**, and nothing on the bus says so |
| **The master's own measurements are not published** | `CALIB_SEND_MASTER_MEASUREMENTS` is 0. The pack divider reads ~30 % low against the JK, the Hall sensor does not reach PC1, and the NTC input sits at full scale. Pack voltage and current faults key on the JK instead |
| **Analogue calibration is inherited, not measured** | The NTC table's fixed resistor and B value come from the old repo. The Hall sensor's sign and gain have never been checked against a known current |
| **Probe assignment is unconfirmed** | Which JK external probe is the contactor and which the control bowl is a wiring fact, not a protocol one |

---

## Documentation

Start at [`docs/index.md`](docs/index.md). The two worth knowing about:

- [`docs/firmwareSpec.md`](docs/firmwareSpec.md) — what the firmware does and why, section by section
- [`docs/notionSpec.md`](docs/notionSpec.md) — the original Notion requirements, translated, with a
  table of where the implementation deliberately diverges

`AGENTS.md` maps each kind of change to the document to read first.

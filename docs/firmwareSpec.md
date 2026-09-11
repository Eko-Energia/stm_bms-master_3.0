# BMS Master Firmware Specification

Design specification for the `stm_bms-master_3.0` application layer, agreed 2026-09-10.

This document is the implementation contract. It states what the firmware does and why,
including the points where a decision was taken against an alternative. Where a value is
assumed rather than confirmed, it says so.

Sources of truth: `BMS-Master.ioc` for hardware, `docs/CAN-DATABASE/*.dbc` for frames,
`docs/notionSpec.md` for intended behaviour.

## 1. Scope

The board sits inside the large battery pack. It measures pack voltage, current and its own
temperature; collects per-pack thermistor readings from CAN2; polls a JK BMS over RS485; drives
the main contactor; and publishes everything on CAN1.

**In scope:** measurement pipeline, pack thermistor aggregation, JK link, contactor control,
CAN1 transmit schedule, fault reporting, LED annunciation.

**Out of scope, by decision:**

| Item | Reason |
| --- | --- |
| nRF905 radio (`PWR_UP`, `TRX_CE`, `TX_EN`, `CD`, `DR`, `AM`) | No SPI in the `.ioc` and no documented SPI pinout. Control pins held at their reset-default idle state. |
| Fan control (`PB1`) | No thermal design specifies what it cools or at what temperature. Pin stays low. |
| HVIL (`PA7`) | Purpose unconfirmed. Configured as an input, otherwise unused and unreported. |
| Independent watchdog | Deliberately deferred to avoid complexity; revisit later. |
| `SafeState_SyncTick` (ID 30) | Not needed. The frame broadcasts a 32-bit ms counter every 10 s so other ECUs can align timestamps, but nothing asks BMS Master to timestamp anything. The CAN1 filter admits only IDs 1 and 3, so it is dropped. |
| `PCBCells<x>_NODE` frames (210, 220 ... 270) | Not consumed. They carry each pack's own `Error_Code` and `Severity`, but a silent pack is already detected from thermistor absence (section 6.3), which is what this board needs. Consequence: a pack reporting its own fault while still sending thermistor data goes unnoticed here. |
| CAN bus-off recovery | `AutoBusOff` stays `DISABLE` with no software recovery; deferred. |

## 2. Hardware baseline

STM32F105R8T6, Cortex-M3, **no FPU**, 64 KB flash, 64 KB RAM.

| Domain | Value |
| --- | --- |
| HSE | 16 MHz **external oscillator**, `RCC_HSE_BYPASS` |
| SYSCLK | 72 MHz (PREDIV1 /2 -> 8 MHz, PLL x9), `FLASH_LATENCY_2` |
| AHB / APB1 / APB2 | 72 / 36 / 72 MHz; APB1 timer clock 72 MHz |
| ADC | PCLK2 /8 = 9 MHz |
| CAN1, CAN2 | 500 kbit/s; Prescaler 9, BS1 6 TQ, BS2 1 TQ, SJW 1 TQ, 87.5 % sample point |
| TIM3 CH3 | PSC 71, ARR 999 -> 1 kHz on PB0 |
| USART1 | 115200 8N1 on PA9/PA10, RS485 to the JK BMS |

The PCB routes **only** `OSC_IN`; there is no `OSC_OUT`, which is why BYPASS is correct.
72 MHz is unreachable from HSI on this part (`IS_RCC_PLL_MUL` permits only x4..x9 and x6.5,
and the HSI path is a fixed 4 MHz), so HSE is mandatory.

### 2.1 Required `.ioc` changes

Application code never lives in generated files, so these are made in CubeMX and regenerated.

| Change | Reason |
| --- | --- |
| Un-assign `PD1-OSC_OUT` | Reserved in error by commit `5486dbc`; a BYPASS clock source does not use it. Frees PD1. |
| TIM3 CH3: Output Compare -> **PWM Generation CH3** | **Blocking.** `OCMode = TIM_OCMODE_TIMING` drives no waveform on PB0, so the contactor is never actuated. |
| ADC sampling `1CYCLE_5` -> **`239CYCLES_5`** (all 3 channels) | 1.5 cycles at 9 MHz is 167 ns; the sample-and-hold cannot charge through a resistor divider or a 10 k NTC. 239.5 cycles = 28 us. |
| `CAN1_RX0_IRQn` priority 0 -> **2** | Priority 0 is the most urgent in the NVIC. CAN1 RX carries two frames every 5 s; CAN2 RX handles 63 thermistor frames. Equal priority (2) is correct. |
| `PA10` pull -> **`GPIO_PULLUP`** | The receiver is muted during transmit, so the transceiver's `RO` goes high-impedance and PA10 floats. Set in the `.ioc` so it survives regeneration. |
| Enable **USART1 DMA**: TX on `DMA1_Channel4`, RX on `DMA1_Channel5` | Required by the non-blocking JK transport (section 7.3). No conflict with ADC1 on `DMA1_Channel1`. |
| Enable the **USART1 global interrupt** | `HAL_UARTEx_ReceiveToIdle_DMA` needs the IDLE flag, and `HAL_UART_TxCpltCallback` fires on TC. Priority 4. |
| Add `App/Inc` and `App/Src` to the build; keep `EKO_Drivers/LED/Inc`; **remove** the `EKO_Drivers/ADC/Inc` and `EKO_Drivers/Error_Corrutines/Inc` include paths | Those two directories do not exist and never will - `notionSpec` forbids `ADC_DRIVER`. Stale paths imply drivers that were deliberately excluded. |
| Verify `FW_F1 V1.8.7` | The `.ioc` claims 1.8.7 but `Drivers/` was untouched by `5486dbc`. |

CAN bit timing is **not** changed: it is a fleet-wide convention, matching
`stm_acc_brake_steer_pcb` (`Prescaler=9`, `BS1=6TQ`).

### 2.2 GPIO polarity

| Signal | Pin | Level | Confidence |
| --- | --- | --- | --- |
| `nCAN1_Stby` | PC11 | **LOW = normal operation** | Confirmed. The `n` prefix misleads; these drive an active-high `STB` input. |
| `nCAN2_Stby` | PC10 | **LOW = normal operation** | Confirmed. Driving HIGH would put both transceivers in standby and kill all CAN. |
| `RS_DIR` | PC4 | `DE`, active high | Confirmed by Bartek, 2026-09-11 |
| `RE_DIR` | PC5 | `/RE`, active low | Confirmed by Bartek, 2026-09-11 |

Both standby pins are written LOW explicitly during CAN init, not left to the generated
`MX_GPIO_Init` reset state, so correctness survives regeneration.

## 3. Architecture

**Language: C only.** No C++ anywhere - no `.cpp` files, no `extern "C"` wrappers, no C++
constructs. The project already relies on C11 features (`_Generic` in `can_driver.h`,
`_Static_assert` in section 6.1), so `gnu11` as CubeIDE emits it is the target standard.

Bare-metal cooperative superloop. **No RTOS**: no tasks, no queues, no scheduler, no blocking
calls. Cadence comes from `HAL_GetTick()` comparisons and the CAN transmit scheduler.

```
App/Inc/    app.h  app_adc.h  app_can.h  app_contactor.h
            app_fault.h  app_jk.h  app_therm.h  app_timing.h  jk_protocol.h
App/Src/    app.c  app_adc.c  app_can.c  app_contactor.c
            app_fault.c  app_jk.c  app_therm.c  jk_protocol.c
EKO_Drivers/CAN/  Inc/{can_driver,CAN_DB,CAN2_DB}.h  Src/{can_driver,CAN_DB,CAN2_DB}.c
EKO_Drivers/LED/  Inc/led_driver.h  Src/led_driver.c
```

`main.c` gains one line, `app_main();` in `USER CODE BEGIN 2`.

### 3.1 State ownership

Each module keeps its state in file-scope statics and exposes `Init`, `Task` and narrow
getters. No shared context struct: cross-module reads go through getters, so no module can
reach into another's data. This was chosen over a single `BMS_t` context struct passed
everywhere, because that makes the struct the de-facto API and lets any module write any
other's fields.

Three interface rules follow from designing these as deep modules - a lot of behaviour behind a
small interface, testable *through* that interface rather than past it:

1. **Accept the buffers hardware fills; do not create them.** `ADC_Init(volatile uint16_t *dmaBuf)`
   takes its DMA buffer from `app.c`. This is what makes the module testable through its real
   interface - a host test supplies its own array, writes raw counts, calls `ADC_Task()` and reads
   the getters. The alternative, exposing `ntc_count_to_centi()` publicly, would widen the
   interface for no caller's benefit.
2. **Accept time; do not read the clock.** `app.c` calls `HAL_GetTick()` once per pass and passes
   `nowMs` to the two time-dependent modules. Tests drive time with no stubbing, and every module
   sees a consistent "now" within one iteration.
3. **Modules raise their own faults.** `app_fault` has zero dependencies on other modules;
   thresholds live next to the values they judge.

Shared timing helper, so the pattern is not repeated seven times:

```c
/* app_timing.h - true once per periodMs; advances *last by whole periods, wrap-safe */
bool Timing_Due(uint32_t *last, uint32_t periodMs);
```

### 3.2 Main loop

```c
void app_main(void)
{
    FAULT_Init();  LED_Init();      /* first, so everything later can report */
    ADC_Init();                     /* calibrate, start circular DMA */
    CAN_App_Init();                 /* standby LOW, filters, both buses, 21 TX frames */
    CONTACTOR_Init();               /* duty 0 %, state OPEN */
    JK_Init();  THERM_Init();

    for (;;) {
        uint32_t now = HAL_GetTick();       /* one consistent view of time per pass */

        ADC_Task();                         /* ISR-flag driven, no time dependency */
        if (Timing_Due(&thermTick, 1000u)) {
            THERM_Task();                   /* app.c owns the 1 Hz cadence */
        }
        JK_Task(now);
        CONTACTOR_Task(now);

        LED_Set(&ledRed, FAULT_Any() ? LED_ON : LED_OFF);
        LED_Handle(&ledRed);  LED_Handle(&ledGreen);

        CAN_App_Task();                     /* scheduler last: getData sees this pass's data */
    }
}
```

### 3.3 Boundary rules

1. **Only `app_can.c` transmits.** Other modules own data and expose getters; all 21 frame
   definitions and their `getData` callbacks live in one file.
2. **Generated headers stay contained.** Only `app_can.c` includes `CAN_DB.h`. `app_therm.c`
   includes `CAN2_DB.h` solely for `_Static_assert`s.
3. **Nothing exceeds 32 bits** (AGENTS.md rule 12). `Error_Specific_Data` is `uint8_t[5]`.

### 3.4 Interrupts

| IRQ | Priority | Work |
| --- | ---: | --- |
| `DMA1_Channel1` | 3 | `HAL_ADC_ConvCpltCallback` -> `adcConvCplt++` |
| `CAN2_RX0` | 2 | `GetRxMessage`, bounds check, `THERM_OnFrame(id, data[0])` |
| `CAN1_RX0` | 2 | `GetRxMessage`, `CONTACTOR_OnSafeStateFrame(id)` |
| `USART1` + DMA | 4 | `TxCpltCallback` -> RS485 turnaround; `RxEventCallback` -> length + flag |
| `SysTick` | 15 | `HAL_GetTick` only |

`HAL_CAN_RxFifo0MsgPendingCallback` is shared by both buses and **must** dispatch on
`hcan->Instance`.

### 3.5 Concurrency

No interrupt is ever masked. Two rules, and they differ for a reason:

- **ADC buffer: no critical section, and one would not work.** `__disable_irq()` does not stop
  the DMA controller. Halfword loads are atomic on Cortex-M3, so no value tears, and cross-scan
  skew between three independent signals is harmless. `volatile uint16_t adcBuf[3]`.
- **Thermistor arrays: no critical section needed either**, because the ISR performs only byte
  stores (`thermLatest`, `thermSeen`), which are atomic on M3. Ordering matters: `THERM_Task()`
  reads-and-clears `thermSeen[p][t]` *before* reading `thermLatest[p][t]`, so a sample arriving
  mid-sweep is still used and its flag survives to the next second.

Everything else is single-writer flags and 32-bit aligned timestamps.

### 3.6 Resource budget

| Item | RAM |
| --- | ---: |
| Thermistors (`latest`, `seen`, `window`, `filtered`, `miss`) | 882 B |
| JK (512 B RX buffer, 21 B request, decoded struct) | ~615 B |
| CAN scheduler, `CAN_MAX_MSG = 28` | ~1240 B |
| ADC (buffer + 3x10 window) | 66 B |
| **Total** | **< 3 KB of 64 KB** |

NTC lookup table: 202 B of flash as `const uint16_t[101]`.

## 4. Numeric conventions

Cortex-M3 has **no FPU**, so every `float` operation is a libgcc soft-float call.
**All arithmetic is scaled integer** (`int32_t`/`uint32_t`), and no soft-float is linked.

Rounding is explicit: `(a + b/2) / b` for unsigned, with the sign handled separately for
signed values. C integer division truncates toward zero, which would otherwise bias every
conversion downward by up to one output LSB.

This is not a precision compromise. IEEE754 single carries a 24-bit mantissa and loses bits on
every operation; scaled integers are exact until the final divide. Quantisation is set by the
ADC and the CAN encodings, never the arithmetic:

| Channel | ADC resolution | CAN encoding | Limiting factor |
| --- | --- | --- | --- |
| Pack voltage | 22.86 mV/count | 0.1 V | CAN, 4.4x coarser |
| Pack current | 0.25 A/count | 0.1 A | ADC, 2.5x coarser |
| Temperature | 0.025-0.125 degC/count | 0.01 degC | ADC |

`roundf()` replaces `round()` in `pwm_driver.c` (AGENTS.md rule 11), the only remaining
floating-point use.

## 5. Measurement pipeline

ADC1 scans PC0 (`TEMP`, IN10), PC1 (`HALL_OUT`, IN11), PC2 (`VOLTAGE`, IN12) continuously into
a circular DMA buffer. `HAL_ADCEx_Calibration_Start()` runs before the first conversion.

### 5.1 Filter

Per channel, a 10-deep rolling window with a **trimmed mean**: sum all ten, subtract the single
minimum and single maximum, divide by eight. This rejects a lone outlier completely rather than
diluting it. `ADC_Ready()` returns false until the window has filled once.

### 5.2 Conversions

```c
decivolts = (count * 228554u) / 1000000u;          /* max intermediate 936M, fits uint32 */
deciamps  = ((int32_t)count - 2108) * 5 / 2;       /* 0.25 A per count */
```

The `28.3626` divider ratio and the `2108` / `4` current calibration are **sensor-specific and
must be recalibrated on hardware** (`docs/adc.md`). Vref and divider tolerance dominate the
error budget by 20-100x over any arithmetic effect.

### 5.3 Temperature

The NTC is on the **high side**, with a fixed 10 k to ground: `Rt = 10000 x (Vcc/V - 1)`
inverts to `V = Vcc x 10000/(10000 + Rt)`. `docs/adc.md`'s prose ("the NTC on the low side")
contradicts its own formula and is wrong; the formula is right.

Bartek's validated resistance table (10.0 k at 25 degC, implied B(0/25) = 3297,
B(25/100) = 3441) is **inverted into a table of expected ADC counts**, monotonically increasing
1092 -> 3728, with integer interpolation:

```c
T_centi = i * 100 + ((count - ntcCount[i]) * 100) / (ntcCount[i+1] - ntcCount[i]);
```

Two consequences of indexing by count rather than resistance:

- **Vref cancels entirely.** The divider is ratiometric with VREF+ (internally tied to VDDA on
  LQFP64), so temperature becomes immune to supply tolerance. The float version computes voltage
  from a hardcoded `STM32_VCC` and inherits its full error.
- **808 B of RAM becomes 202 B of flash.** The original tables are `static float`, not `const`.

| Temperature | 0 | 25 | 60 | 100 degC |
| --- | ---: | ---: | ---: | ---: |
| Expected count | 1092 | 2048 | 3143 | 3728 |

**Sensor fault detection**, which the original cannot do because it clamps: count < 200 implies
an open NTC, count > 4000 a short. Both raise `TEMP_SENSOR_FAULT`. Readings between 200 and 1092
are reported as genuinely below 0 degC.

### 5.4 Range handling

Values outside the DBC range (63-87 V, +/-300 A, 0-100 degC) are detected with the generated
`<Signal>_is_in_range()` helpers, **clamped** to the range before packing, and raise
`PACK_VOLT_RANGE` or `PACK_CURRENT_HIGH` with the true unclamped value in
`Error_Specific_Data`. The bus never carries out-of-spec values.

## 6. Pack thermistor path

Seven `PCBCells` nodes each report nine thermistors on CAN2 at **1 Hz**, and the board
re-publishes them on CAN1 at 1 Hz. The output is a **transpose, not a spatial average**:
`BMSMaster_PCBsTherm<y>Temp` carries thermistor *y* from all seven packs.

Input and output encodings are byte-identical (`u8 x 0.39216 degC` over `[0..100]`), so
**aggregation happens in raw counts** and there is no degC domain, which removes a whole
quantisation step.

### 6.1 Frame identification

The ID map is **not monotonic**: packs 1/3/5/7 count up (211 = Therm1 ... 219 = Therm9) while
packs 2/4/6 count **down** (221 = Therm9 ... 229 = Therm1). A hand-written `id - base` decode is
wrong for three of seven packs and nothing looks broken.

```c
uint8_t pack   = (uint8_t)(id / 10u) - 20u;             /* 211..279 -> 1..7 */
uint8_t offset = (uint8_t)(id % 10u);                   /* 0 = NODE frame, ignore */
uint8_t therm  = (pack & 1u) ? offset : (10u - offset); /* even packs count down */
```

Verified against all 63 IDs, and backed by `_Static_assert` against the generated
`PCBCELLS<x>_THERM<y>_FRAME_ID` macros, so a database renumber **breaks the build** rather than
silently mismapping. `/10` and `%10` on a constant compile to a multiply-and-shift, not a
runtime divide.

### 6.2 Filter

One reading per thermistor per second, so nothing accumulates within a period. Per
(pack, thermistor), a **10-slot ring holding the last 10 seconds**, aggregated with a trimmed
mean (drop min and max, divide by `fill - 2`). Trimming applies once `fill >= 3`; below that a
plain mean over `fill`, so output is sensible from the first second.

The ring is advanced by the **1 Hz emit tick, not by arrival**, using one shared index, so a
pack that drops a frame cannot desynchronise its own history. A silent pack re-pushes its
previous value.

Worked example, one corrupt sample among ten:

```
64, 65, 64, 66, 65, 64, 65, 255, 64, 65      sum = 837
trimmed:  837 - (64 + 255) = 518 -> /8 = 65  -> 25.5 degC   (+0.1 degC vs baseline)
plain:    837 / 10               = 84        -> 32.9 degC   (+7.5 degC, for 10 s)
```

Goal is normalising jittery per-board readings and tolerating spikes, **not** thermal
protection. The filter is deliberately symmetric: no fast-attack asymmetry.

### 6.3 Silent packs

`thermSeen` clears each second. Three consecutive misses raise `CAN2_PACK_SILENT` with a
7-bit bitmap of silent packs. The value holds meanwhile: the `u8 x 0.39216` encoding spans
`[0..100]` with `255 = exactly 100.0 degC`, so **no code is free to mean "no data"** and
staleness must be reported out of band.

### 6.4 Overtemperature

`CAN2_TEMP_HIGH` at **60 degC** with a few degrees of clearing hysteresis, carrying pack index,
thermistor index and raw count.

## 7. JK BMS link

Half-duplex RS485 on USART1 via an SN65HVD72, 115200 8N1. Protocol V2.5
("Monitoring platform and BMS communication protocol"), big-endian throughout.

### 7.1 Frame format

| Offset | Field | Notes |
| --- | --- | --- |
| 0 | STX | `0x4E 0x57` |
| 2 | LENGTH | 2 bytes, = total - 2; includes itself and the checksum |
| 4 | Terminal ID | 4 bytes, `00 00 00 00` default |
| 8 | Command word | `0x01` activate, `0x03` read single, `0x06` read all |
| 9 | Frame source | `0x03` = PC upper computer |
| 10 | Transmission type | `0x00` request, `0x01` reply, `0x02` **unsolicited** |
| 11 | Payload | TLV stream: identifier byte + data |
| ... | Record number | 4 bytes |
| ... | End flag | `0x68` |
| ... | Checksum | 4 bytes: 2 zero (disabled CRC16 slot) + 2-byte accumulated sum over offset 0 through the end flag |

The payload is a **variable-length TLV walk**, not fixed offsets: `0x79` is itself
length-prefixed with 3 bytes per cell. Worst case ~339 bytes, so the RX buffer is **512 B**.
The BMS also sends unsolicited frames (type `0x02`), so the parser must not assume
request/response pairing. Unknown identifiers are skipped by a table-driven length map.

### 7.2 Protocol and transport are separate modules

`jk_protocol` is pure - no HAL, no state, no time - and holds the risky logic:

```c
uint16_t JKP_BuildRequest(uint8_t *out, uint8_t cmd);
bool     JKP_Validate(const uint8_t *buf, uint16_t len);
bool     JKP_Decode(const uint8_t *buf, uint16_t len, JK_Data_t *out);
```

Three functions hiding the accumulated checksum, `LENGTH` semantics, the length-prefixed `0x79`,
both `0x84` encodings, the sign negation and the flag curation. Apply the deletion test: remove
this module and the TLV walk reappears inside the transport, tangled with DMA state.

`app_jk` is then purely transport, and is where the RS485 sequencing lives.

### 7.3 Transport

Non-blocking state machine, so the CAN scheduler keeps exact cadence:

```
IDLE -> build request -> DE high, /RE high -> Transmit_DMA
     -> TxCpltCallback (fires on TC, last stop bit) -> DE low, /RE low
     -> ReceiveToIdle_DMA -> RxEventCallback gives exact length
     -> validate magic, LENGTH, end flag, checksum -> TLV walk -> store -> IDLE
```

Poll **1 Hz** with a **100 ms** timeout (against a 29.4 ms worst-case response). Command `0x01`
is sent once at startup and again only after a timeout, then the poll retried, since the spec
requires activation only when the BMS is asleep. Three consecutive failures raise
`JK_COMMS_TIMEOUT`; frame-level failures raise `JK_FRAME_INVALID`.

`0xC0` (protocol version) arrives free in every read-all response and **must be parsed before
current**, because it selects the `0x84` encoding and the two are indistinguishable at low
currents.

### 7.4 Signal mapping

| CAN signal | JK register | Conversion |
| --- | --- | --- |
| `JK_PackVoltage` (0.01 V) | `0x83` | direct; both are 10 mV/LSB |
| `JK_PackCurrent` (0.01 A) | `0x84` | `0xC0`=0: `10000 - raw`. `0xC0`=1: bit15 direction, bits14..0 magnitude in 10 mA. **Then negated** - see below |
| `JK_SOC` (%) | `0x85` | direct |
| `JK_SOH` (%) | `0xb9` / `0xaa` | **derived**: actual capacity / capacity setting x 100, clamped 0-100. The protocol has no SOH register. |
| `JK_Cell1..21_mV` | `0x79` | direct, mV; cell count = length / 3 |
| `JK_MosTemp` | `0x80` | `v > 100 ? -(v - 100) : v` |
| `JK_BalTemp` | `0x81` | battery-box temperature, same decode |
| `JK_Cycles` | `0x87` | direct |
| `JK_CellCount` | `0x8a` | direct, true runtime count |
| `JK_ModeFlags` | `0x8c` | low byte; only bits 0-3 are defined |
| `JK_StatusFlags` | `0x8b` | **curated 8-bit summary** - see below |

**Current sign convention.** The JK protocol yields positive = charging, while
`docs/notionSpec.md` defines the ADC current as positive = **discharging**. The JK value is
therefore **negated** so both signals use positive = discharging.

The reason is not merely consistency: the two signals measure the *same physical current by
different means* - the Hall sensor on PC1 and the JK's internal shunt - so matching signs makes
them directly comparable, which is a usable cross-check for a drifting Hall sensor. Opposite
signs would make that comparison silently wrong.

The convention is recorded in the database itself, as `CM_` comments on both
`BMSMaster_MasterBatteryCurrent` and `BMSMaster_JK_PackCurrent` (PR #46), so it is authoritative
rather than an undocumented firmware choice. The old `stm_bms-master` sets no precedent: it never
implemented the JK link at all.

**Warning flags.** `0x8b` defines 14 bits but `JK_StatusFlags` is `u8`. Truncating to the low
byte would silently drop b10/b11, monomer over- and under-voltage, which are the cell-level
protections that matter most. A category summary preserves all of them:

| Bit | Meaning | Source bits |
| ---: | --- | --- |
| 0 | over-voltage | b2, b10 |
| 1 | under-voltage | b3, b11 |
| 2 | over-temperature | b1, b4, b8 |
| 3 | low temperature | b9 |
| 4 | over-current | b5, b6 |
| 5 | cell pressure difference | b7 |
| 6 | low capacity | b0 |
| 7 | 309_A / 309_B protection | b12, b13 |

### 7.5 Cell count

The pack is **21S**, confirmed arithmetically: 63 V / 21 = 3.0 V and 87 V / 21 = 4.14 V per
cell, a standard Li-ion range, whereas 12 cells would require an impossible 5.25-7.25 V. The
database originally carried only 12 cell slots. `Eko-Energia/CAN-DATABASE` PR **#46** adds the
rest:

| ID | Frame |
| ---: | --- |
| 141-143 | `Cells_1_4`, `Cells_5_8`, `Cells_9_12` (unchanged) |
| 144 | `BMSMaster_JK_Cells_13_16` (new) |
| 145 | `BMSMaster_JK_Cells_17_20` (new) |
| 146 | `BMSMaster_JK_Cells_21` (new, DLC 2) |
| 147 | `BMSMaster_JK_Temp` (moved from 144) |
| 148 | `BMSMaster_JK_CycleStats` (moved from 145) |

Until #46 merges, generated sources come from the `BMSMaster/21-cells` branch. Once merged, the
submodule pointer is bumped and the sources regenerated to confirm no diff. A JK-reported count
above 21 raises a fault, since those cells would be invisible to the vehicle.

### 7.6 Link loss

After three failed polls the six JK frames are transmitted with **zeroed payloads** and
`JK_COMMS_TIMEOUT` is raised. The cycle time is preserved so consumers watching cadence are
unaffected.

## 8. Contactor and safe state

`PB0` / TIM3_CH3 at 1 kHz. **100 % duty for 2 s on every transition to closed**, then 50 % to
hold - a coil pull-in requirement, not a startup formality. Duty is 0 % whenever open.

Safe-state frames arrive on **CAN1**, per `notionSpec` (which captions the SafeState screenshot
`CAN_DB.dbc`) and per the database split. `docs/can.md`'s claim that the reference filtered
ID 1 on CAN2 describes superseded code and is corrected.

`SafeState_Activ` (ID 1) has **no signals**: its presence is the entire message, and
`CM_ BO_ 1` says "Frame from all PCBs", so any node may assert it.

```
BOOT            -> OPEN
first SafeState_NODE (ID 3) received -> latched permanently, closure permitted
SafeState_Activ (ID 1) received      -> OPEN
300 ms with no SafeState_Activ       -> CLOSED (2 s at 100 %, then 50 %)
```

The "NODE received" precondition is a **one-time latch**, and there is **no heartbeat
supervision**. This is the literal reading of `notionSpec` and was chosen deliberately over a
fail-safe variant that opens the contactor when the SafeState node goes silent. The 300 ms
window is a single named constant so the policy is visible in one place.

Note the timing evidence: `SafeState_NODE` cycles at 5000 ms per the DBC, so the 300 ms window
can only apply to `SafeState_Activ`. `SafeState_Activ` carries two contradictory
`GenMsgCycleTime` attributes (`5000` and `0`), so the database cannot say whether it is periodic
or event-driven.

## 9. CAN configuration

### 9.1 Filters

STM32F105 has 28 filter banks shared between the peripherals; `SlaveStartFilterBank = 14`.

- **CAN1**: one bank, 16-bit list mode, accepting exactly IDs **1** and **3**.
- **CAN2**: five banks, 32-bit mask mode, mask `0x7F0`, covering `0x0D0`-`0x11F` (208-287) —
  the range 210-279 is not cleanly maskable — plus a software bounds check rejecting 208, 209,
  280-287 and the `NODE` offsets.

### 9.2 Transmit schedule

All periods come from `GenMsgCycleTime` in the database. **21 frames**, so
`CAN_MAX_MSG` is raised from 20 to **28**.

| ID | Frame | DLC | Period |
| ---: | --- | ---: | ---: |
| 128 | `BMSMaster_NODE` | 8 | 5000 ms |
| 130 | `BMSMaster_MasterVoltCurrTemp` | 6 | 500 ms |
| 131-139 | `BMSMaster_PCBsTherm1..9Temp` | 7 | 1000 ms |
| 140 | `BMSMaster_JK_Pack` | 8 | 1000 ms |
| 141-146 | `BMSMaster_JK_Cells_*` | 8 / 2 | 1000 ms |
| 147 | `BMSMaster_JK_Temp` | 8 | 1000 ms |
| 148 | `BMSMaster_JK_CycleStats` | 8 | 1000 ms |
| 159 | `BMSMaster_END` | 8 | 1000 ms |

`BMSMaster_END` has no signals defined; it is transmitted as 8 zero bytes to honour the cycle
time. Total offered load is ~21 frames/s, about **0.3 %** of a 500 kbit/s bus.

### 9.3 Driver corrections

`EKO_Drivers/CAN/can_driver.c` is kept — it is a newer fork than the one in `stm_dashboard`,
which has a tick-wrap bug, drifting cadence and a `return` that aborts the whole scheduler on a
failed enqueue. These defects remain and are fixed here:

| Defect | Fix |
| --- | --- |
| `CAN_Init()` hardcodes `FilterBank = 0` and an accept-all mask | Parameterised filter setup. A CAN2 bank **must** be `>= SlaveStartFilterBank`; bank 0 belongs to CAN1, so `CAN_Init(&hcan2)` currently programs a CAN1 bank. |
| `count` and `receiveFlag` are non-`volatile`, and `count++` in the ISR races `count--` in the main loop | `volatile`, with single-producer/single-consumer discipline: `head` owned by the ISR, `tail` by the main loop, no shared counter. |
| `CAN_AddScheduledMsg()` does not null-check `msg` or `buffer` | Add the checks. |
| `readme.md` documents `CAN_AUTO_RETRANSMISSION` / NART handling inside `CAN_Init()` that is absent from the code | **Delete the claim and the macro.** CubeMX's `AutoRetransmission = DISABLE` stands, which the readme itself argues is correct for periodic status frames - a failed frame is dropped immediately and can never block a mailbox. Every frame this board sends is periodic. |
| `CAN_GetLatestMessage()` is documented as returning the lowest CAN ID but is FIFO by `tail` | Correct the documentation. |

## 10. Faults and annunciation

Severity encoding: **0 = safe state, 1 = error, 2 = warning, 3 = info**. Normal operation is
`Error_Code = 0`, `Severity = 3`.

| Code | Name | Severity | `Error_Specific_Data` |
| ---: | --- | --- | --- |
| 1 | `BMS_TEMP_HIGH` | error | temperature, centi-degC `u16` |
| 2 | `CAN2_TEMP_HIGH` | error | pack `u8`, thermistor `u8`, raw count `u8` |
| 3 | `CAN2_PACK_SILENT` | error | bitmap of silent packs `u8` |
| 4 | `JK_COMMS_TIMEOUT` | error | consecutive failures `u8` |
| 5 | `JK_FRAME_INVALID` | warning | reason code `u8` |
| 6 | `PACK_VOLT_RANGE` | error | decivolts `u16` |
| 7 | `PACK_CURRENT_HIGH` | error | deciamps `i16` |
| 8 | `TEMP_SENSOR_FAULT` | error | raw count `u16` |
| 9 | `CAN1_TX_FAIL` | warning | frame ID `u16` |

Codes 1 and 2 come from the team's CSV registry. Codes 3-9 are allocated here and must be
added to it. **No fault emits severity 0**, which would command vehicle-wide safe state; the
CSV grades codes 1 and 2 as `ERROR`, and the old implementation's use of
`ERROR_SEVERITY_SAFE_STATE` for `BMS_TEMP_HIGH` is not followed.

State is a 16-bit active-fault mask plus a 5-byte blob per code. Because `BMSMaster_NODE`
carries a single `Error_Code`, the frame reports the **lowest-numbered active fault**, making
the CSV ordering the priority order. `Node_Execution_Halted` is set when measurement is invalid
or the contactor state machine cannot be trusted.

`Error_Handler()` opens the contactor (duty 0 %), turns `RED_LD` on, then keeps CAN1
transmitting `BMSMaster_NODE` with the fault and `Node_Execution_Halted = 1`, so the vehicle
learns *why* the board stopped rather than only that it vanished. If CAN1 itself failed to
initialise it falls back to LED-and-trap. There is no watchdog and no software reset, by
decision.

LEDs: `GREEN_LD` blinks at 1 Hz to prove the loop is alive; `RED_LD` is solid whenever the
fault mask is non-zero.

## 11. Stated assumptions and open items

| Item | Status |
| --- | --- |
| `RS_DIR` -> `DE` (active high), `RE_DIR` -> `/RE` (active low) | **Confirmed** by Bartek on 2026-09-11, agreeing with the inference from the SN65HVD72 pinout and a boot state of both LOW = listen. Kept as named constants, and bring-up step 6 still puts a scope on PC4/PC5 - a confirmation from memory is not a traced schematic, and the same class of inference proved wrong for the CAN standby pins. |
| Error codes 3-9 | Allocated here; must be added to the team CSV registry. |
| CAN-DATABASE PR #46 | Open. Bump the submodule and regenerate once merged. |
| ADC calibration constants | `28.3626` divider and `2108` / `4` current values ship as named defines marked uncalibrated, and are corrected at bring-up step 3. |
| `HVIL`, fan, radio, watchdog, bus-off recovery | Deferred by decision - section 1. |

## 12. Verification

**Host unit tests** for the logic where a bug yields plausible-looking output rather than an
obvious failure. A `tests/` directory with a plain Makefile and an assert-based runner, no
framework dependency:

Tests exercise the modules **through their real interfaces**, not through internals exposed for
testing. Nothing is added to a public interface for a test's benefit.

| Seam | Covered by driving it |
| --- | --- |
| `app_adc`, via the injected DMA buffer | NTC LUT and interpolation, trimmed mean, all three conversions, DBC clamping, open/short fault bands |
| `app_therm`, via `THERM_OnFrame()` + `THERM_Task()` | the non-monotonic ID map including an **even** pack, trimmed mean, warm-up below `fill = 3`, silent-pack detection at three misses |
| `jk_protocol`, directly - it is already pure | accumulated checksum, `LENGTH` semantics, TLV walk, length-prefixed `0x79`, both `0x84` encodings, sign negation, flag curation |

**On target**, via `stm32-mcp`: `stm32_build_and_flash`, then `stm32_read_memory` on ELF symbols
(`thermFiltered`, `thermMiss`, `faultMask`, ADC outputs) and `live_memory_start` to watch them
move. There is no debug UART - USART1 is the JK link - so SWD is the only channel.

**Bring-up order**, sequenced to isolate the unknowns:

1. Green LED blinking - proves the loop runs and HSE started. A dark LED localises a clock
   failure immediately, since a failed HSE traps in `Error_Handler`.
2. CAN1 TX on an analyser at 500 kbit/s - confirms bit timing and the standby pins together.
3. ADC values over SWD against a multimeter on PC0/PC1/PC2 - calibrates section 5.2.
4. PB0 on a scope - 1 kHz, 100 % for 2 s, then 50 %.
5. CAN2 injection, watching `thermFiltered` - verify an **even-numbered** pack specifically.
6. JK link last, since it carries the remaining unverified assumption.

**Build hygiene:** `-Wall -Wextra -Werror` with no warnings, plus a `cppcheck` pass.

## 13. Documentation to update

Per AGENTS.md rule 7:

| File | Change |
| --- | --- |
| `docs/adc.md` | NTC is on the **high** side; the prose contradicts the formula. Add the count-indexed LUT. |
| `docs/can.md` | SafeState is on **CAN1**, not CAN2. Update the frame table for 21 cells and IDs 147/148. Correct the `CAN_MAX_MSG` figure (32 -> 28) and the `CAN_GetLatestMessage()` description. |
| `docs/projectOverview.md` | Clock is 72 MHz from an external oscillator in BYPASS, not 36 MHz HSI; ADC is PCLK2/8. |
| `docs/pwmGeneration.md` | `tim.c` has **no** USER CODE override to PWM1; the `.ioc` is changed instead. |
| `docs/bmsJk.md` | The JK link is **USART1** on PA9/PA10, not USART2 on PA2/PA3. |
| `docs/canDatabase.md` | Generated sources live in `EKO_Drivers/CAN/Inc` + `Src` and are committed; document the move step and PR #46. |
| `docs/pcb.md` | `PD1-OSC_OUT` is not used; note the ADC sampling time and the standby pin polarity. |
| `docs/index.md` | Add this specification. |

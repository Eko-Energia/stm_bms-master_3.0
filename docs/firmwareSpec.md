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
| `PCBCells<x>_NODE` frames (210, 220 ... 270) | Not consumed. They carry each module's own `Error_Code` and `Severity`, but a silent module is already detected from thermistor absence (section 6.3), which is what this board needs. Consequence: a module reporting its own fault while still sending thermistor data goes unnoticed here. |
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

The PCB routes **only** `OSC_IN`; there is no `OSC_OUT`, which is why BYPASS is correct. In
bypass mode the oscillator amplifier is disabled and PD1 would be free as GPIO, but the `.ioc`
keeps `PD1-OSC_OUT` **assigned and reserved**: it generates no code (CubeMX emits no GPIO init
for OSC pins), nothing needs the pin, and if a crystal is ever fitted the switch to
`RCC_HSE_ON` is then a single change.
72 MHz is unreachable from HSI on this part (`IS_RCC_PLL_MUL` permits only x4..x9 and x6.5,
and the HSI path is a fixed 4 MHz), so HSE is mandatory.

### 2.1 Required `.ioc` changes

Application code never lives in generated files, so these are made in CubeMX and regenerated.

| Change | Reason |
| --- | --- |
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

**Comment discipline.** Keep a comment as short as it can be while still earning its place
(AGENTS.md rule 10), and say *why* rather than *what* - the code already says what.

A trailing comment is a few words. Two or three lines above the code is fine, and sometimes
right, when the point is genuinely non-obvious: a hardware quirk ("NART is only writable between
`HAL_CAN_Init()` and `HAL_CAN_Start()`"), an invariant worth proving (`count` capped at 255 so
the `uint16_t` sum cannot overflow), or a trap like the even-numbered packs numbering downward.
Those save the next reader real time.

What is not wanted is paragraphs, narrative, or restating the function name as prose. Rule of
thumb: if the comment is longer than the code it describes, it probably belongs in this document
instead - which is why the spec records rejected alternatives, so the source does not have to.

Bare-metal cooperative superloop. **No RTOS**: no tasks, no queues, no scheduler, no blocking
calls. Cadence comes from `HAL_GetTick()` comparisons and the CAN transmit scheduler.

```
App/Inc/    app.h  app_adc.h  app_can.h  app_contactor.h  app_thermal.h
            app_jk.h  app_therm.h  app_timing.h  jk_protocol.h
            bms_errors.h  bms_calib.h
App/Src/    app.c  app_adc.c  app_can.c  app_contactor.c  app_thermal.c
            app_jk.c  app_therm.c  jk_protocol.c  bms_calib.c
EKO_Drivers/CAN/              Inc/{can_driver,CAN_DB,CAN2_DB}.h  Src/{can_driver,CAN_DB,CAN2_DB}.c
EKO_Drivers/LED/              Inc/led_driver.h    Src/led_driver.c      (imported)
EKO_Drivers/Error_Corrutines/ Inc/error_handler.h Src/error_handler.c   (imported)
```

`main.c` gains three lines, all inside `USER CODE` sections: `#include "app.h"`,
`app_main();` in `USER CODE BEGIN 2`, and `App_OnFatalError();` in
`USER CODE BEGIN Error_Handler_Debug`.

`app.c` holds **no logic** - init order, ISR dispatch, the `HAL_IncTick`
override and three lines of LED policy. Everything that decides anything lives
in a module that can be tested. That is why there are **no test-only symbols
anywhere in the project**.

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
   interface - a host test supplies its own array, writes raw counts, calls `ADC_Task(nowMs)` and
   reads the getters. The alternative, exposing `ntc_count_to_centi()` publicly, would widen the
   interface for no caller's benefit.
2. **Accept time; do not read the clock.** `app.c` calls `HAL_GetTick()` once per pass and passes
   `nowMs` to the three time-dependent modules. Tests drive time with no stubbing, and every module
   sees a consistent "now" within one iteration. This holds in interrupt context too: the
   contactor's RX handler sets a flag and `CONTACTOR_Task(now)` timestamps it, rather than
   reaching for the clock where no tick can be passed in.
3. **Modules raise their own faults** through `EH_reportEx()`, so thresholds live next to the
   values they judge rather than in a central evaluator.
4. **Constants are private, with one deliberate exception.** Behavioural numbers - window
   lengths, the 300 ms safe-state window, the 2 s pull-in, poll rates, timeouts, temperature
   limits - are `#define`s at the top of the owning module's `.c`, invisible to callers. But
   anything **measured from hardware** goes in `App/Inc/bms_calib.h`: see section 5.3.

Shared timing helper, so the pattern is not repeated seven times:

```c
/* app_timing.h - true once per periodMs; advances *last by whole periods, wrap-safe */
bool Timing_Due(uint32_t now, uint32_t *last, uint32_t periodMs);
```

### 3.2 Main loop

```c
static struct LED       ledGreen = { LED_BLINK, GREEN_LD_GPIO_Port, GREEN_LD_Pin };
static struct LED       ledRed   = { LED_OFF,   RED_LD_GPIO_Port,   RED_LD_Pin   };
static EH_HandleTypeDef eh;

void app_main(void)
{
    CAN_App_Init();
    EH_init(&eh, &hcan1, BMSMASTER_NODE_FRAME_ID, &canScheduler);
    ADC_Init(adcBuf);
    CONTACTOR_Init();
    JK_Init();
    THERM_Init();
    LED_ChangeState(&ledGreen, LED_BLINK);

    for (;;) {
        uint32_t now = HAL_GetTick();

        ADC_Task(now);
        if (Timing_Due(now, &thermTick, 1000u)) {
            THERM_Task();
        }
        JK_Task(now);
        CONTACTOR_Task(now);
        THERMAL_Evaluate(ADC_Ready(), ADC_TempCenti(),
                         THERM_MaxRaw(), THERM_MaxModule(), THERM_MaxTherm());

        LED_STATE_e want = (eh.activeErrorCount > 0u) ? LED_ON : LED_OFF;
        if (ledRed.state != want) {
            LED_ChangeState(&ledRed, want);
        }
        LED_Handle(&ledGreen);
        LED_Handle(&ledRed);

        CAN_App_Task();     /* last: getData sees this pass's data */
    }
}
```

`CAN_App_Init()` comes **first** because `EH_init` needs the scheduler and `hcan1` to register
the node frame. Faults are therefore reportable from the second line onward, which is why ADC
and the rest follow it.

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

Measured on the Debug target build (`arm-none-eabi-size`, `arm-none-eabi-nm --size-sort -S`
against `Debug/BMS-Master.elf`), not estimated. The previous table under-counted by omitting
the error handler, the CubeMX peripheral handles, and the linker's heap/stack reservation -
every line it did budget came in at or under its estimate.

| Item | RAM |
| --- | ---: |
| Thermistors (`latest`, `seen`, `window`, `filtered`, `miss`) | 882 B |
| JK (512 B RX buffer, 21 B request, decoded struct, link state) | 617 B |
| CAN scheduler, `CAN_MAX_MSG = 28` (20 app frames + NODE/heartbeat) | 1240 B |
| ADC (3x10 window, DMA buffer, packed state) | 83 B |
| Error handler (`EH_HandleTypeDef eh`) | 180 B |
| CubeMX peripheral handles (`huart1` 72, `htim3` 72, 3x DMA 68 each, `hcan1`/`hcan2`/`hadc1`) | 476 B |
| Misc application statics (LED, contactor, CAN/node glue) - remainder, not a direct `nm` figure | 186 B |
| Heap + stack reservation (`_Min_Heap_Size` + `_Min_Stack_Size`) | 1536 B |
| **Total (`.data` + `.bss`)** | **5200 B (5.08 KiB) of 64 KB** |

Flash: 39068 B (38.15 KiB) of 64 KB (`.text` 39044 B + `.data` 24 B).

NTC lookup table: 202 B of flash as `const uint16_t[101]`.

Release build (`arm-none-eabi-size Release/BMS-Master.elf`, `-Wall -Wextra -Werror`, same
sources): `.text` 21744 B + `.data` 24 B = **21768 B flash** (21.26 KiB) of 64 KB; `.data` 24 B +
`.bss` 5152 B = **5176 B RAM** (5.05 KiB) of 64 KB. The RAM difference from Debug (5200 B) is
noise-level (24 B) - the same statics, just without debug-build padding/inlining differences in
`.bss` layout.

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
diluting it. `ADC_Ready()` returns false until the window has filled once, **and again once the
conversion stream stalls**: the DMA is a bus master, so if it stops nothing else notices and the
last values would be published and judged forever. `ADC_Task` records the tick of each completed
scan; 100 ms without one (about 1200 missed conversions) raises `ADC_STALLED` and drops
`ADC_Ready()`, so `THERMAL_Evaluate` stops judging frozen data. The next completed scan clears
both. This is a **separate code from `TEMP_SENSOR_FAULT`** on purpose: an open or shorted NTC
sends a technician to the sensor and its wiring, a dead conversion stream sends them to the MCU,
and the two have nothing in common from that end.

### 5.2 Conversions

```c
decivolts = (count * 228554u) / 1000000u;          /* max intermediate 936M, fits uint32 */
deciamps  = ((int32_t)count - 2108) * 5 / 2;       /* 0.25 A per count */
```

The `28.3626` divider ratio (`CALIB_PACK_V_NUM`/`CALIB_PACK_V_DEN` = 228554/1000000) and the
`2108` offset / `5÷2` current gain are **sensor-specific and must be recalibrated on hardware**
(`docs/adc.md`). Vref and divider tolerance dominate the error budget by 20-100x over any
arithmetic effect.

### 5.3 Calibration constants live in a header

Everything that has to be **measured on hardware** lives in `App/Inc/bms_calib.h`, with the NTC
table in `App/Src/bms_calib.c`:

```c
/* Measured on hardware. Bring-up step 3. */
#define CALIB_PACK_V_NUM      228554u   /* decivolts per ADC count, x1e6 */
#define CALIB_PACK_V_DEN      1000000u
#define CALIB_CURRENT_OFFSET  2108      /* ADC count at 0 A */
#define CALIB_CURRENT_NUM     5         /* deciamps = (count - offset) * NUM / DEN */
#define CALIB_CURRENT_DEN     2

extern const uint16_t calibNtcCount[101];   /* expected ADC count per degC, 0..100 */
```

This is a deliberate exception to rule 4 above. These are not behavioural choices but a
*characterisation of this board*, and the people who change them - someone at bring-up with a
multimeter, or after a board revision - need to find them without reading implementation. One
file pair is the whole surface they have to touch.

The values shipped are the reference ones from `docs/adc.md` and Bartek's table, and are
**uncalibrated**; the header says so. The table is `extern` rather than defined in the header so
only one translation unit carries the 202 bytes.

### 5.4 Temperature

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
an open NTC, count > 4000 a short. Both raise `TEMP_SENSOR_FAULT`.

Readings between 200 and 1092 are genuinely below 0 degC but **cannot be reported**: the DBC
signal is `32|16@1+`, unsigned over `[0|100]`, so the frame has no room for a negative value.
They clamp to 0.00 degC. Distinguishing sub-zero from exactly-zero would need a signed signal
upstream; the fault bands above still catch an open or shorted sensor.

### 5.5 Range handling

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
module that drops a frame cannot desynchronise its own history. A silent module re-pushes its
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

`thermSeen` clears each second. Three consecutive misses raise `CAN2_MODULE_SILENT` with a
7-bit bitmap of silent modules. The value holds meanwhile: the `u8 x 0.39216` encoding spans
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

The reply to `0x01` is **acknowledged, never published**. It is a well-formed, checksum-valid
frame with no data TLVs, so it decodes as a valid all-zero `JK_Data_t`; accepting it as a reading
would put 0 % SOC and 21 cells at 0 mV on the bus as healthy and clear `JK_COMMS_TIMEOUT`, at
every boot and every reconnect. The transport remembers which command is in flight and treats an
activation reply as "the BMS is awake" only; the read-all that follows is what publishes.

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

**Battery topology, confirmed 2026-09-11:** seven modules of **3S5P**, giving **21S** overall.
Each packet holds 15 physical cells (3 series x 5 parallel); the battery holds 105. Parallel
cells share a node and self-balance, so the 21 series taps are the complete measurement set -
105 cells, 21 measured values.

This agrees with the arithmetic: 63 V / 21 = 3.0 V and 87 V / 21 = 4.14 V per cell, a standard
Li-ion range, whereas 12 cells would require an impossible 5.25-7.25 V and 15 would require
4.2-5.8 V.

The seven modules are the seven `PCBCells<x>` boards on CAN2, one per module - so the thermistor
topology (7 x 9) and the cell count (21S) describe the same seven modules from different angles,
and neither number constrains the other.

> **Terminology.** A **module** is one of the seven 3S5P groups; the **pack** is the whole
> 21S battery. The code follows this: `THERM_MODULES = 7` and `THERM_Filtered(module, therm)`
> address one module, while `ADC_PackDecivolts()` measures the battery. "Packet" is avoided -
> in a CAN codebase it reads as a frame.

The database originally carried only 12 cell slots. `Eko-Energia/CAN-DATABASE` PR **#46** adds the
rest:

| ID | Frame |
| ---: | --- |
| 141-143 | `Cells_1_4`, `Cells_5_8`, `Cells_9_12` (unchanged) |
| 144 | `BMSMaster_JK_Cells_13_16` (new) |
| 145 | `BMSMaster_JK_Cells_17_20` (new) |
| 146 | `BMSMaster_JK_Cells_21` (new, DLC 2) |
| 147 | `BMSMaster_JK_Temp` (moved from 144) |
| 148 | `BMSMaster_JK_CycleStats` (moved from 145) |

PR #46 is **merged**. The submodule now pins `master` (`60ab52e`) and the sources were
regenerated: the output is byte-identical apart from the generator's banner, confirming the
merged `master` and the branch agree for this node.

A JK-reported count **above** 21 raises a fault, since those cells would be invisible to the
vehicle. A count **below** 21 is accepted, and the absent cells publish **0 mV** - confirmed as
the intended convention on 2026-09-11. Zero is safe as a sentinel because no real cell can read
0 V, and it is the link-down value too, so a consumer treating 0 as "no valid reading" is
correct in both cases. Consumers that need the reason have it: `JK_CellCount` on frame 148 gives
the real count, and `JK_COMMS_TIMEOUT` fires when the link is down.

### 7.6 Link loss

After three failed polls all **nine** JK-sourced frames (140, 141-146, 147, 148 - six cell
frames after PR #46's split, not the three it had before) are transmitted with **zeroed
payloads** and `JK_COMMS_TIMEOUT` is raised. The cycle time is preserved so consumers watching
cadence are unaffected.

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

`app_can` registers **20** of them. `EH_init` registers frame 128 itself, which
is why `CAN_App_Init` runs before it and exposes `CAN_App_Scheduler()`.

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

`EKO_Drivers/CAN/can_driver.c` is kept and brought back in line with the canonical driver in
`Eko-Energia/stm_drivers`. The diff against canonical is confined to **two** functions,
`CAN_Init` and `CAN_HandleScheduled`: `CAN_AddScheduledMsg`, `CAN_AddIncomingMsg` and
`CAN_GetLatestMessage` are byte-identical. So the wrap-safe tick, drift-free cadence and the `CAN_TX_FAIL_LIMIT` recovery
are **canonical features, not local improvements** - the stale copy is the one in
`stm_dashboard`.

`CAN_HandleScheduled` is the one exception, and deliberately so. Canonical re-armed `lastTick`
on a failed enqueue, which costs the frame a whole period. With three mailboxes and ~18 frames
sharing a 1000 ms cycle that starved everything past the third: five frame ids never
transmitted at all and the NODE frame managed 3 of 12 over a simulated minute. The frame now
stays due and retries on the next pass, and the fail counter counts missed **periods** rather
than passes so ordinary sub-millisecond contention cannot trip the abort. **This fix belongs
upstream in `stm_drivers`** - every board using the canonical driver has the same defect.

Our local copy is canonical minus the `CAN_Init` body. These are the changes:

| Defect | Fix |
| --- | --- |
| `CAN_Init()` hardcodes `FilterBank = 0` and an accept-all mask | Parameterised filter setup. A CAN2 bank **must** be `>= SlaveStartFilterBank`; bank 0 belongs to CAN1, so `CAN_Init(&hcan2)` currently programs a CAN1 bank. |
| `count` and `receiveFlag` are non-`volatile`, and `count++` in the ISR races `count--` in the main loop | `volatile`, with single-producer/single-consumer discipline: `head` owned by the ISR, `tail` by the main loop, no shared counter. |
| `CAN_AddScheduledMsg()` does not null-check `msg` or `buffer` | Add the checks. Present in canonical too. |
| The NART / `CAN_AUTO_RETRANSMISSION` block that `readme.md` documents is missing from our copy | **Restore it from canonical.** The readme is not wrong - our copy was stripped. Canonical writes NART between `HAL_CAN_ConfigFilter()` and `HAL_CAN_Start()`, the only window where the bit is writable, and mirrors it into `Init.AutoRetransmission` so a later `HAL_CAN_Init()` cannot silently revert it. Keep `CAN_AUTO_RETRANSMISSION = 0`, giving retransmission **off** - correct for periodic frames, since a failed frame is dropped at once and can never block a mailbox. |
| Canonical `CAN_Init` returns `void` and calls `Error_Handler()` on failure | **Keep our `HAL_StatusTypeDef` return.** A driver must not trap: our `Error_Handler` opens the contactor and keeps the NODE frame alive, which is application policy the caller owns. Keep our `hcanPtr` NULL check and `CAN_FilterTypeDef filterConfig = {0}` too - canonical has neither. |
| `CAN_GetLatestMessage()` is documented as returning the lowest CAN ID but is FIFO by `tail` | Correct the documentation. Present in canonical too. |

The filter-bank and `volatile` defects are **upstream bugs in `stm_drivers`**, not local damage.
The filter one only bites a dual-CAN board, which is likely why it has gone unnoticed. Both
warrant a pull request against `stm_drivers` so the next board does not inherit them.

## 10. Faults and annunciation

Fault handling uses the team's **`Error_Corrutines`** driver (`EH_*`), vendored into
`EKO_Drivers/Error_Corrutines/`. The old `stm_bms-master` used it, and it already implements
what an application-side registry would otherwise reinvent: the 5-byte `Error_Specific_Data`
blob (`ERROR_SPECIFIC_DATA_SIZE`), the `severity:3` / `halted:1` / `reserved:4` bitfield that is
exactly the `BMSMaster_NODE` layout, a 16-entry active-error set, and registration of the frame
with the CAN scheduler. `errorFrameId = nodeId` directly, so
`EH_init(&eh, &hcan1, 128, &canScheduler)` produces ID 128 with no patching.

Severity comes from the driver's `errorSeverity_e`: **0 = safe state, 1 = error, 2 = warning,
3 = info**. Normal operation is the driver's heartbeat: `Error_Code = HEARTBEAT_ERROR_CODE`
(`0xFFFF`), `Severity = 3`. The driver reserves `0xFFFF` for it, so no real fault may use that
value.

| Code | Name | Severity | `Error_Specific_Data` |
| ---: | --- | --- | --- |
| 1 | `BMS_TEMP_HIGH` | error | temperature, centi-degC `u16` |
| 2 | `CAN2_TEMP_HIGH` | error | pack `u8`, thermistor `u8`, raw count `u8` |
| 3 | `CAN2_MODULE_SILENT` | error | bitmap of silent modules `u8` |
| 4 | `JK_COMMS_TIMEOUT` | error | consecutive failures `u8` |
| 5 | `JK_FRAME_INVALID` | warning | reason code `u8` |
| 6 | `PACK_VOLT_RANGE` | error | decivolts `u16` |
| 7 | `PACK_CURRENT_HIGH` | error | deciamps `i16` |
| 8 | `TEMP_SENSOR_FAULT` | error | raw count `u16` |
| 9 | `CAN1_TX_FAIL` | warning | frame ID `u16` |
| 10 | `BMS_ERR_FATAL_INIT` | error | none used |
| 11 | `ADC_STALLED` | error | ms since the last completed ADC scan `u16` |

Codes 1 and 2 come from the team's CSV registry; 3-11 are allocated here and must be added to
it. The codes live in one header of constants, `App/Inc/bms_errors.h` - not a module. Each
module reports its own faults via `EH_reportEx(&eh, code, severity, data, len)` and clears them
with `EH_clear()`, so thresholds sit next to the values they judge.

Two exceptions, both because the judgement spans more than one module:

- **Codes 1 and 2**, the 60 degC limits, are evaluated by `app_thermal`:
  `THERMAL_Evaluate(boardValid, boardCenti, packMaxRaw, packModule, packTherm)`. The two index
  arguments come from `app_therm`'s `THERM_MaxModule()`/`THERM_MaxTherm()` getters, which keep the
  argmax of the same sweep that produced `THERM_MaxRaw()` - code 2's payload has to name the
  thermistor, and the max alone cannot. It compares the on-board NTC against
  the hottest pack thermistor, so it belongs to neither `app_adc` nor `app_therm`. It takes
  values rather than calling getters, which is what keeps it testable without a fake HAL - and
  what keeps `app.c` free of logic. Hysteresis: 60 degC to raise, 57 degC to clear on the board
  sensor; raw 153 to raise, 145 to clear on the pack, at 0.39216 degC per count.
- **Code 9** `CAN1_TX_FAIL` is also raised by `app_can` when `CAN_AddScheduledMsg` rejects a
  frame at init, since a frame that never registered will never transmit.
- **Code 10** `BMS_ERR_FATAL_INIT` is raised by `App_OnFatalError()` (`app.c`), the `main.c`
  fallback called from `Error_Handler()`. It exists because, without it, `App_OnFatalError` had
  no code of its own and reported an unrelated CAN fault code even when the failure was an ADC
  or other peripheral init failure - code 10 names the failure for what it is: reaching
  `Error_Handler()` at all. If `EH_isInitialized()` is false (CAN1 itself never came up), only
  the contactor-open/red-LED fallback runs; otherwise `EH_stop()` reports code 10 and
  `CAN_App_Task()` keeps running so `BMSMaster_NODE` stays on the bus with `halted = 1`.

**No fault emits severity 0.** The CSV grades codes 1 and 2 as `ERROR`, and the old
implementation's use of `ERROR_SEVERITY_SAFE_STATE` for `BMS_TEMP_HIGH` is not followed. That
also means `EH_triggerSafeState()` is never called.

### 10.1 Driver constants

One override, and one constant deliberately left alone.

| Constant | Driver default | Here | Why |
| --- | ---: | ---: | --- |
| `HEARTBEAT_INTERVAL` | 1000 ms | **5000 ms** | The DBC sets `BMSMaster_NODE` to 5000 ms. 1000 ms is a generic driver default matching **no** node in the database: of the 21 `*_NODE` frames, only four set a cycle time at all - `SafeState_NODE` 5000, `BMSMaster_NODE` 5000, `Dashboard_NODE` 5000, `RCD_STATIC_NODE` 2000 - and the rest inherit the database's unset 100 ms default. The old firmware never overrode it, so it transmitted ID 128 five times faster than its own database. |
| `ERROR_INTERVAL` | 300 ms | **300 ms, unchanged** | Left alone deliberately. `GenMsgCycleTime` specifies the frame's *nominal* rate, which is the healthy heartbeat; the database says nothing about how fast the frame may go when faulted, so there is no conflict to resolve. Two separate constants exist precisely so the frame speeds up under fault - that is what an error frame is for - and the multiplexing depends on it. |

The driver **multiplexes** through active errors via `currentTransmitIndex`, which supersedes
the earlier lowest-numbered-active-fault rule: adopting the shared driver means adopting its
behaviour rather than forking a third one. Keeping `ERROR_INTERVAL` at 300 ms is what makes that
workable - three simultaneous faults all reach the bus within about a second, where a 5000 ms
faulted rate would take fifteen, and sixteen active errors would take eighty. Cost is one 8-byte
frame at 300 ms, roughly 0.05 % of a 500 kbit/s bus, and only while a fault is active.

`SAFE_STATE_FRAME_ID (0x000)` in `error_handler.h` is **dead** - referenced nowhere in
`error_handler.c`, and `EH_triggerSafeState()` reports a severity-0 error on the node's own
frame rather than transmitting a dedicated one. Safe state is the *severity field*, not a frame
ID. Delete the constant on import so nobody trusts it: the real safe-state frame is **ID 1**,
per the DBC (`BO_ 1 SafeState_Activ`, `CM_ BO_ 1 "Frame from all PCBs."`, and no frame at ID 0
at all), the old code's `SAFE_STATE frame (StdId = 1)`, and our own `can_id_list.h` where
`SAFE_STATE_ID 0` is commented out and `ERROR_MSG_ID 1` is live.

`Error_Handler()` opens the contactor (duty 0 %), turns `RED_LD` on, then keeps CAN1
transmitting `BMSMaster_NODE` with the fault and `halted = 1`, so the vehicle learns *why* the
board stopped rather than only that it vanished. If CAN1 itself failed to initialise it falls
back to LED-and-trap.

`App_OnFatalError()` must survive being called before anything is initialised: 16 of the 18
`Error_Handler()` call sites are in `SystemClock_Config` and the `MX_*_Init` functions, ahead of
`app_main()`. Every hardware access there is guarded on its handle. The contactor is open at
those sites by construction rather than by luck - `PWM_Out_Init` (inside `CONTACTOR_Init`) is the
only caller of `HAL_TIM_PWM_Start` on TIM3, so until it runs the output is never enabled and
`CCR3` stays 0 - and `CONTACTOR_ForceOpen()` writes nothing while its timer handle is `NULL`.
`RED_LD`'s port and pin are compile-time constants, so `App_OnFatalError` binds them itself if
`initAll` has not. It also opens GPIOB's clock gate and configures the pin itself, because the
three `SystemClock_Config` sites precede `MX_GPIO_Init` and a write to a gated peripheral is
silently discarded on the F1 rather than faulting. RCC is always clocked, so the fault path can
open the gate: the LED lights at **all** 18 sites. There is no watchdog and no software reset,
by decision.

### 10.2 LEDs

`GREEN_LD` blinks at 1 Hz to prove the loop is alive; `RED_LD` is solid whenever
`eh.activeErrorCount > 0`. State is changed only on transition, never every pass.

The canonical driver synchronises blink phase through a shared counter, so
**`LED_IncSyncTick()` must be hooked into `HAL_IncTick()`** or `syncTick` never advances and the
LEDs never blink. `HAL_IncTick` is `__weak`, so `app.c` overrides it:

```c
void HAL_IncTick(void)          /* overrides the __weak HAL implementation */
{
    uwTick += uwTickFreq;
    LED_IncSyncTick();
}
```

`LED_SetSyncTick()` exists so a board can adopt the network-wide tick from
`SafeState_SyncTick` (ID 30) and blink in phase with the rest of the car. That frame is declined
(section 1), so this board's LEDs blink independently - cosmetic, and the only consequence of
that exclusion.

## 11. Stated assumptions and open items

| Item | Status |
| --- | --- |
| `RS_DIR` -> `DE` (active high), `RE_DIR` -> `/RE` (active low) | **Confirmed** by Bartek on 2026-09-11, agreeing with the inference from the SN65HVD72 pinout and a boot state of both LOW = listen. Kept as named constants, and bring-up step 6 still puts a scope on PC4/PC5 - a confirmation from memory is not a traced schematic, and the same class of inference proved wrong for the CAN standby pins. |
| Error codes 3-10 | Allocated here; must be added to the team CSV registry. |
| CAN-DATABASE PR #46 | **Merged.** Submodule pinned to `master` (`60ab52e`); regenerated with no content diff. |
| ADC calibration constants | `28.3626` divider and `2108` offset / `5÷2` current gain ship as named defines marked uncalibrated, and are corrected at bring-up step 3. They live in `App/Inc/bms_calib.h` - see section 5.3. |
| `HVIL`, fan, radio, watchdog, bus-off recovery | Deferred by decision - section 1. |

## 12. Verification

There is no board on hand, so verification is designed to establish as much as possible
off-target. `tests/` is **never compiled into the firmware**: it is absent from `.cproject`'s
`sourceEntries`, so it contributes zero bytes to the image, and there is **no `#ifdef` for tests
anywhere in `App/`**. The host build compiles the same unmodified `App/Src/*.c` against
`tests/fake/` in place of the real HAL, so application code is identical in both builds. This is
permanent test infrastructure, not scaffolding to be removed.

Note what this does *not* require: there are **no test-only symbols in `App/`**. The fake works
by link substitution, and every module that holds logic is reachable through its ordinary
interface. Where that was not true - the 60 degC policy sitting inside a superloop that never
returns - the logic was moved into a module rather than the interface being widened to reach it.

### 12.1 Target build, no board required

`stm32_build` drives the CubeIDE headless builder with nothing attached, and gives:

- `-Wall -Wextra -Werror` with no warnings, and a `cppcheck` pass. `.cproject` records only
  `-Wextra` and `-Werror` as explicit options; `-Wall` is CubeIDE's plugin **default** and so is
  never written to `.cproject`, which stores non-default settings only. This is not a missing
  flag: the generated `Debug/App/Src/subdir.mk` (and its `Release/` counterpart) show the actual
  `arm-none-eabi-gcc` invocation with `-Wall -Wextra -Werror` all present. Recorded here so
  nobody "rediscovers" the same false alarm from reading `.cproject` alone.
- `arm-none-eabi-size` against the 64 KB flash / 64 KB RAM budget of section 3.6.
- The `_Static_assert`s on the thermistor ID map (section 6.1) fire **at compile time**, so a
  database renumber breaks the build here rather than on a bench.

### 12.2 Host unit tests

For the logic where a bug yields plausible-looking output rather than an obvious failure. A
plain Makefile and an assert-based runner, no framework dependency:

Tests exercise the modules **through their real interfaces**, not through internals exposed for
testing. Nothing is added to a public interface for a test's benefit.

| Seam | Covered by driving it |
| --- | --- |
| `app_adc`, via the injected DMA buffer | NTC LUT and interpolation, trimmed mean, all three conversions, DBC clamping, open/short fault bands |
| `app_therm`, via `THERM_OnFrame()` + `THERM_Task()` | the non-monotonic ID map including an **even** pack, trimmed mean, warm-up below `fill = 3`, silent-pack detection at three misses |
| `jk_protocol`, directly - it is already pure | accumulated checksum, `LENGTH` semantics, TLV walk, length-prefixed `0x79`, both `0x84` encodings, sign negation, flag curation |
| `app_thermal`, directly - it takes values, not getters | both 60 degC limits, hysteresis in each direction, and that an invalid board reading is not judged |

### 12.3 Fake HAL: running the application off-target

Because modules accept their DMA buffer and their tick rather than creating them (section 3.1),
`app_main()`'s loop runs natively once ~15 HAL functions are faked. The fakes **record** rather
than simulate:

| Faked | Test sees |
| --- | --- |
| `HAL_GetTick` | a variable the test advances, so time is driven not waited on |
| `HAL_CAN_AddTxMessage` | every transmitted frame appended to a list, with its tick |
| `HAL_CAN_GetRxMessage` | frames the test queues, per bus |
| `HAL_GPIO_WritePin` / `ReadPin` | recorded pin state; readable inputs the test sets |
| `__HAL_TIM_SET_COMPARE` / `__HAL_TIM_GET_AUTORELOAD` | contactor duty as a number |
| `HAL_UARTEx_ReceiveToIdle_DMA`, `HAL_UART_Transmit_DMA` | a canned JK response, and the request that was sent |
| `HAL_ADCEx_Calibration_Start`, `HAL_ADC_Start_DMA`, `HAL_CAN_ConfigFilter`, `HAL_CAN_Start`, `HAL_CAN_ActivateNotification`, `HAL_TIM_PWM_Start` | success, recorded for ordering assertions |

The fake set only needs to supply `stm32f1xx_hal.h`. With `tests/fake/` first on the host
include path, the **real** `Core/Inc/main.h` is used unchanged - it picks up the fake header and
then defines the real pin macros, so pin definitions are never duplicated and cannot drift.

Behaviour this makes assertable with no hardware:

- Inject `SafeState_Activ`, assert duty drops to 0 %; advance 300 ms, assert 100 % for 2 s then
  50 % - the whole state machine of section 8, including the pull-in kick on every re-close.
- Feed CAN2 frames over 10 s of driven time and assert all nine output frames carry correctly
  transposed, trimmed values, **including an even-numbered pack**.
- Feed a canned JK response and assert the contents of frames 140-148, including the negated
  current sign of section 7.4.
- Advance several minutes and assert the transmit cadence really is 500 / 1000 / 5000 ms.
- Assert init ordering: both standby pins driven LOW *before* `HAL_CAN_Start`.

### 12.4 Frame packing checked against the database

Encode a frame with the generated C, then decode it in Python with the cantools fork already
present in `.claude/tmp/.venv`, and assert it round-trips. This checks scaling, byte order and
bit positions against **the database itself** rather than against this document's reading of it.

**Not used: QEMU and Renode.** Neither has an STM32F105 machine - QEMU models F100 and F405 -
and the connectivity line's dual bxCAN, PLL2/PLL3 and ADC-plus-DMA interaction are unmodelled.
That would mean simulating precisely the peripherals whose behaviour is in question, against a
model of unknown fidelity. The fake HAL exercises the same application logic with no pretence
about the hardware.

### 12.5 On target

Via `stm32-mcp`: `stm32_build_and_flash`, then `stm32_read_memory` on ELF symbols
(`thermFiltered`, `thermMiss`, `faultMask`, ADC outputs) and `live_memory_start` to watch them
move. There is no debug UART - USART1 is the JK link - so SWD is the only channel.

**Requires the board, and cannot be established off-target:** CAN bit timing on a terminated
bus; RS485 turnaround and the `DE`/`/RE` levels in practice; the ADC calibration constants of
section 5.3; the NTC curve against the real thermistor; whether the contactor holds at 50 %;
and whether HSE actually starts.

**Bring-up order**, sequenced to isolate the unknowns:

1. Green LED blinking - proves the loop runs and HSE started. A dark LED localises a clock
   failure immediately, since a failed HSE traps in `Error_Handler`.
2. CAN1 TX on an analyser at 500 kbit/s - confirms bit timing and the standby pins together.
3. ADC values over SWD against a multimeter on PC0/PC1/PC2 - calibrates section 5.3.
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
| `docs/pcb.md` | Record `PD1-OSC_OUT` as reserved but not wired, HSE as an external oscillator in BYPASS, the ADC sampling time, and the standby pin polarity. |
| `docs/canDatabase.md` | Note that `HEARTBEAT_INTERVAL` in `Error_Corrutines` must be overridden to the database's 5000 ms, and that `ERROR_INTERVAL` is deliberately left at 300 ms because `GenMsgCycleTime` describes the nominal rate only. |
| `EKO_Drivers/CAN/readme.md` | The NART section is correct; our stripped `CAN_Init` was the deviation. |
| `docs/index.md` | Add this specification. |

# BMS-JK Communication

## Status

Implemented. `App/Inc/jk_protocol.h` / `App/Src/jk_protocol.c` hold the pure protocol logic
(no HAL, no state, no time); `App/Inc/app_jk.h` / `App/Src/app_jk.c` hold the RS485 transport
state machine. Together they poll a JK BMS over RS485 and expose decoded values to `app_can.c`.

## Hardware

| Function | Resource | Configuration |
| --- | --- | --- |
| RS485 TX | `PA9` / `USART1_TX` | DMA (`DMA1_Channel4`) |
| RS485 RX | `PA10` / `USART1_RX` | DMA (`DMA1_Channel5`), `ReceiveToIdle` |
| Driver direction | `PC4` / `RS_DIR` | GPIO output, drives transceiver `DE` |
| Receiver enable | `PC5` / `RE_DIR` | GPIO output, drives transceiver `/RE` |
| UART format | USART1 | 115200 baud, 8N1, asynchronous |

The JK link is **USART1 on PA9/PA10**, not USART2 on PA2/PA3: `BMS-Master.ioc` does not assign
`USART2` at all (`PA9.Mode=Asynchronous`, `PA9.Signal=USART1_TX`; `USART1.BaudRate=115200`,
`USART1.VirtualMode=VM_ASYNC`). USART1 is not reserved for logging in this project - it is the
JK link.

## Direction control (confirmed 2026-09-11)

The fitted transceiver is an SN65HVD72. `App/Src/app_jk.c` drives:

- `RS_DIR` -> `DE`, **active high**.
- `RE_DIR` -> `/RE`, **active low**.
- Both LOW (the GPIO reset state from `MX_GPIO_Init`) is the listening/idle state.

```c
#define DE_ACTIVE    GPIO_PIN_SET     /* transmit: DE high        */
#define DE_IDLE      GPIO_PIN_RESET
#define RE_MUTED     GPIO_PIN_SET     /* /RE high while DE is high */
#define RE_LISTENING GPIO_PIN_RESET   /* /RE low: receiver enabled */
```

## Transport state machine (`app_jk.c`)

Non-blocking, so the CAN scheduler keeps exact cadence - no HAL UART call ever blocks the
superloop:

```
JK_IDLE
  -> Timing_Due (1 Hz) fires: build request, DE high / RE muted,
     HAL_UART_Transmit_DMA -> JK_SENDING
JK_SENDING
  -> JK_OnTxComplete (ISR, fires on TC - the final stop bit): DE low / RE listening,
     HAL_UARTEx_ReceiveToIdle_DMA -> JK_RECEIVING
  -> 100 ms elapsed with no TC: JK_COMMS_TIMEOUT, -> JK_IDLE
JK_RECEIVING
  -> JK_OnRxEvent (ISR) gives exact length; JK_Task validates and decodes -> JK_IDLE
  -> 100 ms elapsed with no RX event: JK_COMMS_TIMEOUT, -> JK_IDLE
```

Poll rate **1000 ms** (`JK_POLL_MS`), timeout **100 ms** (`JK_TIMEOUT_MS`, against a worst-case
29.4 ms response at 115200 baud). Command `0x01` (activate) is sent once at startup and again
only after the BMS is suspected asleep; otherwise command `0x06` (read all) is sent. Three
consecutive failures (`JK_FAIL_LIMIT`) zero the published payload rather than hold stale cell
voltages, and set `needActivation` so the next poll re-activates the link.

A timeout racing the ISR's `SENDING -> RECEIVING` transition is closed with a compare-and-swap
on `state`: the task's timeout write only commits if `state` is still `JK_SENDING` at that
instant, so a transmission that completed a moment earlier is never clobbered back to `IDLE`
while its RX DMA is live.

## Frame format

| Offset | Field | Notes |
| --- | --- | --- |
| 0 | STX | `0x4E 0x57` |
| 2 | LENGTH | 2 bytes, big-endian, = total length - 2 (includes itself and the checksum) |
| 4 | Terminal ID | 4 bytes, `00 00 00 00` |
| 8 | Command word | `0x01` activate, `0x06` read all |
| 9 | Frame source | `0x03` = PC upper computer |
| 10 | Transmission type | `0x00` request, `0x01` reply, `0x02` **unsolicited** |
| 11 | Payload | TLV stream: identifier byte + data, per a table-driven length map |
| ... | Record number | 4 bytes |
| ... | End flag | `0x68` |
| ... | Checksum | 4 bytes: 2 zero (CRC16 slot, disabled) + 2-byte **accumulated sum**, big-endian, over every byte from offset 0 through the end flag |

The BMS also sends **unsolicited** frames (transmission type `0x02`), not only replies to a
request, so the transport must not assume strict request/response pairing - `JKP_Decode()` does
not check offset 10 at all. The payload is a variable-length TLV walk, not fixed offsets:
identifier `0x79` (cell voltages) is itself length-prefixed, 3 bytes per cell. Worst case is
about 339 bytes, so the RX buffer (`JKP_RX_BUF_LEN`) is 512 B.

The checksum is a plain 16-bit accumulated sum (not a CRC), computed identically for building a
request (`JKP_BuildRequest`) and validating a response (`JKP_Validate`): sum every byte up to and
including the end flag, store the result big-endian in the last two bytes.

## Signal mapping and topology

See [firmwareSpec.md](firmwareSpec.md) sections 7.4-7.6 for the full JK-register-to-CAN-signal
table, the current sign convention, the warning-flag curation, and the 21S / seven-module
topology (seven 3S5P modules, 105 physical cells, 21 series taps measured). That table is the
source of truth; it is not duplicated here to avoid the two drifting apart.

## Link loss

After three failed polls, all nine JK-sourced CAN frames (140 `JK_Pack`, 141-146 the six cell
frames, 147 `JK_Temp`, 148 `JK_CycleStats`) transmit with zeroed payloads, because `app_jk.c`
zeroes its own decoded `JK_Data_t` on the third failure - `app_can.c` has no separate zeroing
logic to keep in step. `BMS_ERR_JK_COMMS_TIMEOUT` is raised and the cycle time is preserved so
any consumer watching cadence is unaffected.

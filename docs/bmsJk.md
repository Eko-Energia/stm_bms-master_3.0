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
29.4 ms response at 115200 baud). **Every poll is `0x06` read-all** and the firmware sends no
other command: none of the five reference implementations activates, and the link runs
indefinitely on the bench without it. Nothing gates the retry, so the same read goes out every
poll and the first good frame restores the link however long it has been down.

Three consecutive failures (`JK_FAIL_LIMIT`) zero the published payload rather than hold stale
cell voltages.

A frame carrying **no data TLVs is rejected by the decoder**: every field is conditional, so an
empty but structurally valid frame would decode as an all-zero `JK_Data_t` and publish 0 % SOC
with 21 cells at 0 mV as healthy.

## RS485 turnaround

`DE` and `/RE` are **one net** on this board (`Rs485_TxRxEN`), so the driver and the receiver
switch together. Two consequences, both handled in `app_jk.c`:

**Before transmitting**, the SN65HVD72 takes up to **9 us** to enable its driver, against an
8.68 us bit time at 115200. Handing the bytes to the DMA immediately puts the start bit on a
driver that is still turning on, and the far end sees a corrupt frame. `sendRequest` waits for
the driver first - in `JK_Task`, never in an ISR.

**After transmitting**, releasing needs no hold: driver disable is 0.4 us max, and the stop bit is
sampled at its midpoint well before `TC` fires. The 3.5 bit time figure quoted for RS485 is the
Modbus RTU inter-frame gap, which delimits frames by silence; this protocol has an explicit `0x68`
end flag and a length field.

Releasing does switch the receiver on at the instant the line settles from driven to biased, and
that edge frames as a spurious byte. A line error aborts the DMA, so the reply still arriving
would be lost. `JK_OnUartError` clears the flag, re-arms and keeps listening, with the response
timeout as the backstop; errors past `JK_LINE_ERR_LIMIT` are reported as code 5 with kind 1.

## Timeout race

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
| 8 | Command word | `0x06` read all - the only command this firmware sends |
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
including the end flag, store the result big-endian in the last two bytes. The two CRC16 bytes
before it must be zero, as the protocol declares - they sit outside the sum, so an unchecked
slot is 16 freely malleable bits.

`JKP_Decode()` also checks **values**, not only structure: an unknown identifier, a TLV that runs
past the payload, or a field outside its protocol/CAN_DB range rejects the whole frame, which the
transport reports as `JK_FRAME_INVALID` and which keeps the previous reading. See
[firmwareSpec.md](firmwareSpec.md) section 7.4 for the per-field bounds.

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

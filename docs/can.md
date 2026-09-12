# CAN Frames and canDriver Integration

## Bus configuration

The project configures both bxCAN peripherals for 500 kbit/s:

- CAN1 uses PA11 RX and PA12 TX.
- CAN2 uses PB12 RX and PB13 TX.

The generic driver is in `EKO_Drivers/CAN/Inc/can_driver.h` and `EKO_Drivers/CAN/Src/can_driver.c`.

## Frame structure

A frame contains a standard or extended identifier, data-frame type, DLC, and up to eight payload bytes. The scheduler supports both `CAN_ID_STD` with `StdId` and `CAN_ID_EXT` with `ExtId`.

```c
struct CAN_scheduledMsg message = {
    .header = {
        .StdId = 0x082,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 6
    },
    .periodMs = 500,
    .getData = FillMeasurements,
    .context = &bms
};
```

`DLC` must be at most 8. The callback fills the payload immediately before transmission, so values are not stale in the scheduler.

## Transmit schedule (this project, CAN1, 21 frames)

`CAN_MAX_MSG` in `EKO_Drivers/CAN/Inc/can_driver.h` is **28** (21 CAN1 TX frames plus headroom),
not 32. `app_can.c` registers 20 of them; `EH_init()` registers the 21st, `BMSMaster_NODE`
(ID 128), itself, which is why `CAN_App_Init()` runs before `EH_init()`.

| ID | Frame | DLC | Period |
| ---: | --- | ---: | ---: |
| 128 | `BMSMaster_NODE` | 8 | 5000 ms |
| 130 | `BMSMaster_MasterVoltCurrTemp` | 6 | 500 ms |
| 131-139 | `BMSMaster_PCBsTherm1..9Temp` | 7 | 1000 ms |
| 140 | `BMSMaster_JK_Pack` | 8 | 1000 ms |
| 141-143 | `BMSMaster_JK_Cells_1_4` / `_5_8` / `_9_12` | 8 | 1000 ms |
| 144-145 | `BMSMaster_JK_Cells_13_16` / `_17_20` | 8 | 1000 ms |
| 146 | `BMSMaster_JK_Cells_21` | 2 | 1000 ms |
| 147 | `BMSMaster_JK_Temp` | 8 | 1000 ms |
| 148 | `BMSMaster_JK_CycleStats` | 8 | 1000 ms |
| 159 | `BMSMaster_END` | 8 | 1000 ms |

The JK cell frames now span **IDs 141-146** (21S: four 4-cell frames plus one 1-cell frame),
with `JK_Temp` and `JK_CycleStats` moved down to **147** and **148** respectively
(`Eko-Energia/CAN-DATABASE` PR #46, merged). Total offered load is about 21 frames/s, roughly
0.3 % of a 500 kbit/s bus.

**Safe state arrives on CAN1, not CAN2.** `SafeState_Activ` (ID 1) and `SafeState_NODE` (ID 3)
are both defined in `CAN_DB.dbc` (the CAN1 database), not `CAN2_DB.dbc`. An earlier version of
this document, describing the superseded reference implementation, placed safe state on CAN2;
that was wrong for this project. `SafeState_Activ` has no signals - its presence on the bus is
the entire message - and the state machine that consumes it is documented in
[firmwareSpec.md](firmwareSpec.md) section 8.

## Filter layout

`SlaveStartFilterBank = 14` (STM32F105 has 28 banks shared between CAN1 and CAN2):

- **CAN1**: one bank (bank 0), 16-bit identifier-list mode, accepting exactly IDs **1**
  (`SafeState_Activ`) and **3** (`SafeState_NODE`).
- **CAN2**: five banks starting at bank **14**, 32-bit mask mode, mask `0x7F0`, covering
  `0x0D0`-`0x11F` (208-287 decimal) for the seven `PCBCells<x>_Therm<y>` thermistor frames, plus
  a software bounds check rejecting IDs 208, 209, 280-287, and the `NODE` offsets that the mask
  alone cannot exclude.

## Payload packing

Use one scaling convention and one byte-order convention for every callback. The driver provides `GET_BYTE(value, index)` for extracting little-endian bytes:

```c
uint16_t encodedVoltage = 0;
data[0] = GET_BYTE(encodedVoltage, 0);
data[1] = GET_BYTE(encodedVoltage, 1);
```

The reference BMS layer uses callbacks such as `BMS_CAN_Get_ADC_Data()` and thermistor callbacks to pack data. Keep scaling separate from byte packing.

## Scheduled transmission

The scheduler stores up to `CAN_MAX_MSG` messages - **28** in this project, not 32. Initialize CAN after the Cube-generated peripheral setup, register each message once, and service the scheduler in the main loop:

```c
struct CAN_scheduledMsgList canTx = {0};

CAN_Init(&hcan1);
CAN_AddScheduledMsg(&message, &canTx);

for (;;) {
    CAN_HandleScheduled(&hcan1, &canTx);
}
```

`CAN_AddScheduledMsg()` rejects zero periods and duplicate identifiers. `CAN_HandleScheduled()` clears an eight-byte local buffer, invokes `getData`, calls `HAL_CAN_AddTxMessage()`, and updates the timestamp only after successful enqueue.

## Receiving frames

The driver provides `CAN_IncomingMsg` and `CAN_IncomingMsgList`. A CAN FIFO callback should copy the header and payload into the incoming buffer using `CAN_AddIncomingMsg()`. Application code can then call `CAN_GetLatestMessage()` outside interrupt context and validate identifier, DLC, and payload before decoding.

`CAN_GetLatestMessage()` is **FIFO by `tail`**, not "lowest CAN ID" as an earlier version of this
document claimed: it returns `list[buffer->tail]`, advances `tail`, and decrements `count` -
first message queued, first message returned, regardless of identifier value.

Keep interrupt work short and perform frame decoding in the main loop.

## Integration checklist

- Call `CAN_Init()` after Cube-generated CAN initialization.
- Configure transceiver standby pins before starting the peripheral.
- Use one scheduler per CAN peripheral when ownership differs.
- Keep payload callbacks short and deterministic.
- Validate identifier type, DLC, byte order, and scaling.
- Handle transmit mailbox failures and CAN error callbacks.
- Do not perform lengthy decoding or logging inside the CAN IRQ callback.

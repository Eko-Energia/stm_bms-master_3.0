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

## Reference BMS frames

| Frame | Standard ID | DLC | Period |
| --- | ---: | ---: | ---: |
| Node identification | 128 (`0x080`) | 8 | 5000 ms |
| Voltage/current/temperature | 130 (`0x082`) | 6 | 500 ms |
| Thermistor groups 1 to 9 | 131 to 139 (`0x083` to `0x08B`) | 7 | 1000 ms |
| Safe state | 1 (`0x001`) | application-defined | status/event |

The reference application receives thermistor traffic on CAN2 using decimal IDs 211 through 279. Keep this decimal ID scheme distinct from hexadecimal filter notation.

## Payload packing

Use one scaling convention and one byte-order convention for every callback. The driver provides `GET_BYTE(value, index)` for extracting little-endian bytes:

```c
uint16_t encodedVoltage = 0;
data[0] = GET_BYTE(encodedVoltage, 0);
data[1] = GET_BYTE(encodedVoltage, 1);
```

The reference BMS layer uses callbacks such as `BMS_CAN_Get_ADC_Data()` and thermistor callbacks to pack data. Keep scaling separate from byte packing.

## Scheduled transmission

The scheduler stores up to 32 messages. Initialize CAN after the Cube-generated peripheral setup, register each message once, and service the scheduler in the main loop:

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

The reference CAN2 filter setup includes an exact filter for safe-state ID 1 and a broad filter for thermistor traffic, followed by software bounds checking. Keep interrupt work short and perform frame decoding in the main loop.

## Integration checklist

- Call `CAN_Init()` after Cube-generated CAN initialization.
- Configure transceiver standby pins before starting the peripheral.
- Use one scheduler per CAN peripheral when ownership differs.
- Keep payload callbacks short and deterministic.
- Validate identifier type, DLC, byte order, and scaling.
- Handle transmit mailbox failures and CAN error callbacks.
- Do not perform lengthy decoding or logging inside the CAN IRQ callback.

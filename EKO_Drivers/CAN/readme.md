# CAN_DRIVER
The driver is designed to simplify the usage of CAN bus communication on STM32F303 (might work on other MCUs). Features  of the module are designed specificaly to be used in PERLA - a solar car built by AGH Eko-Energia

## Limitations
None currently known. Filter configuration is the caller's responsibility - see "Configuring filters" below.

## Configuration

User settable options live in the defines section of `Inc/can_driver.h`, next to `CAN_MAX_DLC` and `CAN_MAX_MSG`.

| Macro | Default | Meaning |
| --- | --- | --- |
| `CAN_AUTO_RETRANSMISSION` | `0U` (disabled) | Transmission policy applied by `CAN_Init()`. `1` clears the bxCAN `NART` bit, so the peripheral retries a frame that lost arbitration or hit a bus error until it is acknowledged. `0` sets `NART`, so a frame is sent once and its mailbox is released even if the attempt failed. |
| `CAN_TX_FAIL_LIMIT` | `3U` | Consecutive failed enqueue attempts of one scheduled message before `CAN_HandleScheduled()` aborts all pending TX requests. Counts periods, not loop iterations. `0` disables the recovery. |
| `CAN_MAX_MSG` | `28` | Capacity of one `CAN_scheduledMsgList` and one `CAN_IncomingMsgList`. |
| `CAN_FILTER_BANK_COUNT` | `28` | Total bxCAN filter banks on the STM32F105, shared between CAN1 and CAN2. |
| `CAN_SLAVE_START_FILTER_BANK` | `14` | First bank owned by CAN2. A CAN2 filter must use a bank `>= CAN_SLAVE_START_FILTER_BANK`; a bank below it belongs to CAN1 and silently filters nothing for CAN2. |

Filters are configured by the caller first (see "Configuring filters" below), then
`CAN_Init()` writes `NART` and starts the peripheral. NART is only writable while the
peripheral is still in initialisation mode - after `HAL_CAN_Init()` and before
`HAL_CAN_Start()` - which is why `CAN_Init()` must do it in that order. The value is
mirrored into `hcan.Init.AutoRetransmission` so a later `HAL_CAN_Init()` does not revert
it; the value configured in CubeMX is therefore overridden by this driver.

### Choosing a transmission policy

Automatic retransmission is disabled by default: every frame this board sends is
periodic, so a failed frame is superseded by the next period rather than needing a
retry.

It has one failure mode worth knowing about if enabled. bxCAN has **three** TX mailboxes. A frame that is never acknowledged - single node on the bus, missing 120 Ω termination, bit timing mismatch - is retried by hardware forever and never releases its mailbox. After three such frames every `HAL_CAN_AddTxMessage()` fails. `CAN_HandleScheduled()` handles this in two ways: a failed enqueue leaves the frame due so it retries on the next pass rather than losing its slot, and after `CAN_TX_FAIL_LIMIT` missed **periods** the pending requests are aborted so the queue can drain.

For a pure periodic status frame, where the next period supersedes stale data anyway, `CAN_AUTO_RETRANSMISSION = 0` is the safer choice - a failed frame is dropped immediately and can never block a mailbox. This is why it is the default here.

## Usage

### Configuring filters
Filters must be configured **before** `CAN_Init()`, because the bxCAN `NART` bit is only
writable while the peripheral is still in initialisation mode, and `CAN_Init()` leaves that
mode as part of starting the peripheral.

```C
uint16_t ids[4] = { 130u, 131u, 132u, 132u };   // repeat an id to fill unused slots
CAN_ConfigFilterList16(&hcan1, 0u, ids);        // bank 0: CAN1, list mode, up to 4 std IDs

CAN_ConfigFilterMask32(&hcan2, CAN_SLAVE_START_FILTER_BANK, 0x0D0u, 0x7F0u); // bank 14: CAN2

CAN_Init(&hcan1);
CAN_Init(&hcan2);
```

On the STM32F105 the 28 filter banks (`CAN_FILTER_BANK_COUNT`) are shared between CAN1 and
CAN2: banks below `CAN_SLAVE_START_FILTER_BANK` belong to CAN1, banks at or above it belong to
CAN2. Programming a CAN2 filter into a CAN1 bank silently accepts nothing on CAN2.

### Initialization
Requires HAL CAN instance to be imported from main.c as well as a buffer `CAN_scheduledMsgList`
```C
// app.c

extern CAN_HandleTypeDef;

struct CAN_scheduledMsgList buffer;

void app_main(void)
{
    CAN_ConfigFilterMask32(&hcan, 0u, 0u, 0u); // accept all, adjust as needed
    CAN_Init(&hcan);
    ...
}

```
Unlimited number of buffers can be created, to change maximum number of messages for one buffer edit `CAN_MAX_MSG`

### GetData
Creating a frame requires a function that will fill the outgoing buffer with data on every call. 

```C
static void chargerGetData(uint8_t *data, void *context)
{
	// preserve one decimal place
	float maxCurrent = maxChargerCurrent * 10;
	float maxVoltage = MAX_CHARGER_VOLTAGE * 10;

	uint16_t maxChargerCurrentInt = (uint16_t) maxCurrent;
	uint16_t maxVoltageInt = (uint16_t) maxVoltage;

	data[0] = GET_BYTE(maxVoltageInt, 0);
	data[1] = GET_BYTE(maxVoltageInt, 1);
	data[2] = GET_BYTE(maxChargerCurrentInt, 0);
	data[3] = GET_BYTE(maxChargerCurrentInt, 1);
}
```


### Adding a frame to the buffer
```C
struct CAN_scheduledMsg chargerComms;

chargerComms.header.extId = CANID_RCD_STATIC_CHARGER1COMMS;
chargerComms.header.DLC = 3;
chargerComms.header.IDE = CAN_ID_EXT;
chargerComms.header.RTR = CAN_RTR_DATA;
chargerComms.lastTick = 0;
chargerComms.periodMs = 1000;
chargerComms.getData = chargerGetData;

CAN_AddScheduledMsg(chargerComms, &CAN_buffer);
```

### Handling messages added to the buffer

The function should propably be on the end of the main loop
```C
CAN_HandleScheduled(&hcan, &buffer);
```

Timing notes:
- The due check is `(now - lastTick) >= periodMs`, which stays correct across the 32 bit `HAL_GetTick()` wrap (~49.7 days).
- On success `lastTick` advances by whole periods, so the cadence does not drift with the execution time of the send. If a message falls more than one period behind it resynchronises to the current tick instead of emitting a catch-up burst.
- On a failed enqueue `lastTick` is left alone, so the frame stays due and retries on the next pass; the remaining messages in the list are still processed. Re-arming it instead would silence the frame for a whole period - with three mailboxes and a burst of frames sharing one period, only the first three would ever transmit. `txFailCount` in `CAN_scheduledMsg` is managed by the driver, do not write it.

### Removing a frame
```C
CAN_RemoveScheduledMsg(CANID_RCD_STATIC_CHARGER1COMMS);
```

### Reading incoming messages
`CAN_AddIncomingMsg()` is called from the RX FIFO0 ISR, `CAN_GetLatestMessage()` from the main
loop. Despite its name, `CAN_GetLatestMessage()` returns messages **in FIFO order by arrival**
(oldest first, via `tail`) - not the lowest CAN ID and not the newest message.

```C
struct CAN_IncomingMsg msg;
while (CAN_GetLatestMessage(&rxBuffer, &msg) == HAL_OK)
{
    // handle msg.header / msg.data
}
```



# TO-DO
- [ ] Add non-periodic frame handling (period = 0), send once on handle and delete
- [ ] Handling on bus errors and generic error messages
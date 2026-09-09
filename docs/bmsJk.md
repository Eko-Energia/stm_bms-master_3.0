# BMS-JK Communication

## Status

The project reserves USART2 and RS485 direction GPIOs for a JK BMS link. A completed JK protocol source module is not present in the copied clean project, so the transport and integration behavior are documented here without inventing a JK frame format.

## Hardware and UART

| Function | Resource | Configuration |
| --- | --- | --- |
| RS485 TX | PA2 / USART2_TX | UART output |
| RS485 RX | PA3 / USART2_RX | UART input |
| Driver direction | PC4 / `RS_DIR` | GPIO output |
| Receiver enable | PC5 / `RE_DIR` | GPIO output |
| UART format | USART2 | 115200 baud, 8 data bits, no parity, 1 stop bit |

USART1 is reserved for logging/debug output. USART2 is reserved for the JK link.

## RS485 transmit and receive

A half-duplex transaction must switch the transceiver direction around the UART transfer:

1. Disable or inhibit the receiver and select transmit mode.
2. Assert the RS485 driver enable.
3. Send the complete request with blocking, interrupt, or DMA HAL UART APIs.
4. Wait for UART transmission complete, including the final stop bit.
5. Deassert driver enable and enable the receiver.
6. Collect and validate the response.

The active GPIO levels depend on the fitted transceiver and must be confirmed from the schematic.

```c
HAL_GPIO_WritePin(RS_DIR_GPIO_Port, RS_DIR_Pin, TX_LEVEL);
HAL_GPIO_WritePin(RE_DIR_GPIO_Port, RE_DIR_Pin, RX_DISABLED_LEVEL);
HAL_UART_Transmit(&huart2, request, requestLength, timeoutMs);
/* Wait for transmission-complete before changing direction. */
HAL_GPIO_WritePin(RS_DIR_GPIO_Port, RS_DIR_Pin, RX_LEVEL);
HAL_GPIO_WritePin(RE_DIR_GPIO_Port, RE_DIR_Pin, RX_ENABLED_LEVEL);
HAL_UART_Receive(&huart2, response, responseLength, timeoutMs);
```

Use `HAL_UART_Receive_IT()` or `HAL_UART_Receive_DMA()` for periodic polling so the main loop is not blocked.

## Receiving JK data

The receive state machine should collect bytes into a bounded buffer, identify the frame start/address, determine the expected length, verify the command and checksum, then decode values. On timeout, bad length, bad command, or checksum failure, discard the frame and return to idle.

```text
IDLE -> BUILD_REQUEST -> TX_ENABLE -> SEND
     -> WAIT_TX_COMPLETE -> RX_ENABLE -> RECEIVE
     -> CHECK_FRAME -> DECODE -> STORE -> IDLE
```

Keep JK protocol decoding separate from BMS ADC calculations. JK values are already measured by the external BMS and have their own byte order, scale, sign, and units.

## Integration into the copied project

The copied project already initializes `huart2` and the RS485 GPIOs in generated code. Add a separate JK application module containing the exact JK commands, checksum, receive buffer, timeout policy, decoded-value storage, and CAN publication. Do not place the protocol state machine in generated `usart.c`, because CubeMX regeneration can overwrite it.

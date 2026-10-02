# P9 motor + UART/Qt demo

This is the latest integrated demo in the repository. It is experimental
firmware for STM32F103CBT6, MT6816 and TB67H450 hardware.

## Safety behavior

- The driver starts in `Sleep`; calibration no longer starts automatically.
- Start calibration explicitly with CAN command `0x02` and keep the motor clear
  of people and mechanisms while it moves in both directions.
- An SPI timeout, parity failure or MT6816 no-magnet report immediately stops
  the driver and records an encoder fault state.
- The motor will not produce closed-loop output without a valid calibration
  table and a valid current encoder sample.

## CAN payload rules

CAN uses an 11-bit standard identifier: upper four bits are the node ID and the
lower seven bits are the command. Node `0` is broadcast; the default node is
`1`. Multi-byte payloads use the STM32 little-endian representation for
compatibility with the existing Qt tool.

- `0x03`, `0x04`, `0x05`: one 32-bit float (4 bytes).
- `0x06`: position float followed by time float (8 bytes). There is no ACK flag
  because all eight CAN payload bytes belong to those two values.
- `0x07`: position float followed by velocity-limit float (8 bytes).
- Configuration commands reject truncated payloads. Node IDs must be `0..15`.

Malformed or truncated frames are ignored.

## UART framing

### Raw byte echo diagnosis (currently disabled)

`UART_ECHO_TEST_MODE=1` receives and transmits one byte at a time using polling
HAL calls in the main loop. It adds no text, newline or protocol framing and
does not start encoder sampling, PWM, control timers, DMA RX or CAN commands.
Use a serial terminal at 115200/8N1, disable local echo and automatic sending,
and send a short message or a small hex packet. Adapter TX connects to PB7,
adapter RX to PB6, with shared GND. This is a wiring diagnostic, not a bulk
throughput test. Set the flag to `0` before returning to protocol V1.

### Protocol-only diagnosis (currently disabled)

`UART_COMM_ONLY_TEST=1` runs the real UART DMA/parser/mailbox/response protocol
from the main loop without starting encoder sampling, PWM or control timers.
Sensors remain invalid, so ENABLE is rejected; CAN commands are ignored.
An encoder fault in this mode is expected. Position values do not track the shaft.
Use this mode to isolate handshake failure from control-interrupt starvation.
Set this flag to `0` only after the serial diagnostic has been completed.

Keil Watch counters: `uartDiagPolls` and `uartDiagControlTicks` show loop progress;
`uartDiagRxBytes` shows RX callback delivery, `uartDiagValidFrames` shows parsed
frames, and `uartDiagTxStarts` shows successful TX DMA submissions.

### Temporary TX wiring test (currently disabled)

Set `UART_TX_TEST_MODE` in `UserApp/configurations.h` to `1` to enable this mode.
The normal default is `0`. In test mode,
USART1 transmits `UART1 TX TEST\r\n` repeatedly with a 500 ms delay. Use a serial
terminal at 115200/8N1 with text reception, not the protocol-V1 host application.
Connect adapter RX to PB6 and share GND. The driver is put to sleep; encoder
initialization, PWM and control timer startup, protocol RX and CAN commands are
bypassed. This verifies board TX only, not board RX.

Set `UART_TX_TEST_MODE` to `0`, rebuild and flash again before protocol testing.
The transmit test uses blocking HAL calls in the diagnostic main loop only.

UART now uses protocol revision 1 at 115200/8N1:

`AA55 | version | type | length | session | sequence | command | payload | CRC16`

All multi-byte values are little-endian. The maximum payload is 128 bytes and
CRC is CRC-16/CCITT-FALSE over bytes from `version` through the payload. RX DMA
chunks are copied into a bounded ring and parsed in the main loop; CRC and
command handling never run in the UART interrupt. Responses have priority over
telemetry and TX buffers are not reused until the DMA completion callback.

The first integrated command set is `HELLO`, `GET_INFO`, `GET_STATUS`,
`HEARTBEAT`, `ENABLE`, `DISABLE`, `STOP`, `CLEAR_FAULT` and
`TELEMETRY_CONFIG`. Other revision-1 commands return `UNSUPPORTED` until their
motor, parameter, storage or calibration backend is implemented.

Control commands enter a three-lane mailbox and are acknowledged only after the
20 kHz control boundary applies them. A valid heartbeat is required every
500 ms; timeout disables the driver, clears the active session and latches the
communication fault. A new session or fault clear never re-enables output.

While a UART session owns control, CAN motion and calibration commands are
ignored. A new UART session is rejected while an existing CAN motion or
calibration operation is active.

## Remaining hardening work

The calibration table currently has no header, schema version, CRC or atomic
commit marker. Until that storage format is upgraded, recalibrate after any
unexpected power loss during calibration and validate behavior at low current
before applying a mechanical load.

The current integration has passed host-side protocol/control tests and a full
Keil build, but has not yet been flashed or verified with a physical serial
link. Motion targets, parameter persistence and UART-driven calibration remain
intentionally disabled.

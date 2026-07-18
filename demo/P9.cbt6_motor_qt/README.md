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

The current Qt protocol uses five-byte, idle-delimited frames beginning with
`AA 55`. Short frames are ignored. Receive-to-idle DMA is re-armed after every
frame and telemetry is skipped while the previous DMA transmission is busy.

## Remaining hardening work

The calibration table currently has no header, schema version, CRC or atomic
commit marker. Until that storage format is upgraded, recalibrate after any
unexpected power loss during calibration and validate behavior at low current
before applying a mechanical load.

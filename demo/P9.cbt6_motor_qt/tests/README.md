# P9 serial regression tests

Run from the P9 project root using the installed MinGW compiler. Add
`E:\qt\Tools\mingw730_64\bin` to the current process PATH before execution.

```
g++ -std=c++11 -Wall -Wextra -Werror -I tests/protocol_mocks -I Uart -I Motor tests/uart_handshake_test.cpp Uart/protocol_v1.cpp Motor/control_gate.cpp Motor/control_service.cpp -o tests/uart_handshake_test.exe
tests/uart_handshake_test.exe
g++ -std=c++11 -Wall -Wextra -Wno-missing-field-initializers -Wno-sign-compare -I tests/encoder_mocks -I Encoder tests/encoder_spi_test.cpp Encoder/mt6816_base.cpp -o tests/encoder_spi_test.exe
tests/encoder_spi_test.exe
g++ -std=c++11 -Wall -Wextra -Werror -I tests/mocks -I UserApp tests/uart_tx_test.cpp -o tests/uart_tx_test.exe
tests/uart_tx_test.exe
```

The handshake test compiles the actual P9 UART integration with fake hardware
boundaries, not the portable command dispatcher. It reproduces a HELLO retry
after the control ISR has changed the session but before the main loop has
sent its completion. It also checks same-session HELLO and out-of-order replies.

The encoder test compiles the real encoder implementation with register doubles.
It checks that initialization sets SPE and that stalled RX exits with CS high.
Two existing warning classes in the encoder headers are suppressed in that test.
It does not simulate SPI electrical behavior or verify interrupt execution time.

2026-10-02: HELLO retry and SPI-enable tests both failed against the old code and
passed after fixes. All three executables then passed. Changed firmware units
compiled and linked with the existing Keil ARMClang toolchain, generating HEX.
Physical protocol V1 handshake remains to be verified after flashing.

2026-10-03 update: the reduced-stack UART diagnostic firmware was flashed by
the user. Actual Qt SerialTransport/SessionController passed two Ready/telemetry/
heartbeat/STOP/reconnect rounds on COM8, with 256 and 255 snapshots respectively.
This does not validate normal control interrupts or mechanical motion.

Run `./tests/check-stack.ps1 -Mode normal` after linking normal firmware. The gate
adds the parser's unresolved callback chain, active equal-priority IRQ maximum,
SysTick and 128 bytes of exception/unknown-call allowance. Missing depths or an
exceeded 1 KiB budget are errors. Runtime high-water measurement is still needed.
The normal candidate currently estimates exactly 1024 bytes including allowance;
it has been compiled but not flashed or hardware-validated.

2026-10-03 protocol hardening: after rebuilding uart_protocol.cpp with ARMClang
6.7 Cortex-M3/Thumb -Oz and linking against the existing normal objects, the
new estimate is 840/1024 bytes including allowance (main=280, receiver=88,
IRQ=344). This supersedes the old candidate estimate, not the requirement for
on-target high-water testing. No new firmware was flashed in this round.

After compiling uart_handshake_test.exe with the command above, also run:

```
tests/uart_handshake_test.exe cached-takeover
tests/uart_handshake_test.exe pending-takeover
tests/uart_handshake_test.exe running-takeover
tests/uart_handshake_test.exe info-capabilities
```

The takeover cases failed against the old sequence-only cache/pending lookup.
They now verify an independent HELLO token scope, old-session rejection,
same-session content conflicts, and preserved BUSY/WRONG_STATE safety gates.
The info test exercises the actual GET_INFO frame with HAL UID doubles and
verifies that unimplemented target commands are not advertised as capabilities.
The original suite and all four added cases pass. Encoder, TX and echo
regressions also pass; these are hardware-boundary tests, not electrical tests.

Command-contract alignment (2026-10-03): additionally run
`tests/uart_handshake_test.exe request-contract` after compiling the executable.
It covers all 11 unimplemented commands, validates payload before UNSUPPORTED,
checks session/replay priority, and verifies no mailbox/target/heartbeat effects.
The old implementation returned UNSUPPORTED for a five-byte TASK_QUERY and
failed the BAD_PAYLOAD assertion before this integration.

Uart/command_contract.h is a byte-identical distribution copy of the Qt
workspace firmware/core/command_contract.h. Do not edit it independently.
From the Qt workspace run `./scripts/check-firmware-core.ps1 -P9Project <P9-root>`
before cross-repository delivery. command_support.h declares only real wired
backends; adding a declaration alone does not implement a command or justify
capability bits. The current support remains 9 commands and capabilities 0x180.

Final stack-gate correction: run `./tests/test-stack-gate.ps1`, then
`./tests/check-stack.ps1 -Mode normal` against the actual linked report.
Compile the UART unit with `-g` to emit local stack-frame metadata; unknown
UART/inline-contract frames are now rejected. The earlier 840B estimate is
invalid. Metadata first revealed 1032B; isolating the snapshot return temporary
in a non-inlined main-loop helper reduced the estimate to 1008/1024B
(main312 + receiver200 + IRQ368 + allowance128). Only 16B remains. Existing
library exception/pointer tails still rely on the allowance assumptions; this
is not an absolute bound or hardware acceptance. Full rebuild, on-target stack
high-water and ISR timing verification remain required. No firmware was flashed.

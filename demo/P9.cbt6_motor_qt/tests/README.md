# P9 serial regression tests

Committed calibration format:

```
g++ -std=c++11 -Wall -Wextra -Werror -I Encoder tests/calibration_record_test.cpp -o tests/calibration_record_test.exe
tests/calibration_record_test.exe
```

encoder_spi_test initializes test metadata explicitly: uncommitted, torn,
CRC/schema/length/range-invalid records and non-128KB capacity are rejected.
The storage test checks metadata invalidation before table erase, header then
commit ordering, metadata write failure/corruption/torn commit, and reset-time
acceptance only after a complete valid commit. A cancellation after the durable
commit is not a rollback: RAM remains stopped but the verified stored table may
be accepted on the next boot. Terminal reporting must preserve this distinction.

2026-11-08 cooperative storage update: calibration_storage_test now calls the
real main-loop writer repeatedly and enforces at most one page erase or 32
halfword writes per call. Verify scans at most 128 halfwords per call in source.
Every write chunk relocks before returning; no nested UART polling is used.
Both rotation directions match pre-refactor full-table FNV vectors
479c6945/b04ba6d5. Nine injected faults/cancellations include between-block and
pre-verify cancellation. HAL page operations are still synchronous and their
real latency is not established by these tests.

Flash driver test compiles the actual C translation unit, not a C++ rewrite:

```
gcc -std=c99 -Wall -Wextra -Werror -I tests/flash_mocks -I Memory -c Memory/stockpile_f103cb.c -o tests/flash_driver.o
g++ -std=c++11 -Wall -Wextra -Werror -I tests/flash_mocks -I Memory tests/flash_driver_test.cpp tests/flash_driver.o -o tests/flash_driver_test.exe
tests/flash_driver_test.exe
```

HAL doubles verify error exits, removed redundant wait, 16/32/64-bit program
failure behavior, cursor bounds, null input and count overflow. The installed
HAL still has a 50-second tick-based timeout; these tests do not prove real
worst-case latency or communication service during Flash operations.

Real calibration main-loop storage test (Flash memory is a RAM-backed double):

```
g++ -std=c++11 -Wall -Wextra -Wno-sign-compare -Wno-missing-field-initializers -I tests/calibration_mocks -I Encoder -I Driver tests/calibration_storage_test.cpp Encoder/encoder_calibrator_base.cpp -o tests/calibration_storage_test.exe
tests/calibration_storage_test.exe
```

Covers 16384 generated halfwords, cursor/write failure, immediate and late
corruption, erase failure, mid-write/final/erase cancellation, output sleep and
Flash relock. This does not validate the real HAL erase latency, reset/power-loss
commit integrity or asynchronous IRQ timing; public startup remains disabled.

Internal startup request tests (not a public UART command yet):
`calibration-start-ok`, `calibration-start-stop`, `calibration-start-timeout`,
`calibration-start-unverified`. These cover deferred activation, repeated/current/
session/sensor validation, STOP-before-start, timeout-before-start, and rejection
of success when the old calibrator exits without a verified storage result.
They deliberately use an uncalibrated sensor with valid live samples.

Task-record core (not yet connected to production CALIBRATE_START):

```
g++ -std=c++11 -Wall -Wextra -Werror -I Uart tests/calibration_tasks_test.cpp -o tests/calibration_tasks_test.exe
tests/calibration_tasks_test.exe
```

Tests session isolation, one active task, phase/progress monotonicity, explicit
verification before success, four retained terminal records, cancel-versus-late
completion and independent revision-1 TASK_QUERY/CALIBRATE_QUERY body vectors.
This is a sequential state-machine test, not Flash concurrency or motor validation.

Calibration-current test (real EncoderCalibratorBase implementation, hardware
and Flash replaced; does not rotate a motor):

```
g++ -std=c++11 -Wall -Wextra -Wno-sign-compare -Wno-missing-field-initializers -I tests/calibration_mocks -I Encoder -I Driver tests/calibration_current_test.cpp Encoder/encoder_calibrator_base.cpp -o tests/calibration_current_test.exe
tests/calibration_current_test.exe
```

Checks all six output phases at 250/500/1000mA commands, rejects nonpositive
or >1000mA configuration, canceled/invalid-sensor ticks produce no output,
and no Flash primitive is called by the tested IRQ paths. This does not test
main-loop persistence or establish actual measured coil current.

Calibration output cancellation regression scenarios (offline, fake calibrator
and motor boundary): `calibration-stop`, `calibration-disable`,
`calibration-timeout`, `calibration-encoder`, `calibration-takeover`.
Recompile uart_handshake_test.exe before running them. They assert cancellation
of the trigger/state, StopAndReset invocation, and no automatic re-enable.
Takeover remains rejected with WRONG_STATE(5), and the original session can STOP.
They do not test Flash-write cancellation, task terminal events, or physical output.

2026-11-08 encoder increment: `encoder_spi_test` now checks calibration bounds,
TXE/RXNE/BSY timeout and CS release, error-count saturation, scripted parity and
no-magnet sequences, final-attempt recovery and transport recovery. Scripted
register behavior exists only in `tests/encoder_mocks/spi.h`, not production HAL.
The mock supplies RX data at RXNE checks; it is not a SPI electrical simulator.

After recompiling `uart_handshake_test.exe` with the command below, run
`tests/uart_handshake_test.exe encoder-recovery` and `request-contract` as well
as the existing scenarios. Encoder recovery verifies the real ControlService
with a counted fake motor driver: invalid sampling stops, restored samples keep
the fault latched, CLEAR_FAULT is rejected while invalid, clearing a valid fault
does not enable, and a separate ENABLE is required. This does not execute the
real Motor::Tick20kHz or prove physical output shutdown timing.

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

Subsequent main-loop snapshot/dispatch separation: full normal rebuild yields
992/1024B estimate (312 main +184 receiver +368 IRQ +128 allowance), 32B
headroom; ZI increases16B. Run check-stack.ps1 with `-MinimumHeadroom 32`
to catch regressions to the old16B reserve; this is not a hardware safety limit.
test-stack-gate.ps1 also checks exact reserve acceptance/one-byte deficit rejection.
Only the non-reentrant main-loop parser callback owns requestState/requestTimeMs.

2026-11-09: run `tests/uart_handshake_test.exe calibration-lifecycle` after
rebuilding the UART test with the existing protocol mocks. Covers deferred
storage cancellation, durable commit winning over late STOP, generation
isolation and no automatic enable. Together with the previous 16 scenarios,
17 scenarios pass. This does not expose calibration commands on the wire.

Query increment: `tests/uart_handshake_test.exe task-query-wire` checks both
query response bodies, cached retry versus a fresh terminal query, unknown ID
and session rejection. Rebuild and run all previous scenarios too (18 total).
11 requests are supported; capabilities stay 0x180 until START/CANCEL/events
are complete. No external task creation or hardware acceptance yet.

2026-11-10 supersedes the preceding query-only state: normal profile supports
13 requests / 0x1A0 with START/CANCEL and main-loop 0501 task events. Diagnostic
profiles omit START/CANCEL (11 / 0x180). Run `calibration-wire` plus the previous
18 scenarios. Covers deferred activation, replay task ID, control-boundary
cancel ACK, terminal cancel without stopping later work, successful commit
event, takeover rejection, CALIBRATING and expired-session event suppression.
All are offline tests; no board calibration or motion acceptance.

Fault increment: `calibration-faults` plus previous 19 scenarios (20 total)
checks storage/calibration fault latches, explicit clear completion, no auto
enable, calibration command-current feedback, and a pending cancel blocking
the next task. STOP has one mailbox lane; the second request is BUSY, not
another queued cancel. No FAULT_EVENT or hardware safety acceptance yet.

Run `calibration-interleaving` plus the previous 20 scenarios (21 total).
Injects a newer internal task failure after CLEAR_FAULT IRQ completion but
before main-loop drain; checks old clear rejection and a fresh clear success.
Fills all four non-telemetry TX slots and checks terminal-event retention,
retry after drain and one successful enqueue. This is not exactly-once serial
delivery or exhaustive interrupt scheduling coverage. No product change.

## Real calibration writer / UART integration (2026-11-10)

From the P9 project directory, use the existing MinGW compiler:

```powershell
$env:Path='E:\qt\Tools\mingw730_64\bin;'+$env:Path
g++ -std=c++11 -Wall -Wextra -Wno-sign-compare -Wno-missing-field-initializers -I tests/integration_mocks -I tests/calibration_mocks -I tests/protocol_mocks -I Encoder -I Driver -I Uart -I Motor tests/calibration_uart_integration_test.cpp Encoder/encoder_calibrator_base.cpp Uart/protocol_v1.cpp Motor/control_gate.cpp Motor/control_service.cpp -o tests/calibration_uart_integration_test.exe
0..3 | ForEach-Object { & tests/calibration_uart_integration_test.exe $_; if($LASTEXITCODE){throw "scenario $_ failed"} }
```

Scenarios: success, readback corruption, cancel after 32 table writes, cancel
after complete durable commit. Links real analysis/writer/lifecycle and UART
backend; sends encoded requests through RX/parser and checks decoded query
and terminal event against real record CRC validation. No fake Outcome.
Collected samples are synthetic; Flash, motor and sensors are hardware mocks.
No physical sampling, HAL Flash timing, reset, motion or measured current is
validated. SetFocCurrentVector must not run, and normal enable count stays zero.

## Bounded position-task policy (not connected to hardware)

```powershell
g++ -std=c++11 -Wall -Wextra -Werror -I Motor tests/position_move_test.cpp -o tests/position_move_test.exe
& tests/position_move_test.exe
```

Caller-supplied distance/speed/acceleration and settling limits, explicit
planner-complete before actual arrival, timeout and cancellation. Fixture
numbers are not physical limits; sample example is once per ms. This is not
a trajectory generator, UART MOVE_ABS handler or motor-controller integration.
No production-path/firmware capability change or hardware acceptance.

Real planner / Gate integration (offline only):

```powershell
g++ -std=c++11 -Wall -Wextra -Werror -I Motor tests/position_planner_integration_test.cpp Motor/motion_planner.cpp Motor/control_gate.cpp -o tests/position_planner_integration_test.exe
& tests/position_planner_integration_test.exe
```

Calls the real planner at 20kHz, observes task state at 1ms, heartbeats at
100ms; tests +/-1000 fixture targets, delayed actual arrival, STOP at 200ms,
and loss at 500ms. Existing tracker can reverse to correct overshoot; tests
check speed magnitude and final convergence, not monotonicity or acceleration.
This is not full Motor::Controller / UART / IRQ integration or output testing.

Actual controller adapter (hardware outputs mocked, no UART):

```powershell
g++ -std=c++11 -Wall -Wextra -Wno-missing-field-initializers -Wno-sign-compare -I Motor -I Encoder -I Driver -I UserApp tests/motor_position_target_test.cpp Motor/motor.cpp Motor/motion_planner.cpp -o tests/motor_position_target_test.exe
& tests/motor_position_target_test.exe
```

Checks real target setter, limits/offset overflow, planner completion and real
StopAndReset. KNOWN LIMIT: -100 target at v500/a1000 has not settled after
100000 calls (5s), observed -101/40. Test requires no false completion;
-1000 does converge. This is an unresolved limitation, not a short-move pass.
No new API is yet called by UART; firmware command support does not change.

Short-target fix supersedes the preceding KNOWN LIMIT: the Motor regression
now requires -100 to complete. Matrix test:

```powershell
g++ -std=c++11 -Wall -Wextra -Werror -I Motor tests/position_short_move_test.cpp Motor/motion_planner.cpp -o tests/position_short_move_test.exe
& tests/position_short_move_test.exe
```

30 combinations of +/-1/2/10/100/1000 steps and acceleration 1000/10000/100000,
speed cap 500; 20kHz planner, 10s simulated bound, 1000 stable stopped calls.
Checks velocity delta <= ceil(a/20000) with integer quantization. Also rebuild
Motor adapter and planner/Gate integration tests. No physical acceptance.

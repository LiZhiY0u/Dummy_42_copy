#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../Uart/uart_protocol.cpp"

// Fail to stderr rather than opening the Windows CRT assertion dialog.
#undef assert
#define assert(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
    std::exit(1); } } while (0)

UART_HandleTypeDef huart1 = {};
FakeController fakeController;
Motor motor = { &fakeController };
FakeEncoder mt6816_base;
FakeCalibrator encoder_calibrator_base;
FakeConfig boardConfig = {1000, 2000, 1536000, 1000000};
static uint32_t tick;
static std::vector<stepper::Frame> replies;
uint32_t HAL_GetTick() { return tick; }
static void received(void *, const stepper::Frame &f) { replies.push_back(f); }
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *bytes, uint16_t size)
{
    stepper::Parser decoder;
    decoder.feed(bytes, size, tick, received, nullptr);
    return HAL_OK;
}
static void drain()
{
    for (unsigned i = 0; i < 4; ++i) {
        transmitNext();
        UartProtocolOnTxComplete();
    }
}
static stepper::Frame hello(uint16_t seq)
{
    stepper::Frame f = {};
    f.type = 1; f.sequence = seq; f.command = 1; f.length = 4;
    write32(f.payload, 0x12345678);
    return f;
}
static void completeAndDrain()
{
    UartProtocolControlTick(tick);
    finishControls(tick);
    drain();
}
static void testTakeover(const char *scenario)
{
    UartProtocolInit();
    handleFrame(nullptr, hello(1));
    if (strcmp(scenario, "pending-takeover") != 0)
        completeAndDrain();
    replies.clear();
    auto next = hello(1); // A different session has an independent sequence space.
    write32(next.payload, 0x87654321);
    if (strcmp(scenario, "running-takeover") == 0)
        fakeController.modeRunning = 1;
    handleFrame(nullptr, next);
    completeAndDrain();
    if (strcmp(scenario, "pending-takeover") == 0) {
        bool rejectedBusy = false, firstCompleted = false;
        for (const auto &r : replies) {
            if (read16(r.payload) == 8) rejectedBusy = true;
            if (read16(r.payload) == 0 && r.session == 0x12345678) firstCompleted = true;
        }
        assert(rejectedBusy && firstCompleted);
        assert(controlService.snapshot().session == 0x12345678);
    } else if (strcmp(scenario, "running-takeover") == 0) {
        assert(replies.size() == 1 && read16(replies[0].payload) == 5);
        assert(controlService.snapshot().session == 0x12345678);
    } else {
        assert(replies.size() == 1 && read16(replies[0].payload) == 0);
        assert(replies[0].session == 0x87654321 && highWater == 1);
        replies.clear();
        stepper::Frame hb = {};
        hb.type = 1; hb.session = 0x12345678; hb.sequence = 2; hb.command = 4;
        handleFrame(nullptr, hb); // The old session cannot renew the new session.
        drain();
        assert(replies.size() == 1 && read16(replies[0].payload) == 11);
        replies.clear();
        hb.session = 0x87654321;
        handleFrame(nullptr, hb);
        completeAndDrain();
        assert(replies.size() == 1 && read16(replies[0].payload) == 0);
        replies.clear();
        hb.length = 1; hb.payload[0] = 1; // Same-session content conflict remains rejected.
        handleFrame(nullptr, hb);
        drain();
        assert(replies.size() == 1 && read16(replies[0].payload) == 12);
    }
    std::puts("PASS: session-scoped takeover, old-session rejection and safety gates");
}
static void testInfoCapabilities()
{
    UartProtocolInit();
    handleFrame(nullptr, hello(1));
    completeAndDrain();
    replies.clear();
    stepper::Frame request = {};
    request.type = 1; request.session = 0x12345678; request.sequence = 2; request.command = 2;
    handleFrame(nullptr, request);
    drain();
    assert(replies.size() == 1 && replies[0].length == 52);
    const uint8_t *data = replies[0].payload + 2;
    assert(read16(replies[0].payload) == 0);
    assert(read32(data + 10) == 0x11223344 && read32(data + 14) == 0x55667788 &&
           read32(data + 18) == 0x99aabbcc);
    // Only fault clearing and telemetry are usable advertised feature groups.
    // Modes cannot be advertised until their target commands are implemented.
    assert(read32(data + 22) == 0x00000180);
    replies.clear();
    for (uint16_t command = 0x0103; command <= 0x0105; ++command) {
        request.sequence++; request.command = command;
        handleFrame(nullptr, request);
        drain();
        assert(replies.size() == 1 && read16(replies[0].payload) == 10);
        replies.clear();
    }
    std::puts("PASS: GET_INFO wire layout, UID serialization and truthful capabilities");
}
int main(int argc, char **argv)
{
    if (argc > 1) {
        if (strcmp(argv[1], "info-capabilities") == 0)
            testInfoCapabilities();
        else
            testTakeover(argv[1]);
        return 0;
    }
    UartProtocolInit();
    auto first = hello(1);
    uint8_t wire[144];
    const size_t length = stepper::encode(first, wire, sizeof(wire));
    // Exercise real RX ring/parser and command mailbox.
    UartProtocolOnRx(wire, uint16_t(length));
    UartProtocolPoll(tick);
    UartProtocolControlTick(tick);
    // Reproduce an interrupt applying HELLO before a retry is parsed.
    handleFrame(nullptr, first);
    finishControls(tick);
    drain();
    assert(!replies.empty());
    for (const auto &r : replies)
        assert(r.command == 1 && read16(r.payload) == 0 && r.session == 0x12345678);
    replies.clear();
    // Same-token HELLO must not reset sequence history or telemetry settings.
    telemetryPeriodMs = 20;
    handleFrame(nullptr, hello(2));
    UartProtocolControlTick(tick);
    finishControls(tick);
    drain();
    assert(telemetryPeriodMs == 20);
    assert(highWater == 2);
    // A later heartbeat may complete before an earlier state read.
    stepper::Frame state = {};
    state.type = 1; state.session = 0x12345678; state.sequence = 3; state.command = 3;
    auto heartbeat = state;
    heartbeat.sequence = 4; heartbeat.command = 4;
    handleFrame(nullptr, state);
    handleFrame(nullptr, heartbeat);
    UartProtocolControlTick(tick);
    finishControls(tick);
    assert(highWater == 4);
    drain();
    replies.clear();
    mt6816_base.angleData.sampleValid = false;
    UartProtocolControlTick(tick);
    auto enable = state;
    enable.sequence = 5; enable.command = 0x0101;
    enable.length = 1; enable.payload[0] = 0;
    handleFrame(nullptr, enable);
    UartProtocolControlTick(tick);
    finishControls(tick);
    drain();
    assert(replies.size() == 1 && read16(replies[0].payload) != 0);
    assert(controlService.snapshot().state == 4);
    std::puts("PASS: real RX/parser HELLO retry race, same-session state, out-of-order watermark");
}

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
        request.length = command==0x0103 ? 12 : (command==0x0104 ? 8 : 4);
        memset(request.payload,0,request.length);
        if(command!=0x0105)write32(request.payload+4,1);
        if(command==0x0103)write32(request.payload+8,1);
        handleFrame(nullptr, request);
        drain();
        assert(replies.size() == 1 && read16(replies[0].payload) == 10);
        replies.clear();
    }
    std::puts("PASS: GET_INFO wire layout, UID serialization and truthful capabilities");
}
static void testRequestContract()
{
    const auto support=stepper::p9::supportedCommands();
    unsigned supportCount=0;
    for(unsigned i=0;i<20;++i)if(support.commandBits&(uint32_t(1)<<i))++supportCount;
    assert(supportCount==9 && stepper::contract::capabilities(support)==0x180);
    UartProtocolInit();
    handleFrame(nullptr, hello(1));
    completeAndDrain();
    tick = 100;
    uint16_t sequence = 2;
    auto reject = [&](uint16_t command, const std::vector<uint8_t> &bytes, uint16_t expected) {
        stepper::Frame r = {};
        r.type = 1; r.session = 0x12345678; r.sequence = sequence++; r.command = command;
        r.length = uint16_t(bytes.size());
        for (size_t i=0;i<bytes.size();++i) r.payload[i]=bytes[i];
        const auto before = controlService.snapshot();
        replies.clear(); handleFrame(nullptr,r); drain();
        if (replies.size()!=1 || read16(replies[0].payload)!=expected)
            std::fprintf(stderr,"contract cmd=%04x length=%u expected=%u actual=%u\n",command,r.length,expected,replies.empty()?99:read16(replies[0].payload));
        assert(replies.size()==1 && read16(replies[0].payload)==expected && replies[0].length==2);
        const auto after = controlService.snapshot();
        assert(before.session==after.session && before.sampleCounter==after.sampleCounter && before.timestampMs==after.timestampMs);
        assert(before.mode==after.mode && before.state==after.state && before.faultBits==after.faultBits);
        assert(before.targetPosition==after.targetPosition && before.targetVelocity==after.targetVelocity && before.targetCurrentMa==after.targetCurrentMa);
        for (unsigned i=0;i<3;++i) assert(!pending[i].valid);
        stepper::ControlCommand queued = {};
        assert(!mailbox.takeCommand(queued));
        // Cached rejection, then conflicting content with identical sequence.
        replies.clear(); handleFrame(nullptr,r); drain();
        assert(replies.size()==1 && read16(replies[0].payload)==expected);
        r.length++; r.payload[r.length-1]=0;
        replies.clear(); handleFrame(nullptr,r); drain();
        assert(replies.size()==1 && read16(replies[0].payload)==12);
        // Invalid session wins over malformed payload and sequence conflict.
        r.session=0x87654321;
        replies.clear(); handleFrame(nullptr,r); drain();
        assert(replies.size()==1 && read16(replies[0].payload)==11);
    };
    struct Fixture {uint16_t command;std::vector<uint8_t> bytes;};
    const Fixture fixtures[]={
        {5,{1,0,0,0}}, {0x0103,{0,0,0,0,1,0,0,0,1,0,0,0}},
        {0x0104,{0,0,0,0,1,0,0,0}}, {0x0105,{0,0,0,0}}, {0x0107,{}},
        {0x0201,{}}, {0x0202,{1,0,1,0,1,1,1,4,32,0,0,0}}, {0x0203,{1,0,0,0}},
        {0x0301,{}}, {0x0302,{1,0,0,0}}, {0x0303,{1,0,0,0}}
    };
    for(const auto &fixture:fixtures) {
        reject(fixture.command,fixture.bytes,10);
        auto extra=fixture.bytes; extra.push_back(0);
        reject(fixture.command,extra,3);
    }
    for(uint16_t command:{uint16_t(5),uint16_t(0x0302),uint16_t(0x0303)})reject(command,{0,0,0,0},4);
    reject(0x0103,{0,0,0,0,0,0,0,0,1,0,0,0},4);
    reject(0x0104,{0,0,0,128,1,0,0,0},4);
    reject(0x0105,{0,0,0,128},4);
    reject(0x0202,{1,0,1,0,1,1,1,4,0,1,0,0},4);
    for(uint16_t command:{uint16_t(0x0402),uint16_t(0x0501),uint16_t(0x0502),uint16_t(0xaaaa)})reject(command,{},2);
    // Reject traffic at 100ms must not renew the original 0ms heartbeat.
    tick=500; UartProtocolControlTick(tick);
    assert(controlService.snapshot().session==0 && (controlService.snapshot().faultBits&4)!=0);
    std::puts("PASS: 11 unsupported commands validate first; replay/session priority and no control side effects");
}
int main(int argc, char **argv)
{
    if (argc > 1) {
        if (strcmp(argv[1], "request-contract") == 0)
            testRequestContract();
        else if (strcmp(argv[1], "info-capabilities") == 0)
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

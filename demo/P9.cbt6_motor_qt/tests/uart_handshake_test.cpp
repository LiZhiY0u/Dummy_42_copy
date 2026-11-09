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
uint16_t protocolFlashKib=128;
FakeController fakeController;
Motor motor = { &fakeController };
FakeEncoder mt6816_base;
FakeCalibrator encoder_calibrator_base;
FakeConfig boardConfig = {1000, 2000, 1536000, 1000000};
namespace stepper { namespace calibration {
static uint32_t fakeTask=0;
static Outcome fakeOutcome=Outcome::Unknown;
static bool fakeBusy=false;
bool beginTask(uint32_t id){if(fakeBusy || fakeOutcome==Outcome::Pending)return false;fakeTask=id;fakeOutcome=Outcome::Pending;return true;}
void requestCancel(uint32_t id){if(id==fakeTask && fakeOutcome==Outcome::Pending && !fakeBusy)fakeOutcome=Outcome::Cancelled;}
Outcome outcome(uint32_t id){return id && id==fakeTask?fakeOutcome:Outcome::Unknown;}
uint16_t phasePercent(uint32_t id){return id==fakeTask?(1U<<8):0;}
bool storageBusy(){return fakeBusy;}
} }
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
    assert(read32(data + 22) == 0x000001A0);
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
    assert(supportCount==13 && stepper::contract::capabilities(support)==0x1a0);
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
        {0x0103,{0,0,0,0,1,0,0,0,1,0,0,0}},
        {0x0104,{0,0,0,0,1,0,0,0}}, {0x0105,{0,0,0,0}}, {0x0107,{}},
        {0x0201,{}}, {0x0202,{1,0,1,0,1,1,1,4,32,0,0,0}}, {0x0203,{1,0,0,0}}
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
    std::puts("PASS: 7 unsupported commands validate first; replay/session priority and no control side effects");
}
static void testCalibrationInterleaving()
{
    UartProtocolInit();handleFrame(nullptr,hello(1));completeAndDrain();
    const uint32_t session=controlService.snapshot().session;
    calibrationTasks.session(session);
    uint32_t id=calibrationTasks.accept(session,tick);
    failCalibrationTask(id,9);
    stepper::Frame clear={};clear.type=1;clear.session=session;clear.sequence=2;clear.command=0x0108;
    handleFrame(nullptr,clear);UartProtocolControlTick(++tick);
    // Inject a genuine newer task failure between IRQ completion and main-loop drain.
    id=calibrationTasks.accept(session,tick);assert(id);
    failCalibrationTask(id,13);
    replies.clear();finishControls(tick);drain();
    assert(replies.size()==1 && read16(replies[0].payload)==5 && calibrationFaults==40);
    assert(!driver.enable(0,0));
    clear.sequence=3;replies.clear();handleFrame(nullptr,clear);completeAndDrain();
    assert(replies.size()==1 && read16(replies[0].payload)==0 && calibrationFaults==0);
    // Terminal event waits for queue capacity, then is enqueued exactly once.
    eventTask=id;eventSession=session;eventStarted=true;
    stepper::Frame filler={};filler.type=2;filler.command=2;filler.session=session;
    for(unsigned i=0;i<4;++i)assert(enqueueFrame(filler,false));
    const uint16_t sequence=eventSequence;
    emitCalibrationEvent();emitCalibrationEvent();
    assert(eventTask==id && eventSequence==sequence);
    replies.clear();drain();assert(replies.size()==4);
    replies.clear();emitCalibrationEvent();drain();
    assert(replies.size()==1 && replies[0].type==3 && replies[0].payload[4]==5);
    assert(read16(replies[0].payload+6)==13 && read32(replies[0].payload)==id);
    assert(eventTask==0 && eventSequence==uint16_t(sequence+1));
    replies.clear();emitCalibrationEvent();drain();assert(replies.empty());
    assert(fakeController.enables==0);
    std::puts("PASS: delayed clear preserves newer fault; congested terminal event retries exactly once");
}
static void testCalibrationFaults()
{
    UartProtocolInit();handleFrame(nullptr,hello(1));completeAndDrain();
    uint32_t id=0;auto state=controlService.snapshot();
    assert(queueCalibrationStart(state.session,state,id)==1);UartProtocolControlTick(++tick);
    encoder_calibrator_base.state=FakeCalibrator::CALI_FORWARD_PREPARE;
    motor.config.motionParams.caliCurrent=250;
    uint8_t body[48]={};encodeSnapshot(controlService.snapshot(),body);
    assert(body[40]==3 && read32(body+24)==250 && read32(body+28)==250);
    stepper::calibration::fakeOutcome=stepper::calibration::Outcome::Failed;
    UartProtocolControlTick(++tick);
    encodeSnapshot(controlService.snapshot(),body);
    assert(body[40]==4 && (read32(body+36)&40)==40 && read32(body+24)==0);
    assert(!driver.enable(0,0) && fakeController.enables==0);
    state=controlService.snapshot();uint32_t rejected=0;
    assert(queueCalibrationStart(state.session,state,rejected)==5);
    stepper::Frame clear={};clear.type=1;clear.session=state.session;clear.sequence=2;clear.command=0x0108;
    handleFrame(nullptr,clear);UartProtocolControlTick(++tick);
    assert(calibrationFaults==40); // no clearing before completion consumption
    finishControls(tick);drain();assert(calibrationFaults==0 && fakeController.enables==0);
    assert(queueCalibrationStart(state.session,state,id)==1);UartProtocolControlTick(++tick);
    stepper::Frame cancel=clear;cancel.sequence=3;cancel.command=0x0303;cancel.length=4;write32(cancel.payload,id);
    handleFrame(nullptr,cancel);cancel.sequence=4;handleFrame(nullptr,cancel);
    assert(calibrationCancelQueued==1); // STOP lane rejects the second request as BUSY.
    UartProtocolControlTick(++tick);
    assert(currentCalibrationTask==0 && calibrationCancelQueued==1 && calibrationFaults==0);
    state=controlService.snapshot();assert(queueCalibrationStart(state.session,state,rejected)==8);
    finishControls(tick);drain();assert(calibrationCancelQueued==0);
    assert(queueCalibrationStart(state.session,state,id)==1);
    assert(fakeController.enables==0);
    std::puts("PASS: calibration fault latch, explicit clear, current feedback and queued cancellation isolation");
}
static void testCalibrationWire()
{
    mt6816_base.calibrated=false;
    UartProtocolInit();handleFrame(nullptr,hello(1));completeAndDrain();
    stepper::Frame start={};start.type=1;start.session=0x12345678;start.sequence=2;start.command=0x0301;
    replies.clear();handleFrame(nullptr,start);drain();
    assert(replies.size()==1 && replies[0].length==6 && read16(replies[0].payload)==1);
    const uint32_t id=read32(replies[0].payload+2);
    assert(id && !encoder_calibrator_base.isTriggered);
    auto takeover=hello(1);write32(takeover.payload,0x87654321);
    replies.clear();handleFrame(nullptr,takeover);drain();
    assert(replies.size()==1 && read16(replies[0].payload)==5);
    replies.clear();handleFrame(nullptr,start);drain();
    assert(replies.size()==1 && read32(replies[0].payload+2)==id && currentCalibrationTask==id);
    UartProtocolControlTick(++tick);
    assert(encoder_calibrator_base.isTriggered);
    replies.clear();emitCalibrationEvent();drain();
    assert(replies.size()==1 && replies[0].type==3 && replies[0].payload[4]==1);
    stepper::Frame cancel=start;cancel.sequence=3;cancel.command=0x0303;cancel.length=4;write32(cancel.payload,id);
    replies.clear();handleFrame(nullptr,cancel);drain();
    assert(replies.empty() && encoder_calibrator_base.isTriggered);
    completeAndDrain();
    assert(!encoder_calibrator_base.isTriggered && replies.size()==1 && replies[0].length==6);
    assert(read16(replies[0].payload)==0 && read32(replies[0].payload+2)==id);
    replies.clear();emitCalibrationEvent();drain();
    assert(replies.size()==1 && replies[0].type==3 && replies[0].payload[4]==4 && eventTask==0);
    const unsigned stops=fakeController.stops;
    cancel.sequence=4;replies.clear();handleFrame(nullptr,cancel);drain();
    assert(replies.size()==1 && read16(replies[0].payload)==0 && fakeController.stops==stops);
    start.sequence=5;replies.clear();handleFrame(nullptr,start);drain();
    assert(replies.size()==1 && read16(replies[0].payload)==1);
    const uint32_t second=read32(replies[0].payload+2);assert(second!=id);
    UartProtocolControlTick(++tick);
    uint8_t encodedState[48]={};encodeSnapshot(controlService.snapshot(),encodedState);
    assert(encodedState[40]==3);
    stepper::calibration::fakeOutcome=stepper::calibration::Outcome::Verified;
    UartProtocolControlTick(++tick);
    replies.clear();emitCalibrationEvent();drain();
    assert(replies.size()==1 && replies[0].type==3 && replies[0].payload[4]==6);
    assert(read32(replies[0].payload)==second && read16(replies[0].payload+6)==0);
    start.sequence=6;replies.clear();handleFrame(nullptr,start);drain();
    assert(replies.size()==1 && read16(replies[0].payload)==1);
    UartProtocolControlTick(++tick);
    tick=600;UartProtocolControlTick(tick);
    replies.clear();emitCalibrationEvent();drain();
    // Session expiry suppresses stale-session events; historical result remains queryable internally.
    assert(replies.empty() && !encoder_calibrator_base.isTriggered);
    assert(fakeController.enables==0);
    std::puts("PASS: calibration wire deferred start, idempotent acceptance, control-boundary cancel and events");
}
static void testTaskQueryWire()
{
    UartProtocolInit();handleFrame(nullptr,hello(1));completeAndDrain();
    calibrationTasks.session(0x12345678);
    const uint32_t id=calibrationTasks.accept(0x12345678,12);
    assert(id && calibrationTasks.progress(id,1,25,20));
    stepper::Frame req={};req.type=1;req.session=0x12345678;req.sequence=2;
    req.command=5;req.length=4;write32(req.payload,id);
    replies.clear();handleFrame(nullptr,req);drain();
    assert(replies.size()==1 && replies[0].length==14 && read16(replies[0].payload)==0);
    assert(read32(replies[0].payload+2)==id && replies[0].payload[6]==3 && replies[0].payload[7]==1);
    assert(read32(replies[0].payload+10)==20);
    assert(calibrationTasks.cancel(id,30));
    // A retry returns its cached original answer, a fresh query returns terminal state.
    replies.clear();handleFrame(nullptr,req);drain();
    assert(replies.size()==1 && replies[0].payload[7]==1);
    req.sequence++;req.command=0x0302;
    replies.clear();handleFrame(nullptr,req);drain();
    assert(replies.size()==1 && replies[0].length==15 && read16(replies[0].payload)==0);
    assert(replies[0].payload[6]==6 && replies[0].payload[7]==25 && replies[0].payload[10]==0);
    assert(read32(replies[0].payload+11)==30);
    req.sequence++;write32(req.payload,id+1);
    replies.clear();handleFrame(nullptr,req);drain();
    assert(replies.size()==1 && replies[0].length==2 && read16(replies[0].payload)==4);
    req.sequence++;req.session=0x87654321;
    replies.clear();handleFrame(nullptr,req);drain();
    assert(replies.size()==1 && read16(replies[0].payload)==11);
    assert(fakeController.enables==0 && !encoder_calibrator_base.isTriggered);
    std::puts("PASS: task query wire layouts, cached retry, terminal history and session isolation");
}
static void testCalibrationLifecycle()
{
    using namespace stepper::calibration;
    mt6816_base.calibrated=false;
    UartProtocolInit();handleFrame(nullptr,hello(1));completeAndDrain();
    uint32_t first=0;auto snapshot=controlService.snapshot();
    assert(queueCalibrationStart(snapshot.session,snapshot,first)==1);
    UartProtocolControlTick(++tick);assert(encoder_calibrator_base.isTriggered);
    fakeBusy=true;
    stepper::Frame stop={};stop.type=1;stop.session=snapshot.session;stop.sequence=2;stop.command=0x0106;
    handleFrame(nullptr,stop);completeAndDrain();
    stepper::p9::CalibrationTask record={};
    assert(!encoder_calibrator_base.isTriggered && currentCalibrationTask==first);
    assert(calibrationTasks.query(snapshot.session,first,record) && record.phase<5);
    // Main-loop storage publishes that the durable commit won before STOP.
    fakeBusy=false;fakeOutcome=Outcome::Verified;
    UartProtocolControlTick(++tick);
    assert(calibrationTasks.query(snapshot.session,first,record));
    assert(record.storageVerified==1 && record.phase==5 && record.taskPhase==6);
    assert(!currentCalibrationTask && fakeController.enables==0);
    uint32_t second=0;snapshot=controlService.snapshot();
    assert(queueCalibrationStart(snapshot.session,snapshot,second)==1 && second!=first);
    stop.sequence=3;handleFrame(nullptr,stop);completeAndDrain();
    assert(calibrationTasks.query(snapshot.session,second,record));
    assert(record.phase==6 && record.taskPhase==4 && !record.storageVerified);
    // The previous generation's Verified must not complete a new pending task.
    uint32_t third=0;snapshot=controlService.snapshot();
    assert(queueCalibrationStart(snapshot.session,snapshot,third)==1);
    UartProtocolControlTick(++tick);
    assert(outcome(third)==Outcome::Pending && outcome(first)==Outcome::Unknown);
    assert(calibrationTasks.query(snapshot.session,third,record) && !record.storageVerified && record.phase==1);
    assert(fakeController.enables==0);
    std::puts("PASS: deferred cancellation, committed completion wins, generation isolation and no auto-enable");
}
static void testCalibrationStart(const char *scenario)
{
    mt6816_base.calibrated=false; // calibration must work before a table exists
    UartProtocolInit();handleFrame(nullptr,hello(1));completeAndDrain();
    auto snapshot=controlService.snapshot();uint32_t id=0;
    assert(queueCalibrationStart(0,snapshot,id)==11);
    assert(queueCalibrationStart(8,snapshot,id)==11);
    motor.config.motionParams.caliCurrent=1001;
    assert(queueCalibrationStart(snapshot.session,snapshot,id)==4);
    motor.config.motionParams.caliCurrent=500;
    protocolFlashKib=64;
    assert(queueCalibrationStart(snapshot.session,snapshot,id)==9);
    protocolFlashKib=128;
    snapshot.encoderValid=false;
    assert(queueCalibrationStart(snapshot.session,snapshot,id)==7);
    snapshot=controlService.snapshot();
    assert(queueCalibrationStart(snapshot.session,snapshot,id)==1 && id);
    assert(!encoder_calibrator_base.isTriggered); // accepting is not motion
    uint32_t duplicate=0;
    assert(queueCalibrationStart(snapshot.session,snapshot,duplicate)==8);
    if(strcmp(scenario,"calibration-start-stop")==0) {
        stepper::Frame stop={};stop.type=1;stop.session=snapshot.session;
        stop.sequence=2;stop.command=0x0106;handleFrame(nullptr,stop);
    } else if(strcmp(scenario,"calibration-start-timeout")==0) tick=500;
    UartProtocolControlTick(++tick);
    if(strcmp(scenario,"calibration-start-unverified")==0) {
        encoder_calibrator_base.isTriggered=false;
        UartProtocolControlTick(++tick);
    }
    stepper::p9::CalibrationTask task={};
    assert(calibrationTasks.query(snapshot.session,id,task));
    if(strcmp(scenario,"calibration-start-ok")==0) {
        assert(encoder_calibrator_base.isTriggered && task.phase==1);
        assert(calibrationSession==snapshot.session);
    } else {
        assert(!encoder_calibrator_base.isTriggered && task.phase==6);
        assert(task.taskPhase==4 || task.taskPhase==5);
        if(strcmp(scenario,"calibration-start-unverified")==0)
            assert(task.taskPhase==5 && task.result==13 && !task.storageVerified);
    }
    assert(fakeController.enables==0);
    std::puts("PASS: internal calibration request validation, deferred activation and STOP/timeout priority");
}
static void testCalibrationStop(const char *scenario)
{
    UartProtocolInit();
    handleFrame(nullptr, hello(1));
    completeAndDrain();
    encoder_calibrator_base.isTriggered=true;
    encoder_calibrator_base.state=FakeCalibrator::CALI_FORWARD_PREPARE;
    UartProtocolControlTick(++tick);
    assert(encoder_calibrator_base.isTriggered); // valid owned session
    const unsigned stops=fakeController.stops;
    if (strcmp(scenario,"calibration-timeout")==0) {
        tick=500;
        UartProtocolControlTick(tick);
    } else if (strcmp(scenario,"calibration-encoder")==0) {
        mt6816_base.angleData.sampleValid=false;
        UartProtocolControlTick(++tick);
    } else if (strcmp(scenario,"calibration-takeover")==0) {
        replies.clear();
        auto next=hello(1);write32(next.payload,0x87654321);
        handleFrame(nullptr,next);
        completeAndDrain();
        bool rejected=false;
        for(const auto &r:replies) rejected|=read16(r.payload)==5;
        assert(rejected && encoder_calibrator_base.isTriggered);
        assert(controlService.snapshot().session==0x12345678);
        stepper::Frame stop={};stop.type=1;stop.session=0x12345678;
        stop.sequence=2;stop.command=0x0106;
        handleFrame(nullptr,stop);completeAndDrain();
    } else {
        stepper::Frame stop={};stop.type=1;stop.session=0x12345678;
        stop.sequence=2;stop.command=strcmp(scenario,"calibration-disable")==0 ? 0x0102 : 0x0106;
        handleFrame(nullptr,stop);
        completeAndDrain();
    }
    assert(!encoder_calibrator_base.isTriggered);
    assert(encoder_calibrator_base.state==FakeCalibrator::CALI_DISABLE);
    assert(fakeController.stops>stops);
    assert(fakeController.enables==0);
    mt6816_base.angleData.sampleValid=true;
    UartProtocolControlTick(++tick);
    assert(!encoder_calibrator_base.isTriggered && fakeController.enables==0);
    std::puts("PASS: calibration cancel clears trigger/state and stops driver without auto-resume");
}
static void testEncoderRecovery()
{
    UartProtocolInit();
    handleFrame(nullptr, hello(1));
    completeAndDrain();
    uint32_t id = 100;
    auto command = [&](stepper::ControlKind kind) {
        stepper::ControlCommand c = {};
        c.kind=kind; c.requestId=++id;
        c.session=controlService.snapshot().session; c.mode=1;
        assert(mailbox.submit(c));
        UartProtocolControlTick(++tick);
        stepper::ControlCompletion r;
        assert(mailbox.takeCompletion(r));
        return r.status;
    };
    assert(command(stepper::ControlKind::Enable)==0);
    assert(fakeController.enables==1);
    const unsigned stops=fakeController.stops;
    mt6816_base.angleData.sampleValid=false;
    UartProtocolControlTick(++tick);
    assert(fakeController.stops>stops);
    assert(controlService.snapshot().mode==3 && (controlService.snapshot().faultBits&1));
    assert(command(stepper::ControlKind::ClearFault)!=0);
    mt6816_base.angleData.sampleValid=true;
    UartProtocolControlTick(++tick);
    assert(fakeController.enables==1 && controlService.snapshot().mode==3);
    assert(controlService.snapshot().faultBits&1);
    assert(command(stepper::ControlKind::Enable)!=0);
    assert(fakeController.enables==1);
    assert(command(stepper::ControlKind::ClearFault)==0);
    assert(controlService.snapshot().faultBits==0 && controlService.snapshot().mode==3);
    assert(fakeController.enables==1);
    assert(command(stepper::ControlKind::Enable)==0);
    assert(fakeController.enables==2);
    std::puts("PASS: encoder loss stops; recovery stays latched; clear does not enable; explicit enable required");
}
int main(int argc, char **argv)
{
    if (argc > 1) {
        if (strcmp(argv[1], "calibration-interleaving")==0)
            testCalibrationInterleaving();
        else if (strcmp(argv[1], "calibration-faults")==0)
            testCalibrationFaults();
        else if (strcmp(argv[1], "calibration-wire")==0)
            testCalibrationWire();
        else if (strcmp(argv[1], "task-query-wire")==0)
            testTaskQueryWire();
        else if (strcmp(argv[1], "calibration-lifecycle")==0)
            testCalibrationLifecycle();
        else if (strncmp(argv[1], "calibration-start-",18)==0)
            testCalibrationStart(argv[1]);
        else if (strncmp(argv[1], "calibration-",12)==0)
            testCalibrationStop(argv[1]);
        else if (strcmp(argv[1], "encoder-recovery") == 0)
            testEncoderRecovery();
        else if (strcmp(argv[1], "request-contract") == 0)
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

#include "uart_protocol.h"

#include "common_inc.h"
#include "configurations.h"
#include "control_service.h"
#include "protocol_v1.h"
#include <string.h>

volatile uint32_t uartDiagRxBytes = 0;
volatile uint32_t uartDiagValidFrames = 0;
volatile uint32_t uartDiagTxStarts = 0;
volatile uint32_t uartDiagPolls = 0;
volatile uint32_t uartDiagControlTicks = 0;

namespace
{
const uint16_t STATUS_OK = 0;
const uint16_t STATUS_UNKNOWN_CMD = 2;
const uint16_t STATUS_BAD_PAYLOAD = 3;
const uint16_t STATUS_OUT_OF_RANGE = 4;
const uint16_t STATUS_WRONG_STATE = 5;
const uint16_t STATUS_BUSY = 8;
const uint16_t STATUS_UNSUPPORTED = 10;
const uint16_t STATUS_BAD_SESSION = 11;
const uint16_t STATUS_SEQ_CONFLICT = 12;

const uint16_t CMD_HELLO = 0x0001;
const uint16_t CMD_GET_INFO = 0x0002;
const uint16_t CMD_GET_STATUS = 0x0003;
const uint16_t CMD_HEARTBEAT = 0x0004;
const uint16_t CMD_ENABLE = 0x0101;
const uint16_t CMD_DISABLE = 0x0102;
const uint16_t CMD_STOP = 0x0106;
const uint16_t CMD_CLEAR_FAULT = 0x0108;
const uint16_t CMD_TELEMETRY_CONFIG = 0x0401;
const uint16_t CMD_TELEMETRY = 0x0402;

uint16_t read16(const uint8_t *p)
{
    return uint16_t(uint16_t(p[0]) | (uint16_t(p[1]) << 8));
}

uint32_t read32(const uint8_t *p)
{
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
           (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

void write16(uint8_t *p, uint16_t value)
{
    p[0] = uint8_t(value);
    p[1] = uint8_t(value >> 8);
}

void write32(uint8_t *p, uint32_t value)
{
    p[0] = uint8_t(value);
    p[1] = uint8_t(value >> 8);
    p[2] = uint8_t(value >> 16);
    p[3] = uint8_t(value >> 24);
}

class ArmCriticalSection : public stepper::CriticalSection
{
public:
    uint32_t enter()
    {
        const uint32_t saved = __get_PRIMASK();
        __disable_irq();
        __DMB();
        return saved;
    }

    void leave(uint32_t savedMask)
    {
        __DMB();
        if (!savedMask)
            __enable_irq();
    }
};

class P9ControlDriver : public stepper::ControlDriver
{
public:
    uint8_t supportedModes() const { return 0x07; }
    void disableAndReset() { motor.controller->StopAndReset(); }
    bool enable(uint8_t mode, int32_t initialPosition)
    {
        return motor.controller->EnableProtocolMode(mode, initialPosition);
    }
};

ArmCriticalSection critical;
P9ControlDriver driver;
stepper::ControlMailbox mailbox(critical);
stepper::ControlService controlService(mailbox, critical, driver);
stepper::Parser parser;

volatile uint16_t rxHead = 0;
volatile uint16_t rxTail = 0;
uint8_t rxRing[512];

struct TxSlot
{
    uint8_t bytes[144];
    uint16_t length;
    uint8_t state;
    bool telemetry;
};

TxSlot txSlots[4] = {};
volatile int8_t activeTx = -1;
volatile uint16_t txDropCount = 0;

struct Pending
{
    bool valid;
    uint32_t requestId;
    stepper::Frame request;
};

struct Cached
{
    bool valid;
    uint32_t timestamp;
    stepper::Frame request;
    stepper::Frame response;
};

Pending pending[3] = {};
Cached cached[4] = {};
uint8_t nextCache = 0;
uint16_t highWater = 0;
uint32_t cachedSession = 0;
uint32_t nextRequestId = 1;
uint16_t telemetryPeriodMs = 0;
uint32_t lastTelemetryMs = 0;
uint16_t telemetrySequence = 0;
volatile bool receiveError = false;

void incrementDrop()
{
    if (txDropCount != 0xFFFF)
        ++txDropCount;
}

bool sameRequest(const stepper::Frame &a, const stepper::Frame &b)
{
    return a.session == b.session && a.sequence == b.sequence &&
           a.command == b.command && a.length == b.length &&
           memcmp(a.payload, b.payload, a.length) == 0;
}

void clearTransactions()
{
    for (unsigned i = 0; i < 4; ++i)
        cached[i].valid = false;
    highWater = 0;
    nextCache = 0;
}

bool forwardSequence(uint16_t sequence)
{
    const uint16_t delta = uint16_t(sequence - highWater);
    return !highWater || (delta >= 1 && delta <= 32767);
}

void setStatus(stepper::Frame &response, uint16_t status)
{
    response.length = 2;
    write16(response.payload, status);
}

stepper::Frame &makeResponse(const stepper::Frame &request, uint16_t status)
{
    // Main-loop only, non-reentrant. Avoid many 144-byte return-value
    // temporaries in handleFrame on the STM32's 1 KiB stack.
    static stepper::Frame response;
    memset(&response, 0, sizeof(response));
    response.type = 2;
    response.session = request.session;
    response.sequence = request.sequence;
    response.command = request.command;
    setStatus(response, status);
    return response;
}

bool enqueueFrame(const stepper::Frame &frame, bool telemetry)
{
    static uint8_t encoded[144]; // Main-loop TX encoding is non-reentrant.
    const size_t length = stepper::encode(frame, encoded, sizeof(encoded));
    if (!length)
        return false;

    uint32_t saved = critical.enter();
    int slot = -1;
    for (unsigned i = 0; i < 4; ++i)
        if (txSlots[i].state == 0)
        {
            slot = int(i);
            break;
        }
    if (slot < 0 && !telemetry)
        for (unsigned i = 0; i < 4; ++i)
            if (txSlots[i].state == 1 && txSlots[i].telemetry)
            {
                slot = int(i);
                break;
            }
    if (slot < 0)
    {
        incrementDrop();
        critical.leave(saved);
        return false;
    }
    txSlots[slot].state = 2;
    critical.leave(saved);

    memcpy(txSlots[slot].bytes, encoded, length);
    txSlots[slot].length = uint16_t(length);
    txSlots[slot].telemetry = telemetry;

    saved = critical.enter();
    txSlots[slot].state = 1;
    critical.leave(saved);
    return true;
}

void remember(const stepper::Frame &request, const stepper::Frame &response, uint32_t nowMs)
{
    cached[nextCache].valid = true;
    cached[nextCache].timestamp = nowMs;
    cached[nextCache].request = request;
    cached[nextCache].response = response;
    nextCache = uint8_t((nextCache + 1) % 4);
    if (forwardSequence(request.sequence))
        highWater = request.sequence;
}

bool sameSequenceScope(const stepper::Frame &a, const stepper::Frame &b)
{
    if (a.session != b.session)
        return false;
    // HELLO uses SESSION=0 on the wire; its proposed token is its scope.
    // A new token must not collide with another session's cached HELLO.
    if (a.command == CMD_HELLO && b.command == CMD_HELLO &&
        a.length == 4 && b.length == 4)
        return read32(a.payload) == read32(b.payload);
    return true;
}

int findCached(const stepper::Frame &request, uint32_t nowMs)
{
    for (unsigned i = 0; i < 4; ++i)
    {
        if (!cached[i].valid || cached[i].request.sequence != request.sequence ||
            !sameSequenceScope(cached[i].request, request) ||
            uint32_t(nowMs - cached[i].timestamp) >= 2000)
            continue;
        return sameRequest(cached[i].request, request) ? int(i) : -2;
    }
    return -1;
}

int findPending(const stepper::Frame &request)
{
    for (unsigned i = 0; i < 3; ++i)
        if (pending[i].valid && pending[i].request.sequence == request.sequence &&
            sameSequenceScope(pending[i].request, request))
            return sameRequest(pending[i].request, request) ? int(i) : -2;
    return -1;
}

void sendAndRemember(const stepper::Frame &request, const stepper::Frame &response, uint32_t nowMs)
{
    enqueueFrame(response, false);
    remember(request, response, nowMs);
}

void encodeSnapshot(const stepper::ControlSnapshot &snapshot, uint8_t *p)
{
    write32(p + 0, snapshot.timestampMs);
    write32(p + 4, snapshot.sampleCounter);
    write32(p + 8, uint32_t(snapshot.position));
    write32(p + 12, uint32_t(snapshot.targetPosition));
    write32(p + 16, uint32_t(snapshot.velocity));
    write32(p + 20, uint32_t(snapshot.targetVelocity));
    write32(p + 24, uint32_t(snapshot.currentCommandMa));
    write32(p + 28, uint32_t(snapshot.targetCurrentMa));
    write32(p + 32, uint32_t(snapshot.positionError));
    write32(p + 36, snapshot.faultBits);
    p[40] = snapshot.state;
    p[41] = snapshot.mode;
    p[42] = snapshot.calibrated;
    p[43] = snapshot.encoderValid;
    write16(p + 44, snapshot.encoderErrorCount);
    write16(p + 46, snapshot.txDropCount);
}

void appendSnapshot(stepper::Frame &response, const stepper::ControlSnapshot &snapshot)
{
    response.length = 50;
    encodeSnapshot(snapshot, response.payload + 2);
}

void appendInfo(stepper::Frame &response)
{
    response.length = 52;
    uint8_t *p = response.payload + 2;
    write16(p + 0, 1);
    write16(p + 2, 1);
    write16(p + 4, 0);
    write16(p + 6, 0);
    write16(p + 8, 1);
    // Use the existing HAL boundary instead of directly dereferencing ROM.
    // Explicit little-endian serialization preserves the V1 UID layout.
    write32(p + 10, HAL_GetUIDw0());
    write32(p + 14, HAL_GetUIDw1());
    write32(p + 18, HAL_GetUIDw2());
    // Feature bits describe complete usable command groups, not merely
    // selectable controller modes. Motion targets are still unsupported.
    write32(p + 22, 0x00000180UL); // CLEAR_FAULT | TELEMETRY
    write32(p + 26, uint32_t(motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
    write32(p + 30, uint32_t(boardConfig.currentLimit));
    write32(p + 34, uint32_t(boardConfig.calibrationCurrent));
    write32(p + 38, uint32_t(boardConfig.velocityLimit));
    write32(p + 42, uint32_t(boardConfig.velocityAcc));
    write32(p + 46, 1);
}

bool validTelemetryPeriod(uint16_t period)
{
    return period == 0 || period == 10 || period == 20 || period == 50 || period == 100;
}

bool knownButUnsupported(uint16_t command)
{
    return command == 0x0005 || (command >= 0x0103 && command <= 0x0105) ||
           command == 0x0107 || (command >= 0x0201 && command <= 0x0203) ||
           (command >= 0x0301 && command <= 0x0303);
}

bool queueControl(const stepper::Frame &request, stepper::ControlKind kind,
                  uint8_t mode, uint32_t session)
{
    int freeSlot = -1;
    for (unsigned i = 0; i < 3; ++i)
        if (!pending[i].valid)
        {
            freeSlot = int(i);
            break;
        }
    if (freeSlot < 0)
        return false;
    uint32_t requestId = nextRequestId++;
    if (!requestId)
        requestId = nextRequestId++;
    stepper::ControlCommand command = {kind, mode, requestId, session};
    if (!mailbox.submit(command))
        return false;
    pending[freeSlot].valid = true;
    pending[freeSlot].requestId = requestId;
    pending[freeSlot].request = request;
    // Reserve the sequence on acceptance, not on asynchronous completion.
    if (kind != stepper::ControlKind::Hello || session == cachedSession)
        highWater = request.sequence;
    return true;
}

void handleFrame(void *, const stepper::Frame &request)
{
    ++uartDiagValidFrames;
    const uint32_t nowMs = HAL_GetTick();
    if (request.type != 1 || !request.sequence || request.length > 128)
        return;

    static stepper::ControlSnapshot snapshot; // Main-loop parser is non-reentrant.
    snapshot = controlService.snapshot();
    const uint32_t activeSession = snapshot.session;
    if (activeSession != cachedSession)
    {
        clearTransactions();
        // A control interrupt may already have applied HELLO while its reply
        // is still pending. Never discard the request-to-completion mapping.
        cachedSession = activeSession;
        telemetryPeriodMs = 0;
    }

    const bool hello = request.command == CMD_HELLO;
    if ((hello && request.session != 0) || (!hello && (!activeSession || request.session != activeSession)))
    {
        enqueueFrame(makeResponse(request, STATUS_BAD_SESSION), false);
        return;
    }

    const int cachedIndex = findCached(request, nowMs);
    if (cachedIndex >= 0)
    {
        enqueueFrame(cached[cachedIndex].response, false);
        return;
    }
    const bool newSessionHello = hello && request.length == 4 &&
                                 read32(request.payload) != 0 &&
                                 read32(request.payload) != activeSession;
    const int pendingIndex = findPending(request);
    if (cachedIndex == -2 || pendingIndex == -2)
    {
        enqueueFrame(makeResponse(request, STATUS_SEQ_CONFLICT), false);
        return;
    }
    if (pendingIndex >= 0)
        return;
    if (!newSessionHello && !forwardSequence(request.sequence))
    {
        enqueueFrame(makeResponse(request, STATUS_SEQ_CONFLICT), false);
        return;
    }

    stepper::Frame &response = makeResponse(request, STATUS_OK);
    if (hello)
    {
        if (request.length != 4 || !read32(request.payload))
        {
            sendAndRemember(request, makeResponse(request, STATUS_BAD_PAYLOAD), nowMs);
            return;
        }
        if (read32(request.payload) != activeSession &&
            (encoder_calibrator_base.isTriggered ||
             motor.controller->modeRunning != Motor::MODE_STOP))
        {
            sendAndRemember(request, makeResponse(request, STATUS_WRONG_STATE), nowMs);
            return;
        }
        const uint32_t token = read32(request.payload);
        if (!queueControl(request, stepper::ControlKind::Hello, 3, token))
            sendAndRemember(request, makeResponse(request, STATUS_BUSY), nowMs);
        return;
    }

    switch (request.command)
    {
    case CMD_GET_INFO:
        if (request.length)
            response = makeResponse(request, STATUS_BAD_PAYLOAD);
        else
            appendInfo(response);
        sendAndRemember(request, response, nowMs);
        return;
    case CMD_GET_STATUS:
        if (request.length)
            response = makeResponse(request, STATUS_BAD_PAYLOAD);
        else if (queueControl(request, stepper::ControlKind::ReadState, 3, activeSession))
            return;
        else
            response = makeResponse(request, STATUS_BUSY);
        break;
    case CMD_HEARTBEAT:
        if (request.length)
            response = makeResponse(request, STATUS_BAD_PAYLOAD);
        else if (queueControl(request, stepper::ControlKind::Heartbeat, 3, activeSession))
            return;
        else
            response = makeResponse(request, STATUS_BUSY);
        break;
    case CMD_ENABLE:
        if (request.length != 1)
            response = makeResponse(request, STATUS_BAD_PAYLOAD);
        else if (request.payload[0] > 2)
            response = makeResponse(request, STATUS_OUT_OF_RANGE);
        else if (queueControl(request, stepper::ControlKind::Enable, request.payload[0], activeSession))
            return;
        else
            response = makeResponse(request, STATUS_BUSY);
        break;
    case CMD_DISABLE:
    case CMD_STOP:
        if (request.length)
            response = makeResponse(request, STATUS_BAD_PAYLOAD);
        else if (queueControl(request, stepper::ControlKind::Stop, 3, activeSession))
            return;
        else
            response = makeResponse(request, STATUS_BUSY);
        break;
    case CMD_CLEAR_FAULT:
        if (request.length)
            response = makeResponse(request, STATUS_BAD_PAYLOAD);
        else if (queueControl(request, stepper::ControlKind::ClearFault, 3, activeSession))
            return;
        else
            response = makeResponse(request, STATUS_BUSY);
        break;
    case CMD_TELEMETRY_CONFIG:
        if (request.length != 2)
            response = makeResponse(request, STATUS_BAD_PAYLOAD);
        else if (!validTelemetryPeriod(read16(request.payload)))
            response = makeResponse(request, STATUS_OUT_OF_RANGE);
        else
        {
            telemetryPeriodMs = read16(request.payload);
            response.length = 4;
            write16(response.payload + 2, telemetryPeriodMs);
        }
        break;
    default:
        response = makeResponse(request, knownButUnsupported(request.command) ? STATUS_UNSUPPORTED : STATUS_UNKNOWN_CMD);
        break;
    }
    sendAndRemember(request, response, nowMs);
}

// Keep scheduler workspaces out of the parser caller's stack frame.
__attribute__((noinline)) void finishControls(uint32_t nowMs)
{
    // Completion draining never runs recursively or from an ISR.
    static stepper::ControlCompletion completion;
    while (mailbox.takeCompletion(completion))
    {
        int slot = -1;
        for (unsigned i = 0; i < 3; ++i)
            if (pending[i].valid && pending[i].requestId == completion.requestId)
            {
                slot = int(i);
                break;
            }
        if (slot < 0)
            continue;
        static stepper::Frame request;
        request = pending[slot].request;
        pending[slot].valid = false;
        stepper::Frame &response = makeResponse(request,
            completion.canceled ? STATUS_WRONG_STATE : completion.status);
        if (!completion.status && !completion.canceled)
        {
            if (request.command == CMD_HELLO)
            {
                if (cachedSession != completion.snapshot.session)
                {
                    clearTransactions();
                    telemetryPeriodMs = 0;
                }
                cachedSession = completion.snapshot.session;
                response.session = completion.snapshot.session;
                response.length = 4;
                write16(response.payload + 2, 1);
            }
            else if (request.command == CMD_HEARTBEAT)
            {
                response.length = 6;
                write32(response.payload + 2, nowMs);
            }
            else if (request.command == CMD_ENABLE)
            {
                response.length = 3;
                response.payload[2] = request.payload[0];
            }
            else
                appendSnapshot(response, completion.snapshot);
        }
        sendAndRemember(request, response, nowMs);
    }
}

void transmitNext()
{
    uint32_t saved = critical.enter();
    if (activeTx >= 0 || huart1.gState != HAL_UART_STATE_READY)
    {
        critical.leave(saved);
        return;
    }
    int slot = -1;
    for (unsigned i = 0; i < 4; ++i)
        if (txSlots[i].state == 1 && !txSlots[i].telemetry)
        {
            slot = int(i);
            break;
        }
    if (slot < 0)
        for (unsigned i = 0; i < 4; ++i)
            if (txSlots[i].state == 1)
            {
                slot = int(i);
                break;
            }
    if (slot < 0)
    {
        critical.leave(saved);
        return;
    }
    txSlots[slot].state = 3;
    activeTx = int8_t(slot);
    critical.leave(saved);
    if (HAL_UART_Transmit_DMA(&huart1, txSlots[slot].bytes, txSlots[slot].length) == HAL_OK)
        ++uartDiagTxStarts;
    else
    {
        saved = critical.enter();
        txSlots[slot].state = 0;
        activeTx = -1;
        incrementDrop();
        critical.leave(saved);
    }
}

__attribute__((noinline)) void emitTelemetry(uint32_t nowMs)
{
    static stepper::ControlSnapshot snapshot; // Main-loop scheduler only.
    snapshot = controlService.snapshot();
    if (!telemetryPeriodMs || !snapshot.session ||
        uint32_t(nowMs - lastTelemetryMs) < telemetryPeriodMs)
        return;
    lastTelemetryMs = nowMs;
    static stepper::Frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = 4;
    frame.session = snapshot.session;
    frame.sequence = ++telemetrySequence;
    frame.command = CMD_TELEMETRY;
    frame.length = 48;
    encodeSnapshot(snapshot, frame.payload);
    enqueueFrame(frame, true);
}
}

void UartProtocolInit()
{
    rxHead = rxTail = 0;
    parser.reset();
    motor.controller->StopAndReset();
}

void UartProtocolPoll(uint32_t nowMs)
{
    ++uartDiagPolls;
    static uint8_t bytes[64]; // Single main-loop consumer, never re-entered.
    uint16_t count = 0;
    uint32_t saved = critical.enter();
    const bool hadReceiveError = receiveError;
    receiveError = false;
    while (rxTail != rxHead && count < sizeof(bytes))
    {
        bytes[count++] = rxRing[rxTail];
        rxTail = uint16_t((rxTail + 1) % sizeof(rxRing));
    }
    critical.leave(saved);
    if (hadReceiveError)
        parser.reset();
    if (count)
        parser.feed(bytes, count, nowMs, handleFrame, 0);
    else
        parser.feed(0, 0, nowMs, handleFrame, 0);
    finishControls(nowMs);
    emitTelemetry(nowMs);
    transmitNext();
}

void UartProtocolControlTick(uint32_t nowMs)
{
    ++uartDiagControlTicks;
    // Called from TIM4 in normal mode OR main in diagnostic mode, never both.
    static stepper::Measurement measurement;
    memset(&measurement, 0, sizeof(measurement));
    measurement.position = motor.controller->GetPositionSteps();
    measurement.velocity = motor.controller->Get_est_velocity();
    measurement.currentCommandMa = motor.controller->GetCurrentCommandMa();
    measurement.calibrated = mt6816_base.IsCalibrated();
    measurement.encoderValid = mt6816_base.angleData.sampleValid;
    measurement.encoderErrorCount = mt6816_base.GetConsecutiveErrorCount();
    measurement.txDropCount = txDropCount;
    controlService.tick(nowMs, measurement);
}

void UartProtocolOnRx(const uint8_t *data, uint16_t length)
{
    uartDiagRxBytes += length;
    for (uint16_t i = 0; i < length; ++i)
    {
        const uint16_t next = uint16_t((rxHead + 1) % sizeof(rxRing));
        if (next == rxTail)
        {
            incrementDrop();
            break;
        }
        rxRing[rxHead] = data[i];
        rxHead = next;
    }
}

void UartProtocolOnTxComplete()
{
    const int8_t slot = activeTx;
    if (slot >= 0)
        txSlots[slot].state = 0;
    activeTx = -1;
}

void UartProtocolOnError()
{
    receiveError = true;
}

bool UartProtocolCanAcceptCanMotion()
{
    const stepper::ControlSnapshot snapshot = controlService.snapshot();
    return snapshot.session == 0 && snapshot.faultBits == 0;
}

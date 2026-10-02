#ifndef STEPPER_CONTROL_SERVICE_H
#define STEPPER_CONTROL_SERVICE_H

#include "control_gate.h"

namespace stepper
{
class CriticalSection
{
public:
    virtual ~CriticalSection() {}
    virtual uint32_t enter() = 0;
    virtual void leave(uint32_t savedMask) = 0;
};

enum class ControlKind : uint8_t
{
    Hello,
    Heartbeat,
    Enable,
    Stop,
    ClearFault,
    ReadState
};

struct ControlCommand
{
    ControlKind kind;
    uint8_t mode;
    uint32_t requestId;
    uint32_t session;
};

struct Measurement
{
    int32_t position;
    int32_t velocity;
    int32_t currentCommandMa;
    bool calibrated;
    bool encoderValid;
    uint16_t encoderErrorCount;
    uint16_t txDropCount;
};

struct ControlSnapshot
{
    uint32_t session;
    uint32_t timestampMs;
    uint32_t sampleCounter;
    int32_t position;
    int32_t targetPosition;
    int32_t velocity;
    int32_t targetVelocity;
    int32_t currentCommandMa;
    int32_t targetCurrentMa;
    int32_t positionError;
    uint32_t faultBits;
    uint8_t state;
    uint8_t mode;
    uint8_t calibrated;
    uint8_t encoderValid;
    uint16_t encoderErrorCount;
    uint16_t txDropCount;
};

struct ControlCompletion
{
    uint32_t requestId;
    uint16_t status;
    bool canceled;
    ControlSnapshot snapshot;
};

class ControlMailbox
{
public:
    explicit ControlMailbox(CriticalSection &critical);
    bool submit(const ControlCommand &command);
    bool takeCompletion(ControlCompletion &completion);
    bool takeCommand(ControlCommand &command);
    bool complete(const ControlCommand &command, const ControlCompletion &completion);
    void cancelPendingEnable(const ControlSnapshot &snapshot);

private:
    enum class State : uint8_t { Empty, Pending, Processing, Completed };
    struct Slot
    {
        ControlCommand command;
        ControlCompletion completion;
        State state;
    };
    static unsigned lane(ControlKind kind);
    CriticalSection &critical_;
    Slot slots_[3];
};

class ControlDriver
{
public:
    virtual ~ControlDriver() {}
    virtual uint8_t supportedModes() const = 0;
    virtual void disableAndReset() = 0;
    virtual bool enable(uint8_t mode, int32_t initialPosition) = 0;
};

class ControlService
{
public:
    ControlService(ControlMailbox &mailbox, CriticalSection &critical, ControlDriver &driver);
    void tick(uint32_t nowMs, const Measurement &measurement);
    ControlSnapshot snapshot();

private:
    ControlSnapshot makeSnapshot(uint32_t nowMs, const Measurement &measurement) const;
    uint16_t execute(const ControlCommand &command, uint32_t nowMs, const Measurement &measurement);
    void stopDriver();

    ControlMailbox &mailbox_;
    CriticalSection &critical_;
    ControlDriver &driver_;
    ControlGate gate_;
    ControlSnapshot published_;
    uint32_t counter_;
    int32_t positionTarget_;
    bool initialized_;
    bool appliedEnabled_;
};
}

#endif

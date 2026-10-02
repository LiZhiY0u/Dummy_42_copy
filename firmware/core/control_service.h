#ifndef STEPPER_CONTROL_SERVICE_H
#define STEPPER_CONTROL_SERVICE_H
#include "control_gate.h"
namespace stepper {
// Hardware implementation must save/restore interrupt mask and include compiler
// memory barriers. Do not substitute a blocking mutex in ISR context.
class CriticalSection {
public:
    virtual ~CriticalSection() {}
    virtual uint32_t enter()=0;
    virtual void leave(uint32_t savedMask)=0;
};
enum class ControlKind : uint8_t {Hello,Heartbeat,Enable,Stop,ClearFault,ReadState};
struct ControlCommand {ControlKind kind;uint8_t mode;uint32_t requestId,session;};
struct Measurement {
    int32_t position,velocity,currentCommandMa;
    bool calibrated,encoderValid;
    uint16_t encoderErrorCount,txDropCount;
};
// Session is local metadata; the remaining values map to the 48B wire snapshot.
// Never transmit this struct by memcpy (padding and metadata are not wire data).
struct ControlSnapshot {
    uint32_t session,timestampMs,sampleCounter;
    int32_t position,targetPosition,velocity,targetVelocity,currentCommandMa,targetCurrentMa,positionError;
    uint32_t faultBits;
    uint8_t state,mode,calibrated,encoderValid;
    uint16_t encoderErrorCount,txDropCount;
};
struct ControlCompletion {uint32_t requestId;uint16_t status;bool canceled;ControlSnapshot snapshot;};
class ControlMailbox {
public:
    explicit ControlMailbox(CriticalSection &critical);
    // Nonzero correlation ID must be unique across occupied lanes. STOP blocks
    // new Enable until its completion has been consumed by the main loop.
    bool submit(const ControlCommand &command);
    bool takeCompletion(ControlCompletion &completion);
    // Control owner only; hardware adapter invokes these from the control tick.
    bool takeCommand(ControlCommand &command);
    bool complete(const ControlCommand &command,const ControlCompletion &completion);
    void cancelPendingEnable(const ControlSnapshot &snapshot);
private:
    enum class State : uint8_t {Empty,Pending,Processing,Completed};
    struct Slot {ControlCommand command;ControlCompletion completion;State state;};
    static unsigned lane(ControlKind kind);
    CriticalSection &critical_;
    Slot slots_[3];
};
class ControlDriver {
public:
    virtual ~ControlDriver() {}
    virtual uint8_t supportedModes() const=0;
    // Must stop output, clear goals/planners/integrals and return synchronously.
    virtual void disableAndReset()=0;
    // Position initial target is measured position; velocity/current start at 0.
    // These hooks must have bounded ISR-safe implementations without allocation.
    virtual bool enable(uint8_t mode,int32_t initialPosition)=0;
};
class ControlService {
public:
    ControlService(ControlMailbox &mailbox,CriticalSection &critical,ControlDriver &driver);
    // One control owner (future 20kHz adapter); no CRC, IO waits or Flash here.
    // Measurement must already be coherent and expressed in protocol units.
    void tick(uint32_t nowMs,const Measurement &measurement);
    ControlSnapshot snapshot();
private:
    ControlSnapshot makeSnapshot(uint32_t nowMs,const Measurement &measurement) const;
    uint16_t execute(const ControlCommand &command,uint32_t nowMs,const Measurement &measurement);
    void stopDriver();
    ControlMailbox &mailbox_;
    CriticalSection &critical_;
    ControlDriver &driver_;
    ControlGate gate_;
    ControlSnapshot published_;
    uint32_t counter_;
    int32_t positionTarget_;
    bool initialized_,appliedEnabled_;
};
}
#endif

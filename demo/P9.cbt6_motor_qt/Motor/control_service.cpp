#include "control_service.h"

namespace stepper
{
namespace
{
class Guard
{
public:
    explicit Guard(CriticalSection &critical) : critical_(critical), saved_(critical.enter()) {}
    ~Guard() { critical_.leave(saved_); }

private:
    Guard(const Guard &);
    Guard &operator=(const Guard &);
    CriticalSection &critical_;
    uint32_t saved_;
};

int32_t saturate(int64_t value)
{
    if (value > 2147483647LL)
        return 2147483647;
    if (value < (-2147483647LL - 1))
        return (-2147483647 - 1);
    return int32_t(value);
}
}

unsigned ControlMailbox::lane(ControlKind kind)
{
    return kind == ControlKind::Stop ? 2 : kind == ControlKind::Heartbeat ? 1 : 0;
}

ControlMailbox::ControlMailbox(CriticalSection &critical) : critical_(critical)
{
    for (unsigned i = 0; i < 3; ++i)
        slots_[i].state = State::Empty;
}

bool ControlMailbox::submit(const ControlCommand &command)
{
    if (!command.requestId || uint8_t(command.kind) > uint8_t(ControlKind::ReadState))
        return false;
    Guard guard(critical_);
    Slot &slot = slots_[lane(command.kind)];
    if (command.kind == ControlKind::Enable && slots_[2].state != State::Empty)
        return false;
    for (unsigned i = 0; i < 3; ++i)
        if (slots_[i].state != State::Empty && slots_[i].command.requestId == command.requestId)
            return false;
    if (slot.state != State::Empty)
        return false;
    slot.command = command;
    slot.state = State::Pending;
    return true;
}

bool ControlMailbox::takeCommand(ControlCommand &command)
{
    Guard guard(critical_);
    for (int i = 2; i >= 0; --i)
        if (slots_[i].state == State::Pending)
        {
            command = slots_[i].command;
            slots_[i].state = State::Processing;
            return true;
        }
    return false;
}

bool ControlMailbox::complete(const ControlCommand &command, const ControlCompletion &completion)
{
    Guard guard(critical_);
    Slot &slot = slots_[lane(command.kind)];
    if (slot.state != State::Processing || slot.command.requestId != command.requestId ||
        completion.requestId != command.requestId)
        return false;
    slot.completion = completion;
    slot.state = State::Completed;
    return true;
}

bool ControlMailbox::takeCompletion(ControlCompletion &completion)
{
    Guard guard(critical_);
    for (int i = 2; i >= 0; --i)
        if (slots_[i].state == State::Completed)
        {
            completion = slots_[i].completion;
            slots_[i].state = State::Empty;
            return true;
        }
    return false;
}

void ControlMailbox::cancelPendingEnable(const ControlSnapshot &snapshot)
{
    Guard guard(critical_);
    Slot &slot = slots_[0];
    if (slot.state == State::Pending && slot.command.kind == ControlKind::Enable)
    {
        slot.completion = ControlCompletion();
        slot.completion.requestId = slot.command.requestId;
        slot.completion.status = 5;
        slot.completion.canceled = true;
        slot.completion.snapshot = snapshot;
        slot.state = State::Completed;
    }
}

ControlService::ControlService(ControlMailbox &mailbox, CriticalSection &critical, ControlDriver &driver)
    : mailbox_(mailbox), critical_(critical), driver_(driver), published_(), counter_(0),
      positionTarget_(0), initialized_(false), appliedEnabled_(false)
{
    published_.mode = 3;
}

void ControlService::stopDriver()
{
    gate_.stop();
    driver_.disableAndReset();
    appliedEnabled_ = false;
    positionTarget_ = 0;
}

uint16_t ControlService::execute(const ControlCommand &command, uint32_t now,
                                 const Measurement &measurement)
{
    if (command.kind == ControlKind::Hello)
        return uint16_t(gate_.hello(command.session, now));
    if (!gate_.session() || command.session != gate_.session())
        return 11;
    switch (command.kind)
    {
    case ControlKind::Heartbeat:
        return uint16_t(gate_.heartbeat(command.session, now));
    case ControlKind::Enable:
    {
        if (command.mode > 2)
            return 4;
        if (!(driver_.supportedModes() & (1U << command.mode)))
            return 10;
        const uint16_t status = uint16_t(gate_.enable(command.mode, now));
        if (status)
            return status;
        if (appliedEnabled_)
            return 0;
        positionTarget_ = command.mode == 0 ? measurement.position : 0;
        if (!driver_.enable(command.mode, positionTarget_))
        {
            stopDriver();
            return 13;
        }
        appliedEnabled_ = true;
        return 0;
    }
    case ControlKind::Stop:
        stopDriver();
        mailbox_.cancelPendingEnable(makeSnapshot(now, measurement));
        return 0;
    case ControlKind::ClearFault:
    {
        const uint16_t status = uint16_t(gate_.clearFault());
        if (!status)
            stopDriver();
        return status;
    }
    case ControlKind::ReadState:
        return 0;
    default:
        return 2;
    }
}

ControlSnapshot ControlService::makeSnapshot(uint32_t now, const Measurement &m) const
{
    ControlSnapshot s = {};
    s.session = gate_.session();
    s.timestampMs = now;
    s.sampleCounter = counter_;
    s.position = m.position;
    s.velocity = m.velocity;
    s.currentCommandMa = gate_.enabled() ? m.currentCommandMa : 0;
    s.targetPosition = gate_.enabled() && gate_.mode() == 0 ? positionTarget_ : m.position;
    s.positionError = saturate(int64_t(s.targetPosition) - int64_t(m.position));
    s.faultBits = gate_.faults();
    s.state = gate_.faults() ? 4 : gate_.enabled() ? 1 : 0;
    s.mode = gate_.mode();
    s.calibrated = uint8_t(m.calibrated);
    s.encoderValid = uint8_t(m.encoderValid);
    s.encoderErrorCount = m.encoderErrorCount;
    s.txDropCount = m.txDropCount;
    return s;
}

void ControlService::tick(uint32_t now, const Measurement &measurement)
{
    ++counter_;
    gate_.setSensors(measurement.encoderValid, measurement.calibrated);
    gate_.tick(now);
    if (!initialized_ || (appliedEnabled_ && !gate_.enabled()))
    {
        stopDriver();
        initialized_ = true;
    }
    for (unsigned i = 0; i < 3; ++i)
    {
        ControlCommand command;
        if (!mailbox_.takeCommand(command))
            break;
        ControlCompletion completion = {};
        completion.requestId = command.requestId;
        completion.status = execute(command, now, measurement);
        completion.snapshot = makeSnapshot(now, measurement);
        mailbox_.complete(command, completion);
    }
    const ControlSnapshot next = makeSnapshot(now, measurement);
    Guard guard(critical_);
    published_ = next;
}

ControlSnapshot ControlService::snapshot()
{
    Guard guard(critical_);
    return published_;
}
}

#ifndef STEPPER_COMMAND_DISPATCHER_H
#define STEPPER_COMMAND_DISPATCHER_H
#include "protocol_v1.h"
namespace stepper {
class CommandBackend {
public:
    virtual ~CommandBackend() {}
    // Single-owner contract: refresh watchdog/safety state before each request.
    // Also required periodically when no incoming requests exist.
    virtual void poll(uint32_t nowMs)=0;
    virtual uint32_t sessionId() const=0;
    virtual uint16_t beginSession(uint32_t token,uint32_t nowMs)=0;
    // On return, data.length/payload contain response DATA, without STATUS.
    // Must confirm actual operation before reporting OK. Pending hardware work
    // requires a future asynchronous adapter, not a blocking wait here.
    virtual uint16_t execute(const Frame &request,uint32_t nowMs,Frame &data)=0;
};
class CommandDispatcher {
public:
    explicit CommandDispatcher(CommandBackend &backend);
    // Main-loop/single-owner use only; false means no response (wrong TYPE/SEQ/LEN).
    bool handle(const Frame &request,uint32_t nowMs,Frame &response);
    void reset();
private:
    struct Entry {Frame request,response;uint32_t timestamp;bool valid;};
    int find(const Frame &request,uint32_t nowMs) const;
    bool forward(uint16_t sequence) const;
    void remember(const Frame &request,const Frame &response,uint32_t nowMs);
    CommandBackend &backend_;
    Entry entries_[4];
    uint32_t cachedSession_;
    uint16_t highWater_;
    uint8_t next_;
};
}
#endif

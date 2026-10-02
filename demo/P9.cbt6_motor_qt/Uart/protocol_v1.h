#ifndef STEPPER_PORTABLE_PROTOCOL_V1_H
#define STEPPER_PORTABLE_PROTOCOL_V1_H

#include <stddef.h>
#include <stdint.h>

namespace stepper
{
struct Frame
{
    uint8_t type;
    uint32_t session;
    uint16_t sequence;
    uint16_t command;
    uint16_t length;
    uint8_t payload[128];
};

uint16_t crc16(const uint8_t *bytes, size_t length);
size_t encode(const Frame &frame, uint8_t *output, size_t capacity);

class Parser
{
public:
    typedef void (*Receiver)(void *context, const Frame &frame);

    Parser() : count_(0), candidateSince_(0), candidate_(false) {}
    void feed(const uint8_t *bytes, size_t length, uint32_t nowMs,
              Receiver receiver, void *context);
    void reset();

private:
    void parse(uint32_t nowMs, Receiver receiver, void *context);
    void discard(size_t length);

    uint8_t buffer_[144];
    size_t count_;
    uint32_t candidateSince_;
    bool candidate_;
};
}

#endif

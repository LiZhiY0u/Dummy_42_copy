#ifndef STEPPER_PORTABLE_PROTOCOL_V1_H
#define STEPPER_PORTABLE_PROTOCOL_V1_H
#include <stdint.h>
#include <stddef.h>
namespace stepper {
struct Frame {
    uint8_t type;
    uint32_t session;
    uint16_t sequence,command,length;
    uint8_t payload[128];
};
uint16_t crc16(const uint8_t *bytes,size_t length);
// Zero on invalid frame or insufficient output capacity; output then unchanged.
size_t encode(const Frame &frame,uint8_t *output,size_t capacity);
class Parser {
public:
    typedef void (*Receiver)(void *context,const Frame &frame);
    Parser():count_(0),candidateSince_(0),candidate_(false) {}
    // Main-loop only. Receiver must not recursively feed/reset this parser.
    // Frame is valid only for the duration of the synchronous callback.
    void feed(const uint8_t *bytes,size_t length,uint32_t nowMs,Receiver receiver,void *context);
    void reset();
    size_t buffered() const {return count_;}
private:
    void parse(uint32_t nowMs,Receiver receiver,void *context);
    void discard(size_t length);
    uint8_t buffer_[144];
    size_t count_;
    uint32_t candidateSince_;
    bool candidate_;
};
}
#endif

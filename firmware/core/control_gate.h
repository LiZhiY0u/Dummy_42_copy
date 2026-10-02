#ifndef STEPPER_PORTABLE_CONTROL_GATE_H
#define STEPPER_PORTABLE_CONTROL_GATE_H
#include <stdint.h>
namespace stepper {
enum class Result : uint16_t {Ok=0,OutOfRange=4,WrongState=5,NotCalibrated=6,EncoderFault=7,BadSession=11};
// Single-owner logical safety gate. NOT a HAL driver or an ISR mailbox.
// Integration must apply !enabled() to driver output at each control boundary.
class ControlGate {
public:
    ControlGate():session_(0),lastHeartbeat_(0),faults_(0),mode_(3),enabled_(false),encoderValid_(false),calibrated_(false) {}
    Result hello(uint32_t token,uint32_t nowMs);
    Result heartbeat(uint32_t token,uint32_t nowMs);
    Result enable(uint8_t mode,uint32_t nowMs);
    Result clearFault();
    void stop();
    void tick(uint32_t nowMs);
    void setSensors(bool encoderValid,bool calibrated);
    bool enabled() const {return enabled_;}
    uint32_t session() const {return session_;}
    uint32_t faults() const {return faults_;}
    uint8_t mode() const {return mode_;}
    bool canAcceptCanMotion() const {return session_==0&&!faults_;}
private:
    uint32_t session_,lastHeartbeat_,faults_;
    uint8_t mode_;
    bool enabled_,encoderValid_,calibrated_;
};
}
#endif

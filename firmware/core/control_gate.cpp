#include "control_gate.h"
namespace stepper {
void ControlGate::stop() {enabled_=false;mode_=3;}
Result ControlGate::hello(uint32_t token,uint32_t nowMs) {
    tick(nowMs);
    if(!token)return Result::OutOfRange;
    if(token==session_)return Result::Ok;
    if(enabled_)return Result::WrongState;
    session_=token;lastHeartbeat_=nowMs;return Result::Ok;
}
Result ControlGate::heartbeat(uint32_t token,uint32_t nowMs) {
    tick(nowMs);
    if(!session_||token!=session_)return Result::BadSession;
    lastHeartbeat_=nowMs;return Result::Ok;
}
void ControlGate::tick(uint32_t nowMs) {
    if(session_&&uint32_t(nowMs-lastHeartbeat_)>=500){stop();session_=0;faults_|=4;}
}
void ControlGate::setSensors(bool encoderValid,bool calibrated) {
    encoderValid_=encoderValid;calibrated_=calibrated;
    if(!encoderValid_){stop();faults_|=1;}
    if(!calibrated_)stop();
}
Result ControlGate::enable(uint8_t mode,uint32_t nowMs) {
    tick(nowMs);
    if(!session_)return Result::BadSession;
    if(mode>2)return Result::OutOfRange;
    if(!encoderValid_)return Result::EncoderFault;
    if(faults_)return Result::WrongState;
    if(!calibrated_)return Result::NotCalibrated;
    if(enabled_&&mode!=mode_)return Result::WrongState;
    mode_=mode;enabled_=true;return Result::Ok;
}
Result ControlGate::clearFault() {
    if(enabled_)return Result::WrongState;
    if(!encoderValid_)return Result::EncoderFault;
    faults_=0;stop();return Result::Ok;
}
}

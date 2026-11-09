#ifndef P9_POSITION_MOVE_H
#define P9_POSITION_MOVE_H
#include <stdint.h>
namespace stepper { namespace p9 {
// Application-only task policy. No IO, scheduler, trajectory generation or enable.
// Caller supplies limits; units: subdivision steps, steps/s, steps/s^2, ms.
struct MoveLimits {
    uint32_t distance,velocity,acceleration,positionTolerance,velocityTolerance;
    uint32_t settledSamples,timeoutMs;
};
struct MoveRequest { int32_t position;uint32_t velocity,acceleration; };
struct MoveState { int32_t position,velocity;bool enabled,calibrated,encoderValid;uint8_t mode;uint32_t faults; };
class PositionMove {
public:
    enum Phase { Idle=0,Running=1,Planned=2,Arrived=3,Cancelled=4,Failed=5 };
    PositionMove():limits_(),request_(),started_(0),settled_(0),phase_(Idle),result_(0){}
    uint16_t accept(const MoveRequest &r,const MoveLimits &limits,const MoveState &s,uint32_t now) {
        if(active())return 8;
        if(!s.encoderValid)return 7;
        if(!s.calibrated)return 6;
        if(!s.enabled || s.mode!=0 || s.faults)return 5;
        if(!limits.distance || !limits.velocity || !limits.acceleration ||
           !limits.settledSamples || !limits.timeoutMs || limits.timeoutMs>120000U ||
           !r.velocity || r.velocity>0x7fffffffU || r.velocity>limits.velocity ||
           !r.acceleration || r.acceleration>0x7fffffffU || r.acceleration>limits.acceleration ||
           magnitude(int64_t(r.position)-s.position)>limits.distance)return 4;
        limits_=limits;request_=r;started_=now;settled_=0;phase_=Running;result_=1;
        return 1;
    }
    bool planned() {
        if(phase_!=Running)return false;
        phase_=Planned;settled_=0;return true;
    }
    // Call only after the current task's CalcSoftGoal, not stale NewTask outputs.
    bool observePlan(int32_t plannedPosition,int32_t plannedVelocity) {
        return phase_==Running && plannedPosition==request_.position &&
            plannedVelocity==0 && planned();
    }
    void observe(const MoveState &s,uint32_t now) {
        if(!active())return;
        if(!s.encoderValid){fail(7);return;}
        if(!s.calibrated){fail(6);return;}
        if(s.faults || !s.enabled || s.mode!=0){fail(5);return;}
        // Unsigned subtraction permits a normal uint32 clock rollover.
        if(uint32_t(now-started_)>=limits_.timeoutMs){fail(5);return;}
        if(phase_!=Planned)return;
        if(magnitude(int64_t(request_.position)-s.position)<=limits_.positionTolerance &&
           magnitude(int64_t(s.velocity))<=limits_.velocityTolerance) {
            if(++settled_>=limits_.settledSamples){phase_=Arrived;result_=0;}
        } else settled_=0;
    }
    void cancel(){if(active()){phase_=Cancelled;result_=0;settled_=0;}}
    bool active()const{return phase_==Running || phase_==Planned;}
    Phase phase()const{return phase_;}
    uint16_t result()const{return result_;}
    int32_t target()const{return request_.position;}
private:
    static uint64_t magnitude(int64_t n){return uint64_t(n<0?-n:n);}
    void fail(uint16_t status){phase_=Failed;result_=status;settled_=0;}
    MoveLimits limits_;
    MoveRequest request_;
    uint32_t started_,settled_;
    Phase phase_;
    uint16_t result_;
};
} }
#endif

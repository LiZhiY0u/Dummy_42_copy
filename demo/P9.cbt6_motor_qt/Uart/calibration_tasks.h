#ifndef P9_CALIBRATION_TASKS_H
#define P9_CALIBRATION_TASKS_H
#include <stdint.h>

namespace stepper { namespace p9 {
// Control-tick owner; callers must exclude IRQs when copying records in main.
// No IO or motor calls: recording ACCEPTED must never itself start hardware.
struct CalibrationTask {
    uint32_t id, session, timestamp;
    uint16_t result;
    uint8_t phase, taskPhase, percent, storageVerified;
};
inline void taskWrite32(uint8_t *p,uint32_t n) {
    for(unsigned i=0;i<4;++i)p[i]=uint8_t(n>>(8*i));
}
// Response body after the common u16 STATUS. Revision 1 layouts, little endian.
inline void encodeTaskQuery(const CalibrationTask &task,uint8_t (&out)[12]) {
    taskWrite32(out,task.id);out[4]=3;out[5]=task.taskPhase;
    out[6]=uint8_t(task.result);out[7]=uint8_t(task.result>>8);
    taskWrite32(out+8,task.timestamp);
}
inline void encodeCalibrationQuery(const CalibrationTask &task,uint8_t (&out)[13]) {
    taskWrite32(out,task.id);out[4]=task.phase;out[5]=task.percent;
    out[6]=uint8_t(task.result);out[7]=uint8_t(task.result>>8);
    out[8]=task.storageVerified;taskWrite32(out+9,task.timestamp);
}
class CalibrationTasks {
public:
    CalibrationTasks() : active_(), history_(), nextId_(1), owner_(0), cursor_(0) {}
    void session(uint32_t token) {
        if (token==owner_) return;
        active_=CalibrationTask();
        for(unsigned i=0;i<4;++i) history_[i]=CalibrationTask();
        owner_=token;cursor_=0;
    }
    uint32_t accept(uint32_t token,uint32_t now) {
        if (!token || token!=owner_ || active_.id) return 0;
        active_=CalibrationTask();
        // Wrap safely; skip zero and any retained identifier.
        do { active_.id=nextId_++; } while(!active_.id || retained(active_.id));
        active_.session=token;active_.timestamp=now;
        active_.result=1; // ACCEPTED
        return active_.id;
    }
    bool progress(uint32_t id,uint8_t phase,uint8_t percent,uint32_t now) {
        if (!id || active_.id!=id || phase>4 || percent>99 ||
            phase<active_.phase || phase>active_.phase+1 || percent<active_.percent) return false;
        active_.phase=phase;active_.taskPhase=phase?1:0;
        active_.percent=percent;active_.timestamp=now;return true;
    }
    bool cancel(uint32_t id,uint32_t now) {
        if(!id || active_.id!=id)return false;
        active_.phase=6;active_.taskPhase=4;active_.result=0;
        active_.storageVerified=0;active_.timestamp=now;finish();return true;
    }
    bool fail(uint32_t id,uint16_t result,uint32_t now) {
        if(!id || active_.id!=id || result<2 || result>13)return false;
        active_.phase=6;active_.taskPhase=5;active_.result=result;
        active_.storageVerified=0;active_.timestamp=now;finish();return true;
    }
    bool completeVerified(uint32_t id,uint32_t now) {
        // Completion is legal only after explicit verification phase.
        if(!id || active_.id!=id || active_.phase!=4)return false;
        active_.phase=5;active_.taskPhase=6;active_.result=0;
        active_.percent=100;active_.storageVerified=1;active_.timestamp=now;finish();return true;
    }
    bool query(uint32_t token,uint32_t id,CalibrationTask &out) const {
        if(!token || token!=owner_ || !id)return false;
        if(active_.id==id){out=active_;return true;}
        for(unsigned i=0;i<4;++i)if(history_[i].id==id){out=history_[i];return true;}
        return false;
    }
private:
    bool retained(uint32_t id) const {
        for(unsigned i=0;i<4;++i)if(history_[i].id==id)return true;
        return false;
    }
    void finish() {
        history_[cursor_]=active_;cursor_=uint8_t((cursor_+1)%4);
        active_=CalibrationTask();
    }
    CalibrationTask active_,history_[4];
    uint32_t nextId_,owner_;
    uint8_t cursor_;
};
} }
#endif

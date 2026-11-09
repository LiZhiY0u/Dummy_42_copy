#include "calibration_tasks.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define CHECK(c) do { if(!(c)) {std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);std::exit(1);} } while(0)
int main() {
    stepper::p9::CalibrationTasks tasks;
    stepper::p9::CalibrationTask state={};
    CHECK(!tasks.accept(0,0) && !tasks.accept(7,0));
    tasks.session(7);
    uint32_t id=tasks.accept(7,10);
    CHECK(id && !tasks.accept(7,11) && !tasks.accept(8,11));
    CHECK(tasks.query(7,id,state) && state.phase==0 && state.result==1);
    CHECK(!tasks.query(8,id,state) && !tasks.query(7,0,state));
    CHECK(!tasks.completeVerified(id,12));
    CHECK(!tasks.progress(id,3,50,12));
    CHECK(tasks.progress(id,1,10,12));
    CHECK(!tasks.progress(id,0,10,12) && !tasks.progress(id,1,9,12));
    CHECK(!tasks.progress(id,1,100,12));
    CHECK(tasks.progress(id,2,40,13) && tasks.progress(id,3,70,14));
    CHECK(!tasks.completeVerified(id,15));
    CHECK(tasks.progress(id,4,99,15) && tasks.completeVerified(id,16));
    CHECK(!tasks.cancel(id,17) && !tasks.fail(id,9,17));
    CHECK(tasks.query(7,id,state) && state.storageVerified==1 && state.percent==100);
    CHECK(state.phase==5 && state.taskPhase==6 && state.timestamp==16);
    // Retain exactly four terminal results independent of replay-cache TTL.
    uint32_t ids[4];
    for(unsigned i=0;i<4;++i) {
        ids[i]=tasks.accept(7,2001+i);
        CHECK(ids[i] && !tasks.fail(ids[i],0,2001+i));
        if(i==3) CHECK(tasks.fail(ids[i],9,2001+i));
        else CHECK(tasks.cancel(ids[i],2001+i));
    }
    CHECK(!tasks.query(7,id,state));
    for(unsigned i=0;i<4;++i) CHECK(tasks.query(7,ids[i],state));
    CHECK(state.phase==6 && state.taskPhase==5 && state.result==9 && !state.storageVerified);
    CHECK(tasks.query(7,ids[0],state) && state.phase==6 && state.taskPhase==4);
    tasks.session(7);CHECK(tasks.query(7,ids[0],state));
    tasks.session(8);CHECK(!tasks.query(7,ids[0],state) && !tasks.query(8,ids[0],state));
    uint32_t newer=tasks.accept(8,3000);CHECK(newer && newer!=ids[0]);
    tasks.session(0);CHECK(!tasks.query(8,newer,state) && !tasks.accept(8,3001));
    // A late write/verify completion must not overwrite a cancellation result.
    tasks.session(9);
    for(unsigned phase=3;phase<=4;++phase) {
        uint32_t canceled=tasks.accept(9,10);
        for(unsigned p=1;p<=phase;++p) CHECK(tasks.progress(canceled,uint8_t(p),uint8_t(p*20),10+p));
        CHECK(tasks.cancel(canceled,20));
        CHECK(!tasks.completeVerified(canceled,21) && !tasks.progress(canceled,4,99,21));
        CHECK(tasks.query(9,canceled,state) && state.taskPhase==4 && !state.storageVerified);
    }
    // Independent wire vectors; the common STATUS is not part of these bodies.
    state={0x12345678,9,0x11223344,9,6,5,80,0};
    uint8_t generic[12],calibration[13];
    stepper::p9::encodeTaskQuery(state,generic);
    stepper::p9::encodeCalibrationQuery(state,calibration);
    const uint8_t expectedGeneric[]={0x78,0x56,0x34,0x12,3,5,9,0,0x44,0x33,0x22,0x11};
    const uint8_t expectedCalibration[]={0x78,0x56,0x34,0x12,6,80,9,0,0,0x44,0x33,0x22,0x11};
    CHECK(std::memcmp(generic,expectedGeneric,sizeof generic)==0);
    CHECK(std::memcmp(calibration,expectedCalibration,sizeof calibration)==0);
    std::printf("PASS: calibration task ownership, monotonic phases, verified completion, four-result history; RAM=%u\n",
                unsigned(sizeof(tasks)));
}

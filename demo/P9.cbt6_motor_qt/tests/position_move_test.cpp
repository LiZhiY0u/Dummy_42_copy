#include <cstdio>
#include <cstdlib>
#include <climits>
#include "position_move.h"
#define CHECK(c) do{if(!(c)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);std::exit(1);}}while(0)
using namespace stepper::p9;
int main(){
    // Fixture limits, not approved physical operating limits. Observe once/ms.
    MoveLimits limits={1000,500,1000,2,3,3,120000};
    MoveState s={0,0,true,true,true,0,0};
    MoveRequest r={1000,500,1000};PositionMove move;
    CHECK(move.accept(r,limits,s,0)==1 && move.target()==1000);
    CHECK(move.accept(r,limits,s,0)==8);
    s.position=1000;for(unsigned i=0;i<5;++i)move.observe(s,i);
    CHECK(move.phase()==PositionMove::Running); // near target != planner completion
    CHECK(move.planned());move.observe(s,6);move.observe(s,7);
    s.velocity=4;move.observe(s,8);s.velocity=0;
    move.observe(s,9);move.observe(s,10);CHECK(move.active());
    move.observe(s,11);CHECK(move.phase()==PositionMove::Arrived && move.result()==0);
    move.cancel();CHECK(move.phase()==PositionMove::Arrived);
    s.position=0;CHECK(move.accept(r,limits,s,20)==1);
    s.encoderValid=false;move.observe(s,21);CHECK(move.result()==7 && !move.active());
    s.encoderValid=true;move.observe(s,22);CHECK(move.phase()==PositionMove::Failed);
    s.calibrated=false;CHECK(move.accept(r,limits,s,23)==6);s.calibrated=true;
    s.enabled=false;CHECK(move.accept(r,limits,s,23)==5);s.enabled=true;
    s.faults=4;CHECK(move.accept(r,limits,s,23)==5);s.faults=0;
    s.mode=1;CHECK(move.accept(r,limits,s,23)==5);s.mode=0;
    r.position=1001;CHECK(move.accept(r,limits,s,24)==4);r.position=1000;
    r.velocity=501;CHECK(move.accept(r,limits,s,24)==4);r.velocity=500;
    r.acceleration=0;CHECK(move.accept(r,limits,s,24)==4);r.acceleration=1000;
    s.position=INT_MIN;r.position=INT_MAX;CHECK(move.accept(r,limits,s,24)==4);
    s.position=INT_MAX;r.position=INT_MIN;CHECK(move.accept(r,limits,s,24)==4);
    s.position=0;r.position=-1000;
    CHECK(move.accept(r,limits,s,UINT_MAX-10U)==1);
    move.observe(s,19);CHECK(move.active());move.cancel();CHECK(move.phase()==PositionMove::Cancelled);
    CHECK(move.accept(r,limits,s,100)==1);move.observe(s,120100);
    CHECK(move.phase()==PositionMove::Failed && move.result()==5);
    limits.settledSamples=0;CHECK(move.accept(r,limits,s,0)==4);
    std::puts("PASS: bounded position task, planned/actual distinction, settling, timeout, overflow and cancel");
}

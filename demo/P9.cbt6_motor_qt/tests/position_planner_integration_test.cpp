#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include "position_move.h"
#include "motion_planner.h"
#include "control_gate.h"
#define CHECK(c) do{if(!(c)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);std::exit(1);}}while(0)
using namespace stepper;
using namespace stepper::p9;
static void run(int direction,unsigned interruption) {
    MotionPlanner planner;MotionPlanner::Config_t config={};
    config.ratedVelocity=500;config.ratedVelocityAcc=1000;
    planner.AttachConfig(&config);planner.positionTracker.NewTask(0,0);
    ControlGate gate;gate.setSensors(true,true);
    CHECK(gate.hello(7,0)==Result::Ok && gate.enable(0,0)==Result::Ok);
    PositionMove task;MoveLimits limits={1000,500,1000,2,3,3,120000};
    MoveRequest request={direction*1000,500,1000};
    MoveState measured={0,0,true,true,true,0,0};
    CHECK(task.accept(request,limits,measured,0)==1);
    unsigned plannedAt=0,plannerCalls=0;bool planSeen=false;
    for(unsigned tick=1;tick<120000 && task.active();++tick) {
        const unsigned ms=tick/20; // real planner source period: 20kHz/50us
        if(tick%20==0) {
            if(interruption!=2 && ms%100==0)CHECK(gate.heartbeat(7,ms)==Result::Ok);
            if(interruption==1 && ms==200){gate.stop();task.cancel();}
            gate.tick(ms);
            measured.enabled=gate.enabled();measured.mode=gate.mode();measured.faults=gate.faults();
            task.observe(measured,ms);
        }
        if(!task.active())break; // STOP/fault processing precedes further planner work
        planner.positionTracker.CalcSoftGoal(request.position);++plannerCalls;
        // The existing tracker may reverse briefly to correct quantization overshoot.
        CHECK(std::abs(planner.positionTracker.go_velocity)<=500);
        if(task.observePlan(planner.positionTracker.go_location,planner.positionTracker.go_velocity)) {
            CHECK(!planSeen);planSeen=true;plannedAt=ms;
        }
        // Deliberate 20-step actual lag; plan completion cannot satisfy arrival.
        measured.position=planner.positionTracker.go_location-direction*20;
        measured.velocity=planner.positionTracker.go_velocity;
        if(planSeen && ms-plannedAt>=50){measured.position=request.position;measured.velocity=0;}
        if(planSeen && ms-plannedAt<50)CHECK(task.phase()==PositionMove::Planned);
    }
    CHECK(!task.active());
    const unsigned completedCalls=plannerCalls;
    for(unsigned i=0;i<100;++i){gate.tick(10000+i);task.observe(measured,10000+i);}
    CHECK(plannerCalls==completedCalls);
    if(interruption==0)CHECK(planSeen && task.phase()==PositionMove::Arrived);
    if(interruption==1)CHECK(task.phase()==PositionMove::Cancelled && !gate.enabled() && plannerCalls==3999);
    if(interruption==2)CHECK(task.phase()==PositionMove::Failed && (gate.faults()&4) && plannerCalls==9999);
    std::printf("PASS: real PositionTracker direction=%d interruption=%u plannerCalls=%u\n",direction,interruption,plannerCalls);
}
int main(){run(1,0);run(-1,0);run(1,1);run(1,2);}

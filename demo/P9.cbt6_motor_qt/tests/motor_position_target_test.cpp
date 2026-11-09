#include <cstdio>
#include <cstdlib>
#include <climits>
#include "motor.h"
#include "configurations.h"
#include "tb67h450_base.h"
#define CHECK(c) do{if(!(c)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);std::exit(1);}}while(0)
BoardConfig_t boardConfig={};
static uint16_t table[16384];
MT6816Base mt6816_base(table);
TB67H450Base tb67h450_base;
Motor motor;
bool MT6816Base::UpdateAngle(){return angleData.sampleValid;}
bool MT6816Base::IsCalibrated(){return angleData.rectifyValid;}
void TB67H450Base::Sleep(){}
void TB67H450Base::Brake(){CHECK(false);}
void TB67H450Base::SetFocCurrentVector(uint32_t,int32_t){} // Hardware output replaced, never IO.
int main(){
    boardConfig.velocityLimit=500;boardConfig.velocityAcc=1000;
    mt6816_base.angleData.sampleValid=mt6816_base.angleData.rectifyValid=true;
    motor.AttachEncoder(&mt6816_base);
    auto &c=*motor.controller;
    CHECK(!c.SetProtocolPositionTarget(100,500,1000));
    c.requestMode=c.modeRunning=Motor::MODE_COMMAND_POSITION;
    motor.config.motionParams.encoderHomeOffset=10;
    CHECK(c.SetProtocolPositionTarget(100,500,1000));
    CHECK(c.GetTargetPositionSteps()==100 && motor.config.motionParams.ratedVelocity==500 && motor.config.motionParams.ratedVelocityAcc==1000);
    CHECK(!c.IsProtocolPositionPlanComplete());
    const int32_t goal=c.GetTargetPositionSteps();
    CHECK(!c.SetProtocolPositionTarget(100,501,1000));
    CHECK(!c.SetProtocolPositionTarget(100,500,1001));
    CHECK(!c.SetProtocolPositionTarget(INT_MAX,500,1000));
    CHECK(c.GetTargetPositionSteps()==goal && motor.config.motionParams.ratedVelocity==500);
    motor.config.motionParams.encoderHomeOffset=-10;
    CHECK(!c.SetProtocolPositionTarget(INT_MIN,500,1000));
    motor.config.motionParams.encoderHomeOffset=0;
    CHECK(!c.SetProtocolPositionTarget(INT_MIN,500,1000));
    mt6816_base.angleData.sampleValid=false;CHECK(!c.SetProtocolPositionTarget(0,500,1000));
    mt6816_base.angleData.sampleValid=true;
    CHECK(c.SetProtocolPositionTarget(-100,500,1000));
    motor.Tick20kHz(); // first-ever tick only initializes absolute encoder position
    motor.Tick20kHz(); // consumes the pending new curve
    for(unsigned i=0;i<100000 && !c.IsProtocolPositionPlanComplete();++i)
        motor.motionPlanner.positionTracker.CalcSoftGoal(-100);
    CHECK(c.IsProtocolPositionPlanComplete()); // regression: formerly -101/40 after 5 seconds
    CHECK(c.SetProtocolPositionTarget(-1000,500,1000));motor.Tick20kHz();
    for(unsigned i=0;i<100000 && !c.IsProtocolPositionPlanComplete();++i)
        motor.motionPlanner.positionTracker.CalcSoftGoal(-1000);
    CHECK(c.IsProtocolPositionPlanComplete());
    c.StopAndReset();CHECK(!c.IsProtocolPositionPlanComplete() && c.GetTargetVelocitySteps()==0 && c.GetCurrentCommandMa()==0);
    std::puts("PASS: real Motor controller target limits, offset overflow, planner completion and StopAndReset");
}

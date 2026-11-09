#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <initializer_list>
#include "motion_planner.h"
#define CHECK(c) do{if(!(c)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);std::exit(1);}}while(0)
int main(){
    unsigned cases=0;
    for(int acceleration:{1000,10000,100000})for(int distance:{1,2,10,100,1000})for(int sign:{-1,1}){
        MotionPlanner p;MotionPlanner::Config_t config={};
        config.ratedVelocity=500;config.ratedVelocityAcc=acceleration;p.AttachConfig(&config);
        p.positionTracker.NewTask(0,0);
        bool done=false;int32_t lastVelocity=0;
        for(unsigned tick=0;tick<200000;++tick){
            p.positionTracker.CalcSoftGoal(sign*distance);
            const int32_t v=p.positionTracker.go_velocity;
            CHECK(std::abs(v)<=500);
            // Integer velocity quantization allows ceil(a/f) changes per tick.
            CHECK(std::abs(v-lastVelocity)<=(acceleration+19999)/20000);lastVelocity=v;
            if(p.positionTracker.go_location==sign*distance && v==0){done=true;break;}
        }
        if(!done)std::fprintf(stderr,"unsettled distance=%d acceleration=%d pos=%d vel=%d\n",sign*distance,acceleration,p.positionTracker.go_location,p.positionTracker.go_velocity);
        CHECK(done);
        for(unsigned i=0;i<1000;++i){p.positionTracker.CalcSoftGoal(sign*distance);CHECK(p.positionTracker.go_location==sign*distance && p.positionTracker.go_velocity==0);}
        ++cases;
    }
    std::printf("PASS: %u short-move direction/acceleration cases converge and remain stopped\n",cases);
}

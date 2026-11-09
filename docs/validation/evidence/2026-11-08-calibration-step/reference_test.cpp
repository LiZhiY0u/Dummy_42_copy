#include <cstdio>
#include <cstdlib>
#include "encoder_calibrator_base.h"
#include "mt6816_base.h"
#include "tb67h450_base.h"
#include "stockpile_f103cb.h"
#define CHECK(c) do {if(!(c)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);std::exit(1);}}while(0)
Motor motor;
static uint16_t table[16384];
MT6816Base mt6816_base(table);
TB67H450Base tb67h450_base;
Stockpile_FLASH_Typedef stockpile_quick_cali;
unsigned flashCalls=0,writes=0,ends=0,sleeps=0,mode=0,erases=0,services=0;
void (*flashHook)(unsigned)=nullptr;
static EncoderCalibratorBase cal;
bool MT6816Base::UpdateAngle(){return true;}
void TB67H450Base::Sleep(){++sleeps;}
void TB67H450Base::SetFocCurrentVector(uint32_t,int32_t){CHECK(false);}
static void hook(unsigned operation){
    CHECK(sleeps>0);
    if(operation==0)++erases;
    if(operation==2)++ends;
    if(operation==3){++writes;if(mode==1 && writes==10)stockpile_quick_cali.asce_write_add=stockpile_quick_cali.begin_add+sizeof table;}
    if(operation==4 && mode==2 && writes==10)table[9]^=1;
    if(operation==4 && mode==3 && writes==10){cal.isTriggered=false;cal.state=EncoderCalibratorBase::CALI_DISABLE;}
    if(operation==2 && mode==4 && writes==16384){cal.isTriggered=false;cal.state=EncoderCalibratorBase::CALI_DISABLE;}
    if(operation==2 && mode==5 && writes==16384)table[0]^=1; // late corruption after per-word checks
    if(operation==0 && mode==6)table[0]=123; // erase failure
    if(operation==0 && mode==7){cal.isTriggered=false;cal.state=EncoderCalibratorBase::CALI_DISABLE;}
}
static void reset(unsigned failure){
    cal=EncoderCalibratorBase{};cal.isTriggered=true;cal.state=EncoderCalibratorBase::CALI_CALCULATING;
    for(unsigned i=0;i<=200;++i)cal.sampleDataAverageForward[i]=cal.sampleDataAverageBackward[i]=uint16_t((i*16384/200)%16384);
    for(auto &value:table)value=123;
    stockpile_quick_cali.begin_add=reinterpret_cast<uintptr_t>(table);
    stockpile_quick_cali.area_size=sizeof table;
    stockpile_quick_cali.page_num=32;
    stockpile_quick_cali.asce_write_add=0;
    mode=failure;writes=ends=sleeps=flashCalls=erases=services=0;flashHook=hook;
    mt6816_base.angleData.rectifyValid=false;
}
static void runSteps(){
    do {
        const unsigned oldWrites=writes,oldErases=erases;
        ++services; // opportunity for the outer loop to service UART, not real UART execution
        if(mode==8 && writes>=32){cal.isTriggered=false;cal.state=EncoderCalibratorBase::CALI_DISABLE;}
        cal.TickMainLoop();
        CHECK(writes-oldWrites<=32 && erases-oldErases<=1);
        CHECK(services<5000);
    } while(cal.isTriggered);
    const unsigned calls=flashCalls;
    cal.TickMainLoop();CHECK(flashCalls==calls);
}
static uint32_t hashTable(){ uint32_t h=2166136261U;for(auto value:table){h=(h^uint8_t(value))*16777619U;h=(h^uint8_t(value>>8))*16777619U;}return h; }
int main(){
reset(0);cal.TickMainLoop();CHECK(mt6816_base.angleData.rectifyValid);std::printf("REFERENCE forward=%08x\n",unsigned(hashTable()));
reset(0);for(unsigned i=0;i<=200;++i)cal.sampleDataAverageForward[i]=cal.sampleDataAverageBackward[i]=uint16_t((16384-i*16384/200)%16384);
cal.TickMainLoop();CHECK(mt6816_base.angleData.rectifyValid);std::printf("REFERENCE reverse=%08x\n",unsigned(hashTable()));
}

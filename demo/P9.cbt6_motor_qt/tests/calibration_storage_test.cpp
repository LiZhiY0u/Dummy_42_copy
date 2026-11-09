#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "encoder_calibrator_base.h"
#include "mt6816_base.h"
#include "tb67h450_base.h"
#include "stockpile_f103cb.h"
#include "calibration_record.h"
#include "calibration_lifecycle.h"
#define CHECK(c) do {if(!(c)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);std::exit(1);}}while(0)
Motor motor;
static uint16_t table[16384];
MT6816Base mt6816_base(table);
TB67H450Base tb67h450_base;
Stockpile_FLASH_Typedef stockpile_quick_cali;
Stockpile_FLASH_Typedef *activeFlashRegion=nullptr;
uint16_t storageMetadata[512];
uint16_t storageFlashKib=128;
bool metadataFlashActive=false;
unsigned flashCalls=0,writes=0,ends=0,sleeps=0,mode=0,erases=0,services=0,metaWrites=0,metaEnds=0;
void (*flashHook)(unsigned)=nullptr;
static EncoderCalibratorBase cal;
static uint32_t generation=0;
bool MT6816Base::UpdateAngle(){return true;}
void TB67H450Base::Sleep(){++sleeps;}
void TB67H450Base::SetFocCurrentVector(uint32_t,int32_t){CHECK(false);}
static void hook(unsigned operation){
    CHECK(sleeps>0);
    if(operation==0)++erases;
    if(metadataFlashActive){
        if(operation==2)++metaEnds;
        if(operation==3){++metaWrites;
            if((mode==10 && metaWrites==1)||(mode==13 && metaWrites==10))activeFlashRegion->asce_write_add=0;
        }
        if(operation==4 && mode==12 && metaWrites==9){cal.isTriggered=false;cal.state=EncoderCalibratorBase::CALI_DISABLE;}
        if(operation==4 && mode==14 && metaWrites==10){cal.isTriggered=false;cal.state=EncoderCalibratorBase::CALI_DISABLE;}
        if(operation==2 && mode==11 && metaWrites==8)storageMetadata[0]^=1;
        if(operation==0 && mode==7){cal.isTriggered=false;cal.state=EncoderCalibratorBase::CALI_DISABLE;}
        return;
    }
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
    CHECK(stepper::calibration::beginTask(++generation));
    CHECK(stepper::calibration::outcome(generation)==stepper::calibration::Outcome::Pending);
    CHECK(stepper::calibration::outcome(generation-1)==stepper::calibration::Outcome::Unknown);
    cal=EncoderCalibratorBase{};cal.isTriggered=true;cal.state=EncoderCalibratorBase::CALI_CALCULATING;
    for(unsigned i=0;i<=200;++i)cal.sampleDataAverageForward[i]=cal.sampleDataAverageBackward[i]=uint16_t((i*16384/200)%16384);
    for(auto &value:table)value=123;
    stockpile_quick_cali.begin_add=reinterpret_cast<uintptr_t>(table);
    stockpile_quick_cali.area_size=sizeof table;
    stockpile_quick_cali.page_num=32;
    stockpile_quick_cali.asce_write_add=0;
    for(auto &value:storageMetadata)value=0xffff;
    mode=failure;writes=ends=sleeps=flashCalls=erases=services=metaWrites=metaEnds=0;flashHook=hook;
    mt6816_base.angleData.rectifyValid=false;
}
static void runSteps(){
    do {
        const unsigned oldWrites=writes,oldErases=erases;
        ++services; // opportunity for the outer loop to service UART, not real UART execution
        if(mode==8 && writes>=32){cal.isTriggered=false;cal.state=EncoderCalibratorBase::CALI_DISABLE;}
        if(mode==9 && writes==16384){cal.isTriggered=false;cal.state=EncoderCalibratorBase::CALI_DISABLE;}
        cal.TickMainLoop();
        if(metaWrites<10)CHECK(!stepper::calibration::committed(stepper::calibration::stored()));
        CHECK(writes-oldWrites<=32 && erases-oldErases<=1);
        CHECK(services<5000);
    } while(cal.isTriggered);
    const unsigned calls=flashCalls;
    cal.TickMainLoop();CHECK(flashCalls==calls);
}
static uint32_t tableHash(){
    uint32_t hash=2166136261U;
    for(auto value:table){hash=(hash^uint8_t(value))*16777619U;hash=(hash^uint8_t(value>>8))*16777619U;}
    return hash;
}
int main(){
    reset(0);storageFlashKib=64;runSteps();CHECK(flashCalls==0);
    storageFlashKib=128;
    reset(0);
    const stepper::calibration::Record previous={stepper::calibration::Magic,1,32768,
        stepper::calibration::tableCrc(table),stepper::calibration::Commit};
    std::memcpy(storageMetadata,&previous,sizeof previous);
    CHECK(stepper::calibration::valid(stepper::calibration::stored(),table));
    CHECK(stepper::calibration::outcome(generation)==stepper::calibration::Outcome::Pending);
    cal.TickMainLoop(); // analysis alone preserves the old committed table
    CHECK(stepper::calibration::valid(stepper::calibration::stored(),table));
    cal.TickMainLoop(); // invalidate old metadata before any table erase/write
    CHECK(!stepper::calibration::committed(stepper::calibration::stored()) && writes==0);
    runSteps();CHECK(mt6816_base.angleData.rectifyValid);
    reset(0);runSteps();
    CHECK(writes==16384 && ends==512 && erases==33 && metaEnds==2 && services>500 && mt6816_base.angleData.rectifyValid);
    CHECK(stepper::calibration::outcome(generation)==stepper::calibration::Outcome::Verified);
    CHECK(stepper::calibration::valid(stepper::calibration::stored(),table));
    CHECK(tableHash()==0x479c6945U); // captured from the pre-step real algorithm
    for(auto value:table)CHECK(value<51200);
    for(unsigned failure=1;failure<=13;++failure){
        reset(failure);runSteps();
        CHECK(!mt6816_base.angleData.rectifyValid);
        CHECK(!cal.isTriggered && cal.state==EncoderCalibratorBase::CALI_DISABLE);
        CHECK(ends==(failure>=6 && failure<=7?0U:(failure>=4 && failure<=5)||failure>=9?512U:1U));
        CHECK(!stepper::calibration::valid(stepper::calibration::stored(),table));
        if(failure<=3)CHECK(writes==10);
        if(failure>=6 && failure<=7)CHECK(writes==0);
        if(failure==8)CHECK(writes==32);
        if(failure==9)CHECK(writes==16384);
    }
    reset(0);
    for(unsigned i=0;i<=200;++i)cal.sampleDataAverageForward[i]=cal.sampleDataAverageBackward[i]=uint16_t((16384-i*16384/200)%16384);
    runSteps();CHECK(writes==16384 && mt6816_base.angleData.rectifyValid);
    CHECK(tableHash()==0xb04ba6d5U);
    reset(14);runSteps();
    CHECK(mt6816_base.angleData.rectifyValid); // durable commit won before late cancel
    CHECK(stepper::calibration::valid(stepper::calibration::stored(),table));
    CHECK(stepper::calibration::outcome(generation)==stepper::calibration::Outcome::Verified);
    reset(0);
    stepper::calibration::requestCancel(generation);
    cal.isTriggered=false;cal.state=EncoderCalibratorBase::CALI_DISABLE;
    cal.TickMainLoop();
    CHECK(stepper::calibration::outcome(generation)==stepper::calibration::Outcome::Cancelled);
    // The full commit was already durable before this late cancellation.
    std::puts("PASS: real calibration main-loop full table, write/readback failure, mid-write/final cancel and relock");
}

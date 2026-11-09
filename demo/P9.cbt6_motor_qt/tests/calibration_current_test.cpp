#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include "encoder_calibrator_base.h"
#include "mt6816_base.h"
#include "tb67h450_base.h"
#include "stockpile_f103cb.h"
#define CHECK(c) do { if(!(c)) { std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c); std::exit(1); } } while(0)
Motor motor;
static uint16_t table[16384];
MT6816Base mt6816_base(table);
TB67H450Base tb67h450_base;
Stockpile_FLASH_Typedef stockpile_quick_cali;
Stockpile_FLASH_Typedef *activeFlashRegion=nullptr;
uint16_t storageMetadata[512];
uint16_t storageFlashKib=128;
bool metadataFlashActive=false;
void (*flashHook)(unsigned)=nullptr;
unsigned flashCalls=0, outputs=0, sleeps=0;
int32_t lastCurrent=0;
bool sensorValid=true;
bool MT6816Base::UpdateAngle() { angleData.sampleValid=sensorValid; return sensorValid; }
void TB67H450Base::Sleep() { ++sleeps; }
void TB67H450Base::SetFocCurrentVector(uint32_t, int32_t value) { ++outputs; lastCurrent=value; }
static EncoderCalibratorBase calibrator;
static void reset(int32_t current, EncoderCalibratorBase::State_t state) {
    calibrator=EncoderCalibratorBase{};
    calibrator.isTriggered=true;calibrator.state=state;calibrator.goPosition=51201;
    motor.config.motionParams.caliCurrent=current;
    outputs=sleeps=flashCalls=0;sensorValid=true;
}
int main() {
    const EncoderCalibratorBase::State_t states[]={EncoderCalibratorBase::CALI_DISABLE,
        EncoderCalibratorBase::CALI_FORWARD_PREPARE,EncoderCalibratorBase::CALI_FORWARD_MEASURE,
        EncoderCalibratorBase::CALI_BACKWARD_RETURN,EncoderCalibratorBase::CALI_BACKWARD_GAP_DISMISS,
        EncoderCalibratorBase::CALI_BACKWARD_MEASURE};
    for(auto state:states) for(int32_t current:{int32_t(250),int32_t(500),int32_t(1000)}) {
        reset(current,state);calibrator.Tick20kHz();
        CHECK(outputs==1 && lastCurrent==current && flashCalls==0);
    }
    for(int32_t current:{int32_t(-1),int32_t(0),int32_t(1001),int32_t(3000)}) {
        reset(current,EncoderCalibratorBase::CALI_FORWARD_PREPARE);calibrator.Tick20kHz();
        CHECK(outputs==0 && sleeps>0 && !calibrator.isTriggered);
        CHECK(calibrator.state==EncoderCalibratorBase::CALI_DISABLE && flashCalls==0);
    }
    reset(500,EncoderCalibratorBase::CALI_FORWARD_PREPARE);calibrator.isTriggered=false;
    calibrator.Tick20kHz();CHECK(outputs==0 && sleeps>0);
    reset(500,EncoderCalibratorBase::CALI_FORWARD_PREPARE);sensorValid=false;
    calibrator.Tick20kHz();CHECK(outputs==0 && sleeps>0 && !calibrator.isTriggered);
    reset(500,EncoderCalibratorBase::CALI_CALCULATING);calibrator.Tick20kHz();
    CHECK(outputs==0 && sleeps>0 && flashCalls==0);
    std::puts("PASS: real calibration tick current in six phases, invalid/off/fault paths, no Flash in ISR");
}

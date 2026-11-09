#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "../Uart/uart_protocol.cpp"
#include "tb67h450_base.h"
#include "calibration_record.h"
#define CHECK(c) do{if(!(c)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);std::exit(1);}}while(0)
UART_HandleTypeDef huart1={};
IntegrationController controller;
Motor motor(&controller);
static uint16_t table[16384];
MT6816Base mt6816_base(table);
EncoderCalibratorBase encoder_calibrator_base;
TB67H450Base tb67h450_base;
FakeConfig boardConfig={1000,1000,1536000,1000000};
Stockpile_FLASH_Typedef stockpile_quick_cali;
Stockpile_FLASH_Typedef *activeFlashRegion=nullptr;
uint16_t storageMetadata[512],storageFlashKib=128;
bool metadataFlashActive=false;
unsigned flashCalls=0,writes=0,metaWrites=0;
void (*flashHook)(unsigned)=nullptr;
static unsigned scenario=0;
static uint32_t tick=0;
static std::vector<stepper::Frame> replies;
uint32_t HAL_GetTick(){return tick;}
bool MT6816Base::UpdateAngle(){return angleData.sampleValid;}
bool MT6816Base::IsCalibrated(){return angleData.rectifyValid;}
uint16_t MT6816Base::GetConsecutiveErrorCount()const{return 0;}
void TB67H450Base::Sleep(){}
void TB67H450Base::SetFocCurrentVector(uint32_t,int32_t){CHECK(false);}
static void receive(void *,const stepper::Frame &f){replies.push_back(f);}
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *bytes,uint16_t size){
    stepper::Parser p;p.feed(bytes,size,tick,receive,nullptr);return HAL_OK;
}
static void drain(){for(unsigned i=0;i<8;++i){transmitNext();UartProtocolOnTxComplete();}}
static void hook(unsigned op){
    if(op==3){if(metadataFlashActive)++metaWrites;else ++writes;}
    if(scenario==1 && !metadataFlashActive && op==4 && writes==10)table[9]^=1;
}
static uint16_t seq=1;
static void request(uint16_t cmd,uint32_t value=0){
    stepper::Frame f={};f.type=1;f.command=cmd;f.sequence=seq++;
    f.session=cmd==1?0:0x12345678;
    if(cmd==1 || cmd==0x0302 || cmd==0x0303){f.length=4;write32(f.payload,value);}
    uint8_t wire[144];const size_t n=stepper::encode(f,wire,sizeof(wire));CHECK(n);
    UartProtocolOnRx(wire,uint16_t(n));UartProtocolPoll(tick);drain();
}
int main(int argc,char **argv){
    scenario=argc>1?unsigned(std::atoi(argv[1])):0;CHECK(scenario<=3);
    for(auto &v:table)v=123;
    for(auto &v:storageMetadata)v=0xffff;
    stockpile_quick_cali.begin_add=reinterpret_cast<uintptr_t>(table);
    stockpile_quick_cali.area_size=sizeof table;stockpile_quick_cali.page_num=32;
    flashHook=hook;mt6816_base.angleData.sampleValid=true;
    UartProtocolInit();request(1,0x12345678);UartProtocolControlTick(tick);UartProtocolPoll(tick);drain();
    replies.clear();request(0x0301);
    CHECK(replies.size()==1 && read16(replies[0].payload)==1);
    const uint32_t id=read32(replies[0].payload+2);CHECK(id);
    UartProtocolControlTick(tick);CHECK(encoder_calibrator_base.isTriggered);
    // Synthetic collected samples; real analysis/writer/lifecycle thereafter.
    for(unsigned i=0;i<=200;++i)
        encoder_calibrator_base.sampleDataAverageForward[i]=encoder_calibrator_base.sampleDataAverageBackward[i]=uint16_t((i*16384/200)%16384);
    encoder_calibrator_base.state=EncoderCalibratorBase::CALI_CALCULATING;
    bool cancelled=false;
    for(unsigned step=0;step<2000 && currentCalibrationTask;++step){
        tick=step/10;
        encoder_calibrator_base.TickMainLoop();
        if(!cancelled && ((scenario==2 && writes>=32) || (scenario==3 && metaWrites==10))){
            request(0x0303,id);cancelled=true;
        }
        UartProtocolControlTick(tick);UartProtocolPoll(tick);drain();
    }
    CHECK(currentCalibrationTask==0 && controller.enables==0);
    bool event=false;
    const unsigned expected=scenario==1?5:scenario==2?4:6;
    for(const auto &f:replies)if(f.type==3 && f.command==0x0501 && read32(f.payload)==id && f.payload[4]==expected)event=true;
    CHECK(event);
    replies.clear();request(0x0302,id);
    CHECK(replies.size()==1 && replies[0].length==15 && read16(replies[0].payload)==0);
    const bool valid=scenario==0 || scenario==3;
    CHECK(replies[0].payload[6]==(valid?5:6));
    CHECK(replies[0].payload[10]==unsigned(valid));
    CHECK(stepper::calibration::valid(stepper::calibration::stored(),table)==valid);
    CHECK(mt6816_base.IsCalibrated()==valid);
    CHECK(calibrationFaults==(scenario==1?40U:0U));
    CHECK(!encoder_calibrator_base.isTriggered);
    std::printf("PASS: real calibration writer to UART query/event scenario=%u writes=%u metadata=%u\n",scenario,writes,metaWrites);
}

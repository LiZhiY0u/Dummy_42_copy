#include "command_dispatcher.h"
#include "command_contract.h"
#include <string.h>
namespace stepper {
namespace {
uint32_t read32(const uint8_t *p){return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
void result(Frame &response,uint16_t status){response.length=2;response.payload[0]=uint8_t(status);response.payload[1]=uint8_t(status>>8);}
}
CommandDispatcher::CommandDispatcher(CommandBackend &backend):backend_(backend) {reset();}
void CommandDispatcher::reset() {
    for(unsigned i=0;i<4;++i)entries_[i].valid=false;
    cachedSession_=0;highWater_=0;next_=0;
}
int CommandDispatcher::find(const Frame &request,uint32_t now) const {
    for(unsigned i=0;i<4;++i) {
        const Entry &entry=entries_[i];
        if(entry.valid&&entry.request.sequence==request.sequence&&uint32_t(now-entry.timestamp)<2000) {
            const Frame &old=entry.request;
            if(old.session==request.session&&old.command==request.command&&old.length==request.length&&request.length<=128
               &&memcmp(old.payload,request.payload,request.length)==0)return int(i);
            return -2;
        }
    }
    return -1;
}
bool CommandDispatcher::forward(uint16_t sequence) const {
    const uint16_t delta=uint16_t(sequence-highWater_);
    return !highWater_||(delta>=1&&delta<=32767);
}
void CommandDispatcher::remember(const Frame &request,const Frame &response,uint32_t now) {
    Entry &entry=entries_[next_];entry.request=request;entry.response=response;entry.timestamp=now;entry.valid=true;
    next_=uint8_t((next_+1)%4);highWater_=request.sequence;
}
bool CommandDispatcher::handle(const Frame &request,uint32_t now,Frame &response) {
    if(request.type!=1||!request.sequence||request.length>128)return false;
    backend_.poll(now);
    const uint32_t active=backend_.sessionId();
    if(active!=cachedSession_){reset();cachedSession_=active;}
    response=Frame();response.type=2;response.command=request.command;response.sequence=request.sequence;response.session=request.session;
    if(request.command==1) {
        response.session=0;
        if(request.session!=0){result(response,11);return true;}
        const uint32_t token=request.length>=4?read32(request.payload):0;
        const bool newSession=request.length==4&&token!=0&&token!=active;
        if(active&&!newSession) {
            const int cached=find(request,now);
            if(cached>=0){response=entries_[cached].response;return true;}
            if(cached==-2||!forward(request.sequence)){result(response,12);return true;}
        }
        const uint16_t checked=contract::validatePayload(request.command,request.payload,request.length);
        if(checked){result(response,checked);if(active&&!newSession)remember(request,response,now);return true;}
        const uint16_t status=backend_.beginSession(token,now);
        if(status!=0){result(response,status<=13?status:13);return true;}
        if(backend_.sessionId()!=token){result(response,13);return true;}
        if(token!=cachedSession_){reset();cachedSession_=token;}
        response.session=token;result(response,0);response.length=4;response.payload[2]=1;response.payload[3]=0;
        remember(request,response,now);return true;
    }
    if(!active||request.session!=active){result(response,11);return true;}
    const int cached=find(request,now);
    if(cached>=0){response=entries_[cached].response;return true;}
    if(cached==-2||!forward(request.sequence)){result(response,12);return true;}
    uint16_t status=contract::validatePayload(request.command,request.payload,request.length);
    if(!status) {
        Frame data={};status=backend_.execute(request,now,data);
        if(status>13||data.length>126)status=13;
        if(status<=1){result(response,status);memcpy(response.payload+2,data.payload,data.length);response.length=uint16_t(2+data.length);}
        else result(response,status);
    } else result(response,status);
    // Even rejected new requests consume their sequence. Retry cannot change data.
    remember(request,response,now);return true;
}
}

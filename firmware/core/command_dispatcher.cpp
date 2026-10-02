#include "command_dispatcher.h"
#include <string.h>
namespace stepper {
namespace {
uint16_t read16(const uint8_t *p){return uint16_t(uint16_t(p[0])|(uint16_t(p[1])<<8));}
uint32_t read32(const uint8_t *p){return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
bool positive(uint32_t n){return n>0&&n<0x80000000u;}
void result(Frame &response,uint16_t status){response.length=2;response.payload[0]=uint8_t(status);response.payload[1]=uint8_t(status>>8);}
uint16_t parameters(const Frame &request) {
    if(request.length<4||read16(request.payload)!=1)return 3;
    const uint16_t count=read16(request.payload+2);
    if(!count||count>15)return 3;
    uint16_t ids[15];size_t offset=4;
    for(uint16_t index=0;index<count;++index) {
        if(offset+4>request.length)return 3;
        const uint16_t id=read16(request.payload+offset);const uint8_t type=request.payload[offset+2],length=request.payload[offset+3];
        for(uint16_t prior=0;prior<index;++prior)if(ids[prior]==id)return 3;
        ids[index]=id;uint8_t expected=0;
        if((id>=1&&id<=4)||(id>=0x0401&&id<=0x0403))expected=2;
        else if((id>=0x0101&&id<=0x0103)||(id>=0x0201&&id<=0x0204))expected=1;
        else if(id==0x0301)expected=3;
        else return 3;
        if(type!=expected||length!=(type==3?1:4)||offset+4+length>request.length)return 3;
        offset+=4;
        if(type==3){if(request.payload[offset]>1)return 4;}
        else {
            const uint32_t value=read32(request.payload+offset);
            if(type==1&&value>(id<=0x0103?255u:4095u))return 4;
            if(type==2) {
                if(id<=4&&!positive(value))return 4;
                if(id==0x0401&&(!value||value>51200))return 4;
                if(id==0x0402&&value>51200)return 4;
                if(id==0x0403&&(value<10||value>2000))return 4;
            }
        }
        offset+=length;
    }
    return offset==request.length?0:3;
}
uint16_t validate(const Frame &r) {
    if(r.length>128)return 3;
    switch(r.command) {
    case 1:return r.length==4&&read32(r.payload)!=0?0:3;
    case 2:case 3:case 4:case 0x0102:case 0x0106:case 0x0107:case 0x0108:case 0x0201:case 0x0301:return r.length==0?0:3;
    case 0x0101:if(r.length!=1)return 3;return r.payload[0]<=2?0:4;
    case 0x0103:if(r.length!=12)return 3;return positive(read32(r.payload+4))&&positive(read32(r.payload+8))?0:4;
    case 0x0104:if(r.length!=8)return 3;return read32(r.payload)!=0x80000000u&&positive(read32(r.payload+4))?0:4;
    case 0x0105:if(r.length!=4)return 3;return read32(r.payload)!=0x80000000u?0:4;
    case 0x0203:return r.length==4?0:3;
    case 5:case 0x0302:case 0x0303:if(r.length!=4)return 3;return read32(r.payload)!=0?0:4;
    case 0x0202:return parameters(r);
    case 0x0401: {
        if(r.length!=2)return 3;
        const uint16_t period=read16(r.payload);
        return period==0||period==10||period==20||period==50||period==100?0:4;
    }
    default:return 2;
    }
}
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
        const uint16_t checked=validate(request);
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
    uint16_t status=validate(request);
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

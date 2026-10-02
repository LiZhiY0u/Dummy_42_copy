#include "protocol_v1.h"
#include <string.h>
namespace stepper {
namespace {
uint16_t read16(const uint8_t *p) {return uint16_t(uint16_t(p[0])|(uint16_t(p[1])<<8));}
uint32_t read32(const uint8_t *p) {return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
void write16(uint8_t *p,uint16_t value) {p[0]=uint8_t(value);p[1]=uint8_t(value>>8);}
void write32(uint8_t *p,uint32_t value) {for(unsigned i=0;i<4;++i)p[i]=uint8_t(value>>(8*i));}
bool validType(uint8_t type) {return type>=1&&type<=4;}
}
uint16_t crc16(const uint8_t *bytes,size_t length) {
    uint16_t crc=0xffff;
    for(size_t i=0;i<length;++i) {
        crc^=uint16_t(bytes[i])<<8;
        for(unsigned bit=0;bit<8;++bit)crc=uint16_t((crc&0x8000)?(crc<<1)^0x1021:crc<<1);
    }
    return crc;
}
size_t encode(const Frame &frame,uint8_t *output,size_t capacity) {
    const size_t total=16+size_t(frame.length);
    if(!output||frame.length>128||!validType(frame.type)||capacity<total)return 0;
    output[0]=0xaa;output[1]=0x55;output[2]=1;output[3]=frame.type;
    write16(output+4,frame.length);write32(output+6,frame.session);
    write16(output+10,frame.sequence);write16(output+12,frame.command);
    memcpy(output+14,frame.payload,frame.length);write16(output+total-2,crc16(output+2,total-4));
    return total;
}
void Parser::reset() {count_=0;candidate_=false;}
void Parser::discard(size_t length) {
    if(length<count_)memmove(buffer_,buffer_+length,count_-length);
    count_-=length;candidate_=false;
}
void Parser::feed(const uint8_t *bytes,size_t length,uint32_t nowMs,Receiver receiver,void *context) {
    if(candidate_&&uint32_t(nowMs-candidateSince_)>=100)reset();
    if(!bytes&&length)return;
    for(size_t i=0;i<length;++i) {
        if(count_==sizeof(buffer_))reset();
        buffer_[count_++]=bytes[i];parse(nowMs,receiver,context);
    }
}
void Parser::parse(uint32_t nowMs,Receiver receiver,void *context) {
    while(count_) {
        if(buffer_[0]!=0xaa){discard(1);continue;}
        if(!candidate_){candidate_=true;candidateSince_=nowMs;}
        if(count_<2)return;
        if(buffer_[1]!=0x55){discard(1);continue;}
        if(count_<14)return;
        const uint16_t length=read16(buffer_+4);
        if(buffer_[2]!=1||!validType(buffer_[3])||length>128){discard(1);continue;}
        const size_t total=16+size_t(length);
        if(count_<total)return;
        if(crc16(buffer_+2,total-4)!=read16(buffer_+total-2)){discard(1);continue;}
        Frame frame={};frame.type=buffer_[3];frame.length=length;
        frame.session=read32(buffer_+6);frame.sequence=read16(buffer_+10);frame.command=read16(buffer_+12);
        memcpy(frame.payload,buffer_+14,length);discard(total);
        if(receiver)receiver(context,frame);
    }
}
}

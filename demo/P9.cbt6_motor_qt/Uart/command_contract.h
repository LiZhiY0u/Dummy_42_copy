#ifndef STEPPER_COMMAND_CONTRACT_H
#define STEPPER_COMMAND_CONTRACT_H
#include <stdint.h>
#include <stddef.h>

// Canonical source: Qt workspace firmware/core/command_contract.h.
// P9 distributes an identical copy; validate it before cross-repository delivery.
namespace stepper { namespace contract {
enum class Rule : uint8_t {Empty,Hello,Enable,MoveAbsolute,Velocity,Current,TaskId,Parameters,Save,Telemetry};
enum class Group : uint8_t {Core,Motion,Parameters,Calibration,Telemetry};
struct Description {uint16_t command;Rule rule;Group group;uint8_t ordinal;};
struct Support {uint32_t commandBits;};

inline const Description *describe(uint16_t command) {
    static const Description descriptions[]={
        {1,Rule::Hello,Group::Core,0},{2,Rule::Empty,Group::Core,1},
        {3,Rule::Empty,Group::Core,2},{4,Rule::Empty,Group::Core,3},
        {5,Rule::TaskId,Group::Core,4},{0x0101,Rule::Enable,Group::Motion,5},
        {0x0102,Rule::Empty,Group::Motion,6},{0x0103,Rule::MoveAbsolute,Group::Motion,7},
        {0x0104,Rule::Velocity,Group::Motion,8},{0x0105,Rule::Current,Group::Motion,9},
        {0x0106,Rule::Empty,Group::Motion,10},{0x0107,Rule::Empty,Group::Motion,11},
        {0x0108,Rule::Empty,Group::Motion,12},{0x0201,Rule::Empty,Group::Parameters,13},
        {0x0202,Rule::Parameters,Group::Parameters,14},{0x0203,Rule::Save,Group::Parameters,15},
        {0x0301,Rule::Empty,Group::Calibration,16},{0x0302,Rule::TaskId,Group::Calibration,17},
        {0x0303,Rule::TaskId,Group::Calibration,18},{0x0401,Rule::Telemetry,Group::Telemetry,19}
    };
    for(unsigned i=0;i<20;++i)if(descriptions[i].command==command)return &descriptions[i];
    return nullptr;
}
inline bool supports(Support support,uint16_t command) {
    const Description *d=describe(command);
    return d&&(support.commandBits&(uint32_t(1)<<d->ordinal))!=0;
}
namespace detail {
inline uint16_t read16(const uint8_t *p) {return uint16_t(uint16_t(p[0])|(uint16_t(p[1])<<8));}
inline uint32_t read32(const uint8_t *p) {return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
inline bool positive(uint32_t n) {return n>0&&n<0x80000000u;}
inline uint16_t parameters(const uint8_t *payload,size_t length) {
    if(length<4||read16(payload)!=1)return 3;
    const uint16_t count=read16(payload+2);
    if(!count||count>15)return 3;
    uint16_t ids[15];size_t offset=4;
    for(uint16_t index=0;index<count;++index) {
        if(offset+4>length)return 3;
        const uint16_t id=read16(payload+offset);
        const uint8_t type=payload[offset+2],size=payload[offset+3];
        for(uint16_t prior=0;prior<index;++prior)if(ids[prior]==id)return 3;
        ids[index]=id;uint8_t expected=0;
        if((id>=1&&id<=4)||(id>=0x0401&&id<=0x0403))expected=2;
        else if((id>=0x0101&&id<=0x0103)||(id>=0x0201&&id<=0x0204))expected=1;
        else if(id==0x0301)expected=3;
        else return 3;
        if(type!=expected||size!=(type==3?1:4)||offset+4+size>length)return 3;
        offset+=4;
        if(type==3) {if(payload[offset]>1)return 4;}
        else {
            const uint32_t value=read32(payload+offset);
            if(type==1&&value>(id<=0x0103?255u:4095u))return 4;
            if(type==2) {
                if(id<=4&&!positive(value))return 4;
                if(id==0x0401&&(!value||value>51200))return 4;
                if(id==0x0402&&value>51200)return 4;
                if(id==0x0403&&(value<10||value>2000))return 4;
            }
        }
        offset+=size;
    }
    return offset==length?0:3;
}
}
// Payload only: OK never means that a backend operation was executed.
// Session, replay, ownership and state validation belong to the caller.
inline uint16_t validatePayload(uint16_t command,const uint8_t *payload,size_t length) {
    if(length>128||(length&&!payload))return 3;
    const Description *d=describe(command);
    if(!d)return 2;
    using namespace detail;
    switch(d->rule) {
    case Rule::Empty:return length==0?0:3;
    case Rule::Hello:return length==4&&read32(payload)!=0?0:3;
    case Rule::Enable:if(length!=1)return 3;return payload[0]<=2?0:4;
    case Rule::MoveAbsolute:if(length!=12)return 3;return positive(read32(payload+4))&&positive(read32(payload+8))?0:4;
    case Rule::Velocity:if(length!=8)return 3;return read32(payload)!=0x80000000u&&positive(read32(payload+4))?0:4;
    case Rule::Current:if(length!=4)return 3;return read32(payload)!=0x80000000u?0:4;
    case Rule::TaskId:if(length!=4)return 3;return read32(payload)!=0?0:4;
    case Rule::Parameters:return parameters(payload,length);
    case Rule::Save:return length==4?0:3;
    case Rule::Telemetry: {
        if(length!=2)return 3;
        const uint16_t period=read16(payload);
        return period==0||period==10||period==20||period==50||period==100?0:4;
    }
    }
    return 2;
}
// Composite capabilities require all backend operations, not just declarations.
inline uint32_t capabilities(Support support) {
    uint32_t bits=0;
    if(supports(support,0x0101)&&supports(support,0x0103))bits|=1;
    if(supports(support,0x0101)&&supports(support,0x0104))bits|=2;
    if(supports(support,0x0101)&&supports(support,0x0105))bits|=4;
    if(supports(support,0x0201)&&supports(support,0x0202))bits|=8;
    if(supports(support,0x0203)&&supports(support,5))bits|=16;
    if(supports(support,0x0301)&&supports(support,0x0302)&&supports(support,0x0303)&&supports(support,5))bits|=32;
    if(supports(support,0x0107))bits|=64;
    if(supports(support,0x0108))bits|=128;
    if(supports(support,0x0401))bits|=256;
    return bits;
}
} }
#endif

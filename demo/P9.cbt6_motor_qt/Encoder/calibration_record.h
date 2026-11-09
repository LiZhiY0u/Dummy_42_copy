#ifndef P9_CALIBRATION_RECORD_H
#define P9_CALIBRATION_RECORD_H
#include <stdint.h>
#include <stddef.h>
#ifndef P9_CALIBRATION_RECORD_ADDRESS
#define P9_CALIBRATION_RECORD_ADDRESS 0x08017800U
#endif
namespace stepper { namespace calibration {
const uint32_t Magic=0x314c4143U, Version=1, Bytes=32768U, Commit=0x54494d43U;
struct Record { uint32_t magic,version,bytes,crc,commit; };
static_assert(sizeof(Record)==20,"Calibration record wire layout");
inline uint32_t updateWord(uint32_t crc,uint16_t value) {
    for(unsigned byte=0;byte<2;++byte){
        crc^=uint8_t(value>>(8*byte));
        for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^(0xedb88320U & (0U-(crc&1U)));
    }
    return crc;
}
static inline bool committed(const volatile Record &record) {
    return record.commit==Commit && record.magic==Magic && record.version==Version && record.bytes==Bytes;
}
inline uint32_t tableCrc(const volatile uint16_t *table) {
    uint32_t crc=0xffffffffU;
    for(unsigned i=0;i<Bytes/2;++i)crc=updateWord(crc,table[i]);
    return ~crc;
}
inline bool valid(const volatile Record &record,const volatile uint16_t *table) {
    if(!table || !committed(record))return false;
    uint32_t crc=0xffffffffU;
    for(unsigned i=0;i<Bytes/2;++i){const uint16_t value=table[i];if(value>=51200U)return false;crc=updateWord(crc,value);}
    return ~crc==record.crc;
}
inline const volatile Record &stored() {
    return *reinterpret_cast<const volatile Record *>(uintptr_t(P9_CALIBRATION_RECORD_ADDRESS));
}
} }
#endif

#include "calibration_record.h"
#include <cstdio>
#include <cstdlib>
#define CHECK(c) do{if(!(c)){std::fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c);std::exit(1);}}while(0)
static uint16_t table[16384];
int main(){
    using namespace stepper::calibration;
    for(unsigned i=0;i<16384;++i)table[i]=uint16_t(i*3);
    Record record={Magic,Version,Bytes,tableCrc(table),0xffffffffU};
    CHECK(!valid(record,table));
    record.commit=uint16_t(Commit)|0xffff0000U;CHECK(!valid(record,table));
    record.commit=Commit;CHECK(valid(record,table));
    table[500]^=1;CHECK(!valid(record,table));table[500]^=1;
    record.version=2;CHECK(!valid(record,table));record.version=Version;
    record.bytes-=2;CHECK(!valid(record,table));record.bytes=Bytes;
    record.magic=0;CHECK(!valid(record,table));record.magic=Magic;
    table[42]=51200;record.crc=tableCrc(table);CHECK(!valid(record,table));
    std::puts("PASS: calibration record torn/uncommitted, CRC, schema, length and range guards");
}

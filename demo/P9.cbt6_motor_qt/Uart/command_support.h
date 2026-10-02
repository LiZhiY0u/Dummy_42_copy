#ifndef STEPPER_P9_COMMAND_SUPPORT_H
#define STEPPER_P9_COMMAND_SUPPORT_H
#include "command_contract.h"
namespace stepper { namespace p9 {
// A command belongs here only after its real backend and completion are wired.
// ENABLE alone must not advertise an unimplemented motion target capability.
inline contract::Support supportedCommands() {
    static const uint16_t commands[]={1,2,3,4,0x0101,0x0102,0x0106,0x0108,0x0401};
    contract::Support support={0};
    for(unsigned i=0;i<sizeof(commands)/sizeof(commands[0]);++i) {
        const contract::Description *d=contract::describe(commands[i]);
        if(d)support.commandBits|=uint32_t(1)<<d->ordinal;
    }
    return support;
}
} }
#endif

#ifndef STEPPER_P9_COMMAND_SUPPORT_H
#define STEPPER_P9_COMMAND_SUPPORT_H
#include "command_contract.h"
#include "configurations.h"
namespace stepper { namespace p9 {
// A command belongs here only after its real backend and completion are wired.
// ENABLE alone must not advertise an unimplemented motion target capability.
inline contract::Support supportedCommands() {
    static const uint16_t commands[]={1,2,3,4,5,0x0101,0x0102,0x0106,0x0108,0x0302,0x0401
#if !UART_COMM_ONLY_TEST && !UART_ECHO_TEST_MODE && !UART_TX_TEST_MODE
    ,0x0301,0x0303
#endif
    };
    contract::Support support={0};
    for(unsigned i=0;i<sizeof(commands)/sizeof(commands[0]);++i) {
        const contract::Description *d=contract::describe(commands[i]);
        if(d)support.commandBits|=uint32_t(1)<<d->ordinal;
    }
    return support;
}
} }
#endif

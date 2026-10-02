#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

// Current diagnostic: echo received bytes exactly, without protocol framing.
#ifndef UART_ECHO_TEST_MODE
#define UART_ECHO_TEST_MODE 0
#endif

// Temporary UART wiring diagnostic. Set to 0 and rebuild to restore protocol V1.
#ifndef UART_TX_TEST_MODE
#define UART_TX_TEST_MODE 0
#endif

// Temporary protocol diagnostic: no encoder/PWM/control timers, no motion.
// Set to 0 after serial handshake has been verified.
#ifndef UART_COMM_ONLY_TEST
#define UART_COMM_ONLY_TEST 0
#endif

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------- C Scope ---------------------------*/
#include <stdbool.h>
#include "stdint.h"

typedef enum configStatus_t
{
    CONFIG_RESTORE = 0,
    CONFIG_OK,
    CONFIG_COMMIT
} configStatus_t;


typedef struct Config_t
{
    configStatus_t configStatus;
    uint32_t canNodeId;
    int32_t encoderHomeOffset;
    uint32_t defaultMode;
    int32_t currentLimit;
    int32_t velocityLimit;// 速度限制
    int32_t velocityAcc;
    int32_t calibrationCurrent;
    int32_t dce_kp;
    int32_t dce_kv;
    int32_t dce_ki;
    int32_t dce_kd;
    bool enableMotorOnBoot;
    bool enableStallProtect;
} BoardConfig_t;

extern BoardConfig_t boardConfig;


#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/

// #include <Platform/Memory/eeprom_interface.h>
#include "motor.h"


#endif
#endif

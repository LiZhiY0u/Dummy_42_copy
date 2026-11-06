#pragma once
#include <cstdint>
inline void __DMB() {}
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __set_PRIMASK(uint32_t) {}
struct Motor {
    enum { MODE_STOP=0, MODE_COMMAND_POSITION=1, MODE_COMMAND_VELOCITY=2 };
    struct Controller { float velocity = 0; float GetVelocity() { return velocity; } } instance;
    Controller* controller = &instance;
};
extern Motor motor;
void OnCanCmd(uint8_t, uint8_t*, uint32_t);
extern "C" void UartProcess();
extern "C" void OnUartChunk(uint8_t*, uint16_t);

#include "common_inc.h"
#include "usart.h"
#include <cassert>
#include <cstring>
#include <cstdio>
Motor motor;
UART_HandleTypeDef huart1;
uint8_t rx_buffer[BUFFER_SIZE];
uint32_t ticks=0;
unsigned frames=0, sends=0, rearms=0;
uint8_t* dmaSource = nullptr;
float lastPosition=0;
void OnUartCmd(uint8_t*, uint16_t);
void OnCanCmd(uint8_t cmd, uint8_t* data, uint32_t len) {
    if (cmd == 0x05) { assert(len == 4); ++frames; std::memcpy(&lastPosition, data, 4); }
}
uint32_t HAL_GetTick() { return ticks; }
int HAL_UART_Receive_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t len) { assert(len==128); ++rearms; huart1.RxState=1; return 0; }
int HAL_UART_Transmit_DMA(UART_HandleTypeDef*, uint8_t* p, uint16_t len) { assert(len==5); dmaSource=p; ++sends; huart1.gState=1; return 0; }
int main() {
    uint8_t burst[130];
    for (unsigned i=0;i<26;++i) { uint8_t frame[]={0xaa,0x55,1,0x13,0xec}; std::memcpy(burst+i*5,frame,5); } // one turn
    std::memcpy(rx_buffer,burst,128);
    HAL_UART_RxCpltCallback(&huart1); // full DMA chunk must not create idle boundary
    assert(rearms==1);
    OnUartCmd(burst+128,2);
    UartProcess(); UartProcess();
    assert(frames==26 && lastPosition==1.0f);
    uint8_t telemetry[]={0xaa,0x55,0x22,1,0};
    OnUartCmd(telemetry,5); ticks=10; UartProcess();
    assert(sends==1 && dmaSource);
    uint8_t previous[5]; std::memcpy(previous,dmaSource,5);
    motor.controller->velocity=10;
    ticks=20; UartProcess();
    assert(sends==1 && std::memcmp(previous,dmaSource,5)==0);
    // Lost data: discard even plausible command prefixes until the next idle.
    OnUartChunk(burst,3);
    HAL_UART_ErrorCallback(&huart1);
    huart1.RxState=0;
    UartProcess();
    OnUartCmd(burst+3,2); // recovery boundary, must be dropped
    UartProcess();
    assert(frames==26);
    OnUartCmd(burst,5); UartProcess();
    assert(frames==27 && lastPosition==1.0f);
    std::puts("UART pipeline: 130-byte burst, TX buffer ownership, overrun recovery passed");
}

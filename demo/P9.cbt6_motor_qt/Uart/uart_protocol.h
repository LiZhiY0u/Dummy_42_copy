#ifndef P9_UART_PROTOCOL_H
#define P9_UART_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

void UartProtocolInit();

// Keil watch variables for locating the last working stage, no serial logging.
extern volatile uint32_t uartDiagRxBytes;
extern volatile uint32_t uartDiagValidFrames;
extern volatile uint32_t uartDiagTxStarts;
extern volatile uint32_t uartDiagPolls;
extern volatile uint32_t uartDiagControlTicks;
void UartProtocolPoll(uint32_t nowMs);
void UartProtocolControlTick(uint32_t nowMs);
void UartProtocolOnRx(const uint8_t *data, uint16_t length);
void UartProtocolOnTxComplete();
void UartProtocolOnError();
bool UartProtocolCanAcceptCanMotion();

#ifdef __cplusplus
}
#endif

#endif

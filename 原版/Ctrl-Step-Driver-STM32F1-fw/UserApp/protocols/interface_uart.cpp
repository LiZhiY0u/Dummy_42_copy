#include "common_inc.h"
#include "configurations.h"
#include "protocol_safety.h"
#include "uart_framer.h"
#include "usart.h"
#include <cstdlib>
#include <cstring>
#include <cmath>

extern Motor motor;
namespace {
constexpr uint16_t Capacity = 512;
uint16_t queue[Capacity];
volatile uint16_t head = 0, tail = 0;
volatile bool overflow = false;
volatile bool discardUntilIdle = false;
UartFramer framer;
bool telemetry = false;
uint8_t tx[5] = {0xaa, 0x55, 0x22, 0, 0};

void SetFloatCommand(uint8_t command, float value) {
    uint8_t payload[8]{};
    std::memcpy(payload, &value, 4);
    OnCanCmd(command, payload, 4);
}
void Dispatch(UartFramer::Event event) {
    if (event == UartFramer::Binary) {
        const uint8_t mode = framer.binary[2];
        const uint16_t value = (uint16_t(framer.binary[3]) << 8) | framer.binary[4];
        if (mode == Motor::MODE_COMMAND_VELOCITY) SetFloatCommand(0x04, (int32_t(value) - 3000) / 100.0f);
        else if (mode == Motor::MODE_COMMAND_POSITION) SetFloatCommand(0x05, (int32_t(value) - 5000) / 100.0f);
        else if (mode == 0x22 && framer.binary[4] == 0 && framer.binary[3] <= 1) telemetry = framer.binary[3] == 1;
        else if (mode == Motor::MODE_STOP && value == 0) { uint8_t stop[4]{}; OnCanCmd(0x01, stop, 4); }
    } else if (event == UartFramer::Text) {
        const char* line = framer.text;
        if (std::strcmp(line, "s") == 0) { uint8_t stop[4]{}; OnCanCmd(0x01, stop, 4); return; }
        if ((line[0] != 'c' && line[0] != 'v' && line[0] != 'p') || line[1] != ' ') return;
        char* end;
        const float value = std::strtof(line+2, &end);
        if (end == line+2) return;
        while (*end == ' ') ++end;
        if (*end || !std::isfinite(value)) return;
        SetFloatCommand(line[0] == 'c' ? 0x03 : line[0] == 'v' ? 0x04 : 0x05, value);
    }
}
}

// Both USART and DMA IRQs use the same preemption priority (3): one producer.
void QueueChunk(uint8_t* data, uint16_t length, bool idle)
{
    if (discardUntilIdle) {
        if (idle) { discardUntilIdle = false; overflow = true; }
        return;
    }
    if (!data || length > BUFFER_SIZE || overflow) return;
    const uint16_t available = (tail + Capacity - head - 1U) % Capacity;
    if (length + (idle ? 1U : 0U) > available) {
        overflow = true;
        discardUntilIdle = !idle;
        return;
    }
    for (uint16_t i = 0; i < length; ++i) {
        queue[head] = data[i];
        __DMB();
        head = (head + 1U) % Capacity;
    }
    if (idle) {
        queue[head] = 0x100;
        __DMB();
        head = (head + 1U) % Capacity;
    }
}

void OnUartCmd(uint8_t* data, uint16_t length) { QueueChunk(data, length, true); }
extern "C" void OnUartChunk(uint8_t* data, uint16_t length) { QueueChunk(data, length, false); }

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uart)
{
    if (uart != &huart1) return;
    // A full buffer is a chunk, NOT an idle-delimited text command.
    OnUartChunk(rx_buffer, BUFFER_SIZE);
    (void)HAL_UART_Receive_DMA(&huart1, rx_buffer, BUFFER_SIZE);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart)
{
    if (uart == &huart1) {
        overflow = true;
        discardUntilIdle = true;
    }
}

extern "C" void UartProcess()
{
    if (huart1.RxState == HAL_UART_STATE_READY)
        (void)HAL_UART_Receive_DMA(&huart1, rx_buffer, BUFFER_SIZE);
    if (overflow) {
        const uint32_t mask = __get_PRIMASK();
        __disable_irq();
        tail = head;
        overflow = false;
        framer.Reset();
        __set_PRIMASK(mask);
    }
    for (unsigned n = 0; n < 128 && tail != head; ++n) {
        const uint16_t value = queue[tail];
        __DMB();
        tail = (tail + 1U) % Capacity;
        Dispatch(value == 0x100 ? framer.Idle() : framer.Feed(value));
    }
    static uint32_t last = 0;
    const uint32_t now = HAL_GetTick();
    if (!telemetry || now-last < 10 || huart1.gState != HAL_UART_STATE_READY) return;
    last = now;
    float encoded = motor.controller->GetVelocity() * 1000.0f + 30000.0f;
    if (!std::isfinite(encoded)) return;
    if (encoded < 0) encoded = 0;
    if (encoded > 65535) encoded = 65535;
    const uint16_t value = static_cast<uint16_t>(encoded);
    // This buffer is not touched again until HAL marks the DMA transfer complete.
    tx[3] = value >> 8;
    tx[4] = value & 0xff;
    (void)HAL_UART_Transmit_DMA(&huart1, tx, sizeof tx);
}

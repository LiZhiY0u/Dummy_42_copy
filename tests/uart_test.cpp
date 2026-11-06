#include "uart_framer.h"
#include <cassert>
#include <cstring>
#include <cstdio>
int main() {
    UartFramer f;
    assert(f.Feed(0xaa) == UartFramer::None);
    assert(f.Idle() == UartFramer::None); // fragmented binary survives idle
    assert(f.Feed(0x55) == UartFramer::None);
    f.Feed(2); f.Feed(0x0b);
    assert(f.Feed(0xb8) == UartFramer::Binary);
    assert(f.binary[2] == 2 && f.binary[4] == 0xb8);
    for (int n=0;n<2;++n) {
        f.Feed(0xaa); f.Feed(0x55); f.Feed(1); f.Feed(0);
        assert(f.Feed(n) == UartFramer::Binary); // concatenated frames
    }
    for (char c : "v 1.5") if (c) f.Feed(c);
    assert(f.Idle() == UartFramer::Text && std::strcmp(f.text, "v 1.5") == 0);
    for (int n=0;n<100;++n) f.Feed('x');
    assert(f.Idle() == UartFramer::None); // overlong commands are discarded whole
    f.Feed('s');
    assert(f.Feed('\n') == UartFramer::Text && std::strcmp(f.text, "s") == 0);
    std::puts("UART: fragmented/coalesced binary, idle text, overflow resynchronization passed");
}

#pragma once
// Keep multi-field foreground updates atomic relative to motor/button IRQs.
// Header is also usable by host tests, where there are no hardware IRQs.
class InterruptGuard {
public:
    explicit InterruptGuard(bool active = true) : active_(active) {
#if defined(__arm__) || defined(__thumb__)
        if (active_) { __asm volatile("mrs %0, primask" : "=r"(saved_) :: "memory"); __asm volatile("cpsid i" ::: "memory"); }
#endif
    }
    ~InterruptGuard() {
#if defined(__arm__) || defined(__thumb__)
        if (active_) __asm volatile("msr primask, %0" :: "r"(saved_) : "memory");
#endif
    }
    InterruptGuard(const InterruptGuard&) = delete;
    InterruptGuard& operator=(const InterruptGuard&) = delete;
private:
    bool active_;
    unsigned saved_ = 0;
};

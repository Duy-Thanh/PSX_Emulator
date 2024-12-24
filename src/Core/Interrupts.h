#pragma once

#include <cstdint>

class InterruptController {
    uint32_t status;    // Current interrupts
    uint32_t mask;      // Interrupt mask
    
    void Trigger(uint32_t interrupt);
    bool IsInterruptPending();
    uint32_t GetPendingInterrupt();
};
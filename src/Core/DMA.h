#pragma once

#include <cstdint>
#include <array>

class DMA {
    struct Channel {
        uint32_t base_addr;
        uint32_t block_size;
        uint32_t block_count;
        uint32_t control;
    };
    
    std::array<Channel, 7> channels;  // 7 DMA channels
    uint32_t control;
    uint32_t interrupt;
    
    void TransferBlock(int channel);
};
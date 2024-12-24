#pragma once

#include <cstdint>
#include <array>

namespace PSX {
    class SPU {
    private:
        // SPU RAM: 512KB
        static constexpr uint32_t SPU_RAM_SIZE = 512 * 1024;
        std::array<uint8_t, SPU_RAM_SIZE> ram;

        // SPU Status Register
        uint16_t status = 0;

        // SPU Control Register
        uint16_t control = 0;

        // SPU Transfer Control
        bool transfer_busy = false;
        uint32_t transfer_addr = 0;

        // Voice registers (24 voices)
        struct Voice {
            uint16_t volume_left;
            uint16_t volume_right;
            uint16_t pitch;
            uint16_t start_addr;
            uint16_t adsr[2];
            uint16_t current_addr;
            uint16_t repeat_addr;
            bool key_on;
            bool key_off;
        };
        std::array<Voice, 24> voices;

        // Add these private methods for voice register access
        uint16_t ReadVoiceRegister(uint32_t voice, uint32_t reg);
        void WriteVoiceRegister(uint32_t voice, uint32_t reg, uint16_t value);

    public:
        SPU();
        ~SPU();

        void Reset();
        
        // Register access
        uint16_t ReadRegister(uint32_t addr);
        void WriteRegister(uint32_t addr, uint16_t value);
        
        // RAM access
        uint8_t ReadRAM(uint32_t addr);
        void WriteRAM(uint32_t addr, uint8_t value);
        
        // Status checks
        bool IsBusy() const { return transfer_busy; }
        bool IsTransferring() const { return transfer_busy; }
    };
} 
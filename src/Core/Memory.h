#pragma once

#include <cstdint>
#include <string>
#include <array>
#include "GPU.h"
#include "GTE.h"

namespace PSX {
    class Memory {
        private:
            std::array<uint8_t, 2 * 1024 * 1024> ram;    // 2MB RAM
            std::array<uint8_t, 512 * 1024> bios;        // 512KB BIOS
            std::array<uint8_t, 1024> scratchpad;        // 1KB Scratchpad

            // Memory map constants
            static constexpr uint32_t KUSEG_START = 0x00000000;  // User virtual memory
            static constexpr uint32_t KSEG0_START = 0x80000000;  // Kernel virtual memory (cached)
            static constexpr uint32_t KSEG1_START = 0xA0000000;  // Kernel virtual memory (uncached)
            static constexpr uint32_t KSEG2_START = 0xC0000000;  // Kernel virtual memory (isolated)

            static constexpr uint32_t RAM_START = 0x00000000;
            static constexpr uint32_t RAM_SIZE = 2 * 1024 * 1024;
            static constexpr uint32_t RAM_MASK = RAM_SIZE - 1;

            static constexpr uint32_t BIOS_START = 0x1FC00000;
            static constexpr uint32_t BIOS_SIZE = 512 * 1024;
            static constexpr uint32_t BIOS_MASK = BIOS_SIZE - 1;

            static constexpr uint32_t SCRATCHPAD_START = 0x1F800000;
            static constexpr uint32_t SCRATCHPAD_SIZE = 1024;
            static constexpr uint32_t SCRATCHPAD_MASK = SCRATCHPAD_SIZE - 1;

            uint32_t TranslateAddress(uint32_t address);

            GPU* gpu;
            GTE* gte;

        public:
            Memory();
            ~Memory();

            void Reset();
            bool LoadBIOS(const std::string& path);

            // Memory access functions
            uint8_t Read8(uint32_t address);
            uint16_t Read16(uint32_t address);
            uint32_t Read32(uint32_t address);

            void Write8(uint32_t address, uint8_t value);
            void Write16(uint32_t address, uint16_t value);
            void Write32(uint32_t address, uint32_t value);

            uint8_t ReadIO8(uint32_t address);
            void WriteIO8(uint32_t address, uint8_t value);

            uint16_t ReadIO16(uint32_t address);
            uint32_t ReadIO32(uint32_t address);
            void WriteIO16(uint32_t address, uint16_t value);
            void WriteIO32(uint32_t address, uint32_t value);

            void AttachGPU(GPU* gpu) { this->gpu = gpu; }
            void AttachGTE(GTE* gte) { this->gte = gte; }

            // Stub implementations for now
            uint8_t ReadJoyData();
            uint8_t ReadJoyStat();
            uint8_t ReadJoyMode();
            uint8_t ReadJoyCtrl();
            uint8_t ReadCDROM(uint8_t reg);
            uint8_t ReadMDEC(uint32_t address);

            void WriteJoyData(uint8_t value);
            void WriteJoyStat(uint8_t value);
            void WriteJoyMode(uint8_t value);
            void WriteJoyCtrl(uint8_t value);
            void WriteCDROM(uint8_t reg, uint8_t value);
            void WriteMDEC(uint32_t address, uint8_t value);
    };
}
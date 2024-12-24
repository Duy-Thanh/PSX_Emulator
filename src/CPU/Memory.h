#pragma once

#include <cstdint>
#include <string>
#include <array>

namespace PSX {
    class Memory {
        private:
            std::array<uint8_t, 2 * 1024 * 1024> ram; // 2MB RAM
            std::array<uint8_t, 512 * 1024> bios; // 512KB BIOS

            static constexpr uint32_t KUESG_START = 0x00000000;
            static constexpr uint32_t KESG0_START = 0x80000000;
            static constexpr uint32_t KSEG1_START = 0xA0000000;
            static constexpr uint32_t KSEG2_START = 0xC0000000;

            static constexpr uint32_t RAM_MASK = 0x1FFFFF;
            static constexpr uint32_t BIOS_MASK = 0x7FFFF;

            uint32_t TranslateAddress(uint32_t address);

        public:
            Memory();
            ~Memory();

            void Reset();
            bool LoadBIOS(const std::string& path);

            uint8_t Read8(uint32_t address);
            uint16_t Read16(uint32_t address);
            uint32_t Read32(uint32_t address);

            void Write8(uint32_t address, uint8_t value);
            void Write16(uint32_t address, uint16_t value);
            void Write32(uint32_t address, uint32_t value);
    };
}
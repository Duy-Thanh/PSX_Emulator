#pragma once

#include <cstdint>
#include <array>
#include <string>

namespace PSX {
    class CDROM {
    private:
        // CD-ROM registers
        uint8_t status = 0;
        uint8_t interrupt_flag = 0;
        uint8_t interrupt_enable = 0;

        // Command buffer
        std::array<uint8_t, 16> cmd_buffer;
        uint8_t cmd_index = 0;

        // Response buffer
        std::array<uint8_t, 16> response_buffer;
        uint8_t response_size = 0;

        // Data buffer
        static constexpr uint32_t SECTOR_SIZE = 2048;
        std::array<uint8_t, SECTOR_SIZE> data_buffer;
        bool data_ready = false;

        // Drive status
        bool motor_on = false;
        bool seeking = false;
        bool reading = false;
        bool shell_open = false;

        // Private methods
        void ExecuteCommand(uint8_t cmd);

    public:
        CDROM();
        ~CDROM();

        void Reset();
        
        // Register access
        uint8_t ReadRegister(uint32_t addr);
        void WriteRegister(uint32_t addr, uint8_t value);
        
        // Status checks
        bool IsBusy() const { return seeking || reading; }
        bool IsReading() const { return reading; }
        bool IsDataReady() const { return data_ready; }
        
        // Drive operations
        void Seek(uint32_t sector);
        void Read();
        void Stop();
    };
} 
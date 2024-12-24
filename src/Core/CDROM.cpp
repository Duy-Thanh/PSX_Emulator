#include "CDROM.h"

namespace PSX {
    CDROM::CDROM() {
        Reset();
    }

    CDROM::~CDROM() {
    }

    void CDROM::Reset() {
        // Reset registers
        status = 0;
        interrupt_flag = 0;
        interrupt_enable = 0;

        // Clear buffers
        cmd_buffer.fill(0);
        cmd_index = 0;
        response_buffer.fill(0);
        response_size = 0;
        data_buffer.fill(0);
        data_ready = false;

        // Reset drive state
        motor_on = false;
        seeking = false;
        reading = false;
        shell_open = false;
    }

    uint8_t CDROM::ReadRegister(uint32_t addr) {
        switch (addr & 3) {
            case 0: // Status register
                return status;
            case 1: // Response FIFO
                if (response_size > 0) {
                    uint8_t value = response_buffer[0];
                    // Shift response buffer
                    for (size_t i = 0; i < response_size - 1; i++) {
                        response_buffer[i] = response_buffer[i + 1];
                    }
                    response_size--;
                    return value;
                }
                return 0;
            case 2: // Data FIFO
                return data_ready ? data_buffer[0] : 0;
            case 3: // Interrupt Enable/Flag
                return (interrupt_enable << 4) | interrupt_flag;
            default:
                return 0;
        }
    }

    void CDROM::WriteRegister(uint32_t addr, uint8_t value) {
        switch (addr & 3) {
            case 0: // Command register
                cmd_buffer[cmd_index++] = value;
                if (cmd_index == 1) {
                    ExecuteCommand(value);
                }
                break;
            case 1: // Sound Map Data Out
                // Handle CD-DA volume
                break;
            case 2: // Sound Map Coding Info
                // Handle CD-XA ADPCM settings
                break;
            case 3: // Interrupt Enable/Flag
                if (value & 0x40) {
                    // Acknowledge interrupt
                    interrupt_flag &= ~(value & 0x1F);
                } else {
                    // Set interrupt enable
                    interrupt_enable = value & 0x1F;
                }
                break;
        }
    }

    void CDROM::ExecuteCommand(uint8_t cmd) {
        switch (cmd) {
            case 0x01: // GetStat
                response_buffer[0] = status;
                response_size = 1;
                interrupt_flag |= 3;
                break;
            case 0x02: // Setloc
                // Wait for 3 position bytes
                seeking = true;
                break;
            case 0x09: // Pause
                reading = false;
                motor_on = false;
                break;
            case 0x0A: // Init
                Reset();
                break;
            case 0x0E: // Setmode
                // Wait for mode byte
                break;
            case 0x15: // SeekL
                seeking = true;
                break;
            case 0x1B: // ReadN
                reading = true;
                motor_on = true;
                break;
            // ... other commands ...
        }
        cmd_index = 0;
    }

    void CDROM::Seek(uint32_t sector) {
        seeking = true;
        // Simulate seek time based on current position
        // This would be implemented in the emulator's timing system
    }

    void CDROM::Read() {
        if (!motor_on || !reading) return;

        // Simulate reading a sector
        data_ready = true;
        // This would be implemented in the emulator's timing system
    }

    void CDROM::Stop() {
        motor_on = false;
        reading = false;
        seeking = false;
        data_ready = false;
    }
} 
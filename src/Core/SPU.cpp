#include "SPU.h"

namespace PSX {
    SPU::SPU() {
        Reset();
    }

    SPU::~SPU() {
    }

    void SPU::Reset() {
        // Clear SPU RAM
        ram.fill(0);

        // Reset registers
        status = 0;
        control = 0;
        transfer_busy = false;
        transfer_addr = 0;

        // Reset all voices
        for (auto& voice : voices) {
            voice.volume_left = 0;
            voice.volume_right = 0;
            voice.pitch = 0;
            voice.start_addr = 0;
            voice.adsr[0] = 0;
            voice.adsr[1] = 0;
            voice.current_addr = 0;
            voice.repeat_addr = 0;
            voice.key_on = false;
            voice.key_off = false;
        }
    }

    uint16_t SPU::ReadRegister(uint32_t addr) {
        switch (addr & 0x3FF) {
            case 0x1AA: // SPUSTAT
                return status;
            case 0x1AC: // SPU Control
                return control;
            default:
                if (addr >= 0x1C0 && addr < 0x200) {
                    // Voice registers
                    uint32_t voice = (addr - 0x1C0) / 16;
                    uint32_t reg = (addr - 0x1C0) % 16;
                    return ReadVoiceRegister(voice, reg);
                }
                return 0;
        }
    }

    void SPU::WriteRegister(uint32_t addr, uint16_t value) {
        switch (addr & 0x3FF) {
            case 0x1AA: // SPUSTAT (read-only)
                break;
            case 0x1AC: // SPU Control
                control = value;
                break;
            default:
                if (addr >= 0x1C0 && addr < 0x200) {
                    // Voice registers
                    uint32_t voice = (addr - 0x1C0) / 16;
                    uint32_t reg = (addr - 0x1C0) % 16;
                    WriteVoiceRegister(voice, reg, value);
                }
        }
    }

    uint8_t SPU::ReadRAM(uint32_t addr) {
        if (addr < SPU_RAM_SIZE) {
            return ram[addr];
        }
        return 0;
    }

    void SPU::WriteRAM(uint32_t addr, uint8_t value) {
        if (addr < SPU_RAM_SIZE) {
            ram[addr] = value;
        }
    }

    uint16_t SPU::ReadVoiceRegister(uint32_t voice, uint32_t reg) {
        if (voice >= 24) return 0;

        switch (reg) {
            case 0: return voices[voice].volume_left;
            case 2: return voices[voice].volume_right;
            case 4: return voices[voice].pitch;
            case 6: return voices[voice].start_addr;
            case 8: return voices[voice].adsr[0];
            case 10: return voices[voice].adsr[1];
            case 12: return voices[voice].current_addr;
            case 14: return voices[voice].repeat_addr;
            default: return 0;
        }
    }

    void SPU::WriteVoiceRegister(uint32_t voice, uint32_t reg, uint16_t value) {
        if (voice >= 24) return;

        switch (reg) {
            case 0: voices[voice].volume_left = value; break;
            case 2: voices[voice].volume_right = value; break;
            case 4: voices[voice].pitch = value; break;
            case 6: voices[voice].start_addr = value; break;
            case 8: voices[voice].adsr[0] = value; break;
            case 10: voices[voice].adsr[1] = value; break;
            case 12: voices[voice].current_addr = value; break;
            case 14: voices[voice].repeat_addr = value; break;
        }
    }
} 
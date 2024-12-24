#include "Memory.h"
#include <fstream>
#include <iostream>

namespace PSX {
    Memory::Memory() {
        Reset();
    }

    Memory::~Memory() {
    }

    void Memory::Reset() {
        ram.fill(0);
        bios.fill(0);
        scratchpad.fill(0);
    }

    bool Memory::LoadBIOS(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file) {
            std::cerr << "Failed to open BIOS file: " << path << std::endl;
            return false;
        }

        file.read(reinterpret_cast<char*>(bios.data()), BIOS_SIZE);
        return true;
    }

    uint32_t Memory::TranslateAddress(uint32_t address) {
        // Remove region bits for KSEG0 and KSEG1
        if (address >= KSEG0_START && address < KSEG2_START) {
            address &= 0x1FFFFFFF;
        }

        return address;
    }

    uint8_t Memory::Read8(uint32_t address) {
        uint32_t masked_addr = TranslateAddress(address);
        
        // Add missing memory regions
        if (masked_addr >= 0x1F000000 && masked_addr < 0x1F080000) {  // Parallel I/O
            return ReadParallelPort(masked_addr);
        }
        else if (masked_addr >= 0x1F080000 && masked_addr < 0x1F100000) {  // Serial I/O
            return ReadSerialPort(masked_addr);
        }
        else if (masked_addr < 0x00800000) {              // RAM: 2MB
            return ram[masked_addr & 0x1FFFFF];
        }
        else if (masked_addr >= 0x1F800000 && masked_addr < 0x1F800400) {  // Scratchpad: 1KB
            return scratchpad[masked_addr & 0x3FF];
        }
        else if (masked_addr >= 0x1FC00000 && masked_addr < 0x1FC80000) {  // BIOS: 512KB
            return bios[masked_addr & 0x7FFFF];
        }
        else if (masked_addr >= 0x1F801000 && masked_addr < 0x1F803000) {  // I/O Ports
            return ReadIO8(masked_addr);
        }
        
        std::cerr << "Unhandled memory read8: 0x" << std::hex << address << std::endl;
        return 0xFF;
    }

    uint16_t Memory::Read16(uint32_t address) {
        if (address & 1) {
            std::cerr << "Unaligned memory read16: 0x" << std::hex << address << std::endl;
            return 0xFFFF;
        }
        
        return Read8(address) | (Read8(address + 1) << 8);
    }

    uint32_t Memory::Read32(uint32_t address) {
        if (address & 3) {
            std::cerr << "Unaligned memory read32: 0x" << std::hex << address << std::endl;
            return 0xFFFFFFFF;
        }
        
        return Read16(address) | (Read16(address + 2) << 16);
    }

    void Memory::Write8(uint32_t address, uint8_t value) {
        uint32_t masked_addr = address & 0x1FFFFFFF;  // Mask region bits
        
        if (masked_addr < 0x00800000) {              // RAM: 2MB
            ram[masked_addr & 0x1FFFFF] = value;
        }
        else if (masked_addr >= 0x1F800000 && masked_addr < 0x1F800400) {  // Scratchpad: 1KB
            scratchpad[masked_addr & 0x3FF] = value;
        }
        else if (masked_addr >= 0x1F801000 && masked_addr < 0x1F803000) {  // I/O Ports
            WriteIO8(masked_addr, value);
        }
        else {
            std::cerr << "Unhandled memory write8: 0x" << std::hex << address << " = " << (int)value << std::endl;
        }
    }

    void Memory::Write16(uint32_t address, uint16_t value) {
        if (address & 1) {
            std::cerr << "Unaligned memory write16: 0x" << std::hex << address << std::endl;
            return;
        }
        
        Write8(address, value & 0xFF);
        Write8(address + 1, (value >> 8) & 0xFF);
    }

    void Memory::Write32(uint32_t address, uint32_t value) {
        if (address & 3) {
            std::cerr << "Unaligned memory write32: 0x" << std::hex << address << std::endl;
            return;
        }
        
        Write16(address, value & 0xFFFF);
        Write16(address + 2, (value >> 16) & 0xFFFF);
    }

    uint8_t Memory::ReadIO8(uint32_t address) {
        switch (address) {
            case 0x1F801040:  // JOY_DATA
                return ReadJoyData();
                
            case 0x1F801044:  // JOY_STAT
                return ReadJoyStat();
                
            case 0x1F801048:  // JOY_MODE
                return ReadJoyMode();
                
            case 0x1F80104A:  // JOY_CTRL
                return ReadJoyCtrl();
                
            case 0x1F801800:  // CD ROM
                return ReadCDROM(0);
                
            case 0x1F801801:
                return ReadCDROM(1);
                
            case 0x1F801802:
                return ReadCDROM(2);
                
            case 0x1F801803:
                return ReadCDROM(3);
                
            default:
                if (address >= 0x1F801810 && address < 0x1F801814) {  // GPU
                    return gpu->ReadGPUSTAT();  // GPU status register
                }
                else if (address >= 0x1F801820 && address < 0x1F801828) {  // MDEC
                    return ReadMDEC(address);
                }
                
                std::cerr << "Unhandled I/O read8: 0x" << std::hex << address << std::endl;
                return 0xFF;
        }
    }

    void Memory::WriteIO8(uint32_t address, uint8_t value) {
        switch (address) {
            case 0x1F801040:  // JOY_DATA
                WriteJoyData(value);
                break;
                
            case 0x1F801044:  // JOY_STAT
                WriteJoyStat(value);
                break;
                
            case 0x1F801048:  // JOY_MODE
                WriteJoyMode(value);
                break;
                
            case 0x1F80104A:  // JOY_CTRL
                WriteJoyCtrl(value);
                break;
                
            case 0x1F801800:  // CD ROM
                WriteCDROM(0, value);
                break;
                
            case 0x1F801801:
                WriteCDROM(1, value);
                break;
                
            case 0x1F801802:
                WriteCDROM(2, value);
                break;
                
            case 0x1F801803:
                WriteCDROM(3, value);
                break;
                
            default:
                if (address == 0x1F801810) {  // GP0 - GPU Command/Data
                    gpu->WriteGP0(value);
                }
                else if (address == 0x1F801814) {  // GP1 - GPU Control/Status
                    gpu->WriteGP1(value);
                }
                else if (address >= 0x1F801820 && address < 0x1F801828) {  // MDEC
                    WriteMDEC(address, value);
                }
                else {
                    std::cerr << "Unhandled I/O write8: 0x" << std::hex << address << " = " << (int)value << std::endl;
                }
                break;
        }
    }

    uint16_t Memory::ReadIO16(uint32_t address) {
        if (address >= 0x1F801810 && address < 0x1F801814) {  // GPU
            return gpu->ReadGPUSTAT() & 0xFFFF;
        }
        return (ReadIO8(address) | (ReadIO8(address + 1) << 8));
    }

    uint32_t Memory::ReadIO32(uint32_t address) {
        if (address >= 0x1F801810 && address < 0x1F801814) {  // GPU
            if (address == 0x1F801810) {
                return gpu->ReadGPUREAD();
            }
            return gpu->ReadGPUSTAT();
        }
        return (ReadIO16(address) | (ReadIO16(address + 2) << 16));
    }

    void Memory::WriteIO16(uint32_t address, uint16_t value) {
        if (address == 0x1F801810) {  // GP0 - GPU Command/Data
            gpu->WriteGP0(value);
        }
        else if (address == 0x1F801814) {  // GP1 - GPU Control/Status
            gpu->WriteGP1(value);
        }
        else {
            WriteIO8(address, value & 0xFF);
            WriteIO8(address + 1, (value >> 8) & 0xFF);
        }
    }

    void Memory::WriteIO32(uint32_t address, uint32_t value) {
        if (address == 0x1F801810) {  // GP0 - GPU Command/Data
            gpu->WriteGP0(value);
        }
        else if (address == 0x1F801814) {  // GP1 - GPU Control/Status
            gpu->WriteGP1(value);
        }
        else {
            WriteIO16(address, value & 0xFFFF);
            WriteIO16(address + 2, (value >> 16) & 0xFFFF);
        }
    }

    // Stub implementations for now
    uint8_t Memory::ReadJoyData() { return 0xFF; }
    uint8_t Memory::ReadJoyStat() { return 0xFF; }
    uint8_t Memory::ReadJoyMode() { return 0xFF; }
    uint8_t Memory::ReadJoyCtrl() { return 0xFF; }
    uint8_t Memory::ReadCDROM(uint8_t reg) { return 0xFF; }
    uint8_t Memory::ReadMDEC(uint32_t address) { return 0xFF; }

    void Memory::WriteJoyData(uint8_t value) {}
    void Memory::WriteJoyStat(uint8_t value) {}
    void Memory::WriteJoyMode(uint8_t value) {}
    void Memory::WriteJoyCtrl(uint8_t value) {}
    void Memory::WriteCDROM(uint8_t reg, uint8_t value) {}
    void Memory::WriteMDEC(uint32_t address, uint8_t value) {}

    void Memory::WriteDMAControl(uint32_t channel, uint32_t value) {
        if (channel >= 7) return;
        
        DMAChannel& dma = dma_channels[channel];
        dma.control = value;
        
        // Check if transfer should start
        if (value & 0x01000000) {  // DMA enable bit
            switch (channel) {
                case 2:  // GPU
                    if (dma.control & 0x200) {  // CPU to GPU
                        uint32_t* src = (uint32_t*)(ram.data() + dma.base_addr);
                        gpu->DMAIn(src, dma.block_size);
                    } else {  // GPU to CPU
                        uint32_t* dst = (uint32_t*)(ram.data() + dma.base_addr);
                        gpu->DMAOut(dst, dma.block_size);
                    }
                    break;
                    
                // Add other DMA channels (MDEC, CD-ROM, etc.)
            }
        }
    }
}
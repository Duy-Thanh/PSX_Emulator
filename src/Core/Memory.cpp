#include "Memory.h"
#include <fstream>
#include <iostream>

namespace PSX {
    Memory::Memory() : gpu(nullptr), gte(nullptr) {
        InitDMAChannels();
        Reset();
    }

    Memory::~Memory() {
        // Hardware components are owned by CPU, don't delete them here
        gpu = nullptr;
        gte = nullptr;
    }

    void Memory::Reset() {
        ram.fill(0);
        scratchpad.fill(0);
        icache.fill(0);
        
        interrupt_stat = 0;
        interrupt_mask = 0;
        cache_enabled = false;
        
        // Reset peripheral states
        joy_transfer_active = false;
        joy_status = 0;
        joy_control = 0;
        joy_mode = 0;
        joy_baud = 0;
        joy_rx_data.fill(0);
        joy_tx_data.fill(0);
        joy_rx_pos = 0;
        joy_tx_pos = 0;

        cdrom_busy = false;
        cdrom_status = 0;
        cdrom_command = 0;
        cdrom_interrupt = 0;
        cdrom_response.fill(0);
        cdrom_sector_buffer.fill(0);
        cdrom_response_size = 0;
        cdrom_response_pos = 0;

        mdec_busy = false;
        mdec_status = 0;
        mdec_control = 0;

        spu_busy = false;
        spu_status = 0;
        spu_control = 0;
        
        InitDMAChannels();
    }

    bool Memory::LoadBIOS(const std::string& path) {
        FILE* file = fopen(path.c_str(), "rb");
        if (!file) {
            std::cerr << "Failed to open BIOS file: " << path << std::endl;
            return false;
        }

        size_t read = fread(bios.data(), 1, BIOS_SIZE, file);
        fclose(file);

        if (read != BIOS_SIZE) {
            std::cerr << "Failed to read BIOS file: " << path << 
                        " (read " << read << " bytes)" << std::endl;
            return false;
        }

        return true;
    }

    uint32_t Memory::TranslateAddress(uint32_t address) {
        // PS1 Quirk: Address masking behavior
        if (address >= KSEG0_START && address < KSEG2_START) {
            // KSEG0/KSEG1: Strip top 3 bits
            address &= 0x1FFFFFFF;
            
            // PS1 Quirk: BIOS mirrors in its 512KB region
            if (address >= BIOS_START && address < (BIOS_START + 0x80000)) {
                address = BIOS_START + (address & BIOS_MASK);
            }
            
            // PS1 Quirk: RAM mirrors
            if (address < RAM_SIZE) {
                address &= RAM_MASK;
            }
        }
        return address;
    }

    uint8_t Memory::Read8(uint32_t address) {
        uint32_t masked_addr = TranslateAddress(address);

        // RAM access
        if (masked_addr < RAM_SIZE) {
            return ram[masked_addr];
        }

        // BIOS access
        if (masked_addr >= BIOS_START && masked_addr < (BIOS_START + BIOS_SIZE)) {
            return bios[masked_addr & BIOS_MASK];
        }

        // Scratchpad access
        if (masked_addr >= SCRATCHPAD_START && 
            masked_addr < (SCRATCHPAD_START + SCRATCHPAD_SIZE)) {
            return scratchpad[masked_addr & SCRATCHPAD_MASK];
        }

        // I/O ports
        if (masked_addr >= 0x1F801000 && masked_addr < 0x1F803000) {
            return ReadIO8(masked_addr);
        }

        std::cerr << "Unhandled memory read8: 0x" << std::hex << address << std::endl;
        return 0xFF;
    }

    uint16_t Memory::Read16(uint32_t address) {
        // PS1 handles unaligned reads in hardware - no exceptions
        uint8_t b0 = Read8(address);
        uint8_t b1 = Read8(address + 1);
        return b0 | (b1 << 8);
    }

    uint32_t Memory::Read32(uint32_t address) {
        // PS1 Quirk: Cache behavior
        if (IsCacheable(address) && cache_enabled) {
            uint32_t cache_line = (address & 0x3F0) >> 4;
            uint32_t tag = address & ~0x3FF;
            
            if (cache_valid[cache_line] && cache_tags[cache_line] == tag) {
                // Cache hit
                uint32_t offset = address & 0xF;
                return *(uint32_t*)(&icache[cache_line * 16 + offset]);
            } else {
                // Cache miss - load new line
                UpdateCache(address);
            }
        }
        
        // PS1 Quirk: Memory mirroring in RAM region
        uint32_t masked_addr = TranslateAddress(address);
        if (masked_addr < RAM_SIZE) {
            masked_addr &= RAM_MASK;
            return *(uint32_t*)&ram[masked_addr];
        }
        
        // PS1 Quirk: BIOS mirroring
        if (masked_addr >= BIOS_START && masked_addr < (BIOS_START + BIOS_SIZE)) {
            masked_addr = (masked_addr - BIOS_START) & BIOS_MASK;
            return *(uint32_t*)&bios[masked_addr];
        }
        
        return ReadIO32(address);
    }

    void Memory::Write8(uint32_t address, uint8_t value) {
        uint32_t masked_addr = TranslateAddress(address);

        // RAM access
        if (masked_addr < RAM_SIZE) {
            ram[masked_addr] = value;
            return;
        }

        // Scratchpad access
        if (masked_addr >= SCRATCHPAD_START && 
            masked_addr < (SCRATCHPAD_START + SCRATCHPAD_SIZE)) {
            scratchpad[masked_addr & SCRATCHPAD_MASK] = value;
            return;
        }

        // I/O ports
        if (masked_addr >= 0x1F801000 && masked_addr < 0x1F803000) {
            WriteIO8(masked_addr, value);
            return;
        }

        // BIOS is read-only
        if (masked_addr >= BIOS_START && masked_addr < (BIOS_START + BIOS_SIZE)) {
            std::cerr << "Attempted write to BIOS: 0x" << std::hex << address << std::endl;
            return;
        }

        std::cerr << "Unhandled memory write8: 0x" << std::hex << address << 
                     " = 0x" << static_cast<int>(value) << std::endl;
    }

    void Memory::Write16(uint32_t address, uint16_t value) {
        // PS1 handles unaligned writes in hardware - no exceptions
        Write8(address, value & 0xFF);
        Write8(address + 1, (value >> 8) & 0xFF);
    }

    void Memory::Write32(uint32_t address, uint32_t value) {
        // PS1 Quirk: Cache control register
        if (address == CACHE_CTRL) {
            if (value & 0x800) {
                cache_enabled = true;
            }
            if (value & 0x4) {
                // Cache invalidation
                std::fill(cache_valid.begin(), cache_valid.end(), false);
                std::fill(icache.begin(), icache.end(), 0);
            }
            return;
        }
        
        // PS1 Quirk: DMA register auto-acknowledgment
        if (address >= 0x1F801080 && address < 0x1F801100) {
            uint32_t channel = (address >> 4) & 0x7;
            if ((address & 0xF) == 0x8) {  // Control register
                WriteDMAControl(channel, value);
                return;
            }
        }
        
        uint32_t masked_addr = TranslateAddress(address);
        
        // PS1 Quirk: Write to RAM
        if (masked_addr < RAM_SIZE) {
            masked_addr &= RAM_MASK;
            *(uint32_t*)&ram[masked_addr] = value;
            
            // PS1 Quirk: Cache coherency
            if (cache_enabled) {
                InvalidateCacheLine(address);
            }
            return;
        }
        
        WriteIO32(address, value);
    }

    uint8_t Memory::ReadIO8(uint32_t address) {
        // PS1 Quirk: Some I/O ports return last written value
        static uint8_t last_io_write = 0xFF;
        
        switch (address) {
            case 0x1F801040:  // JOY_DATA
                // PS1 Quirk: Reading during transfer returns 0xFF
                if (joy_transfer_active) {
                    return 0xFF;
                }
                return ReadJoyData();
                
            case 0x1F801044:  // JOY_STAT
                return ReadJoyStat();
                
            case 0x1F801048:  // JOY_MODE
                return ReadJoyMode();
                
            case 0x1F80104A:  // JOY_CTRL
                return ReadJoyCtrl();
                
            case 0x1F801800:  // CD ROM
                // PS1 Quirk: Reading while busy returns 0x80
                if (cdrom_busy) {
                    return 0x80;
                }
                // PS1 Quirk: Reading from CD response FIFO removes the byte
                if ((address & 3) == 1 && cdrom_response_size > 0) {
                    uint8_t value = cdrom_response[cdrom_response_pos];
                    cdrom_response_pos = (cdrom_response_pos + 1) % cdrom_response.size();
                    cdrom_response_size--;
                    return value;
                }
                return ReadCDROM(address & 3);
                
            default:
                if (address >= 0x1F801810 && address < 0x1F801814) {  // GPU
                    return gpu->ReadGPUSTAT();  // GPU status register
                }
                else if (address >= 0x1F801820 && address < 0x1F801828) {  // MDEC
                    return ReadMDEC(address);
                }
                
                std::cerr << "Unhandled I/O read8: 0x" << std::hex << address << std::endl;
                return last_io_write;
        }
    }

    void Memory::WriteIO8(uint32_t address, uint8_t value) {
        // PS1 Quirk: Some writes to I/O ports are ignored but still update last_io_write
        static uint8_t& last_io_write = *reinterpret_cast<uint8_t*>(ram.data() + 0x60);
        last_io_write = value;

        // PS1 Quirk: Writing to certain ports while busy is ignored
        if ((address >= 0x1F801800 && address < 0x1F801804 && cdrom_busy) ||
            (address >= 0x1F801820 && address < 0x1F801824 && mdec_busy)) {
            return;
        }

        // Normal I/O handling...
    }

    uint16_t Memory::ReadIO16(uint32_t address) {
        if (address >= 0x1F801810 && address < 0x1F801814) {  // GPU
            return gpu->ReadGPUSTAT() & 0xFFFF;
        }
        return (ReadIO8(address) | (ReadIO8(address + 1) << 8));
    }

    uint32_t Memory::ReadIO32(uint32_t address) {
        switch (address) {
            case 0x1F801070:  // I_STAT - Interrupt status
                return interrupt_stat;
            case 0x1F801074:  // I_MASK - Interrupt mask
                return interrupt_mask;
            // ... existing GPU and other cases ...
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
        switch (address) {
            case 0x1F801810:  // GPU DATA
                // PS1 Quirk: GPU busy state affects writes
                if (!gpu->IsBusy()) {
                    gpu->WriteGP0(value);
                }
                break;
            
            case 0x1F801814:  // GPU CONTROL
                // PS1 Quirk: Some GPU control commands execute even when busy
                if ((value >> 24) == 0x01 || (value >> 24) == 0x02) {
                    gpu->WriteGP1(value);  // Reset and display enable always work
                } else if (!gpu->IsBusy()) {
                    gpu->WriteGP1(value);
                }
                break;
            
            case 0x1F801820:  // MDEC DATA
                // PS1 Quirk: MDEC busy state blocks writes
                if (!mdec_busy) {
                    WriteMDEC(address, value);
                }
                break;
        }
    }

    // Stub implementations for now
    uint8_t Memory::ReadJoyData() {
        // PS1 Quirk: Controller state machine
        if (joy_transfer_active) {
            if (joy_rx_pos < joy_rx_data.size()) {
                uint8_t data = joy_rx_data[joy_rx_pos++];
                if (joy_rx_pos >= joy_rx_data.size()) {
                    joy_transfer_active = false;
                    joy_status |= 0x2;  // Transfer complete
                }
                return data;
            }
        }
        return 0xFF;  // No data available
    }

    uint8_t Memory::ReadJoyStat() {
        // Status bits:
        // Bit 0: TX Ready 1 (1=Ready/Started)
        // Bit 1: RX FIFO Not Empty (0=Empty)
        // Bit 2: TX Ready 2 (1=Ready/Finished)
        // Bit 7: ACK Input Level (0=Low, 1=High)
        return 0x5;  // TX ready, no data in RX FIFO
    }

    uint8_t Memory::ReadJoyMode() {
        // Return current mode settings
        return 0;  // Default mode
    }

    uint8_t Memory::ReadJoyCtrl() {
        // Return current control settings
        return 0;  // No special control bits set
    }

    uint8_t Memory::ReadCDROM(uint8_t reg) {
        switch (reg) {
            case 0:  // Status register
                return 0x18;  // Shell open, motor off
            case 1:  // Response FIFO
                return 0;
            case 2:  // Data FIFO
                return 0;
            case 3:  // IRQ Enable/Flags
                return 0;
            default:
                return 0xFF;
        }
    }

    uint8_t Memory::ReadMDEC(uint32_t address) { return 0xFF; }

    void Memory::WriteJoyData(uint8_t value) {}
    void Memory::WriteJoyStat(uint8_t value) {}
    void Memory::WriteJoyMode(uint8_t value) {}
    void Memory::WriteJoyCtrl(uint8_t value) {}
    void Memory::WriteCDROM(uint8_t reg, uint8_t value) {
        switch (reg) {
            case 0:  // Command register
                // Handle CD-ROM commands
                break;
            case 1:  // Sound Map Data Out
                break;
            case 2:  // Sound Map Coding Info
                break;
            case 3:  // IRQ Enable/Flags
                break;
        }
    }
    void Memory::WriteMDEC(uint32_t address, uint8_t value) {}

    void Memory::WriteDMAControl(uint32_t channel, uint32_t value) {
        if (channel >= dma_channels.size()) return;
        
        DMAChannel& dma = dma_channels[channel];
        dma.control = value;
        
        // Check if transfer should start
        if (value & 0x01000000) {  // DMA enable bit
            dma.active = true;
            HandleDMATransfer(channel);
        }
    }

    uint8_t Memory::ReadParallelPort(uint32_t address) {
        // Stub implementation
        return 0xFF;
    }

    uint8_t Memory::ReadSerialPort(uint32_t address) {
        // Stub implementation
        return 0xFF;
    }

    void Memory::WriteParallelPort(uint32_t address, uint8_t value) {
        // Stub implementation
    }

    void Memory::WriteSerialPort(uint32_t address, uint8_t value) {
        // Stub implementation
    }

    void Memory::SetDMABaseAddr(uint32_t channel, uint32_t addr) {
        if (channel < dma_channels.size()) {
            dma_channels[channel].base_addr = addr & 0x1FFFFF;  // Mask to RAM size
        }
    }

    void Memory::SetDMABlockSize(uint32_t channel, uint32_t size) {
        if (channel < dma_channels.size()) {
            dma_channels[channel].block_size = size;
        }
    }

    uint32_t Memory::GetDMAStatus(uint32_t channel) {
        if (channel < dma_channels.size()) {
            return dma_channels[channel].control;
        }
        return 0;
    }

    bool Memory::IsCacheable(uint32_t address) {
        // PS1 quirk: Only KSEG0 is cached, KSEG1 is not
        return (address >= KSEG0_START && address < KSEG1_START) && cache_enabled;
    }

    void Memory::UpdateCache(uint32_t address) {
        if (!cache_enabled) return;
        
        uint32_t cache_line = (address >> 4) & 0x3F;
        uint32_t cache_tag = address >> 10;
        
        // PS1 Quirk: Cache updates can happen even with invalid tags
        cache_tags[cache_line] = cache_tag;
        cache_valid[cache_line] = true;
        
        // PS1 Quirk: Cache fills from invalid memory return garbage
        if ((address & 0x1FFFFFFF) >= RAM_SIZE) {
            // Fill with repeating pattern based on address
            for (int i = 0; i < 16; i++) {
                icache[cache_line * 16 + i] = (address + i) & 0xFF;
            }
        } else {
            // Normal cache fill
            for (int i = 0; i < 16; i++) {
                icache[cache_line * 16 + i] = ram[(address & ~0xF) + i];
            }
        }
    }

    void Memory::InitDMAChannels() {
        for (auto& channel : dma_channels) {
            channel.base_addr = 0;
            channel.block_size = 0;
            channel.control = 0;
            channel.active = false;
        }
    }

    void Memory::HandleDMATransfer(uint32_t channel) {
        auto& dma = dma_channels[channel];
        
        // PS1 Quirk: DMA transfer behavior depends on channel
        switch (channel) {
            case 2:  // GPU
                if (!gpu->IsBusy()) {
                    // Transfer data to/from GPU
                    uint32_t addr = dma.base_addr & 0x1FFFFC;
                    for (uint32_t i = 0; i < dma.block_size; i++) {
                        if (dma.control & 1) {  // To RAM
                            Write32(addr, gpu->ReadGPUDATA());
                        } else {  // From RAM
                            gpu->WriteGP0(Read32(addr));
                        }
                        addr += 4;
                    }
                    dma.active = false;
                    interrupt_stat |= (1 << (channel + 24));  // Set DMA interrupt
                }
                break;
            
            case 3:  // CDROM
                if (!cdrom_busy) {
                    // Handle CDROM DMA
                    // ... CDROM specific DMA logic ...
                }
                break;
        }
    }

    void Memory::InvalidateCacheLine(uint32_t address) {
        if (!cache_enabled) return;
        
        uint32_t cache_line = (address >> 4) & 0x3F;
        uint32_t cache_tag = address >> 10;
        
        // Clear cache line
        std::fill(icache.begin() + cache_line * 16, 
                  icache.begin() + (cache_line + 1) * 16, 
                  0);
    }
}
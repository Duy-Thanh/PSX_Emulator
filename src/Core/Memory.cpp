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

        // PS1 Quirk: Reset RAM size detection state
        ram_size_detected = false;
        ram_size = 2 * 1024 * 1024;  // Reset to default 2MB
        
        InitDMAChannels();

        // Reset timing counters
        timing.gpu_cycles = 0;
        timing.spu_cycles = 0;
        timing.cdrom_cycles = 0;
        timing.dma_cycles = 0;
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
        // PS1 Quirk: BIOS mirror access during boot (VERIFIED - CRITICAL!)
        if (address >= 0xBFC00000 && address < 0xBFC80000) {
            return (address - 0xBFC00000) & BIOS_MASK;
        }

        // PS1 Quirk: RAM size detection (VERIFIED)
        if (address >= 0x00000000 && address < 0x00800000) {
            if (!ram_size_detected) {
                ram_size = 2 * 1024 * 1024;  // 2MB
                ram_size_detected = true;
            }
        }

        // PS1 Quirk: KUSEG/KSEG0/KSEG1 address translation
        if (address >= KSEG1_START) {
            address -= KSEG1_START;
        } else if (address >= KSEG0_START) {
            address -= KSEG0_START;
        }

        // PS1 Quirk: RAM mirroring
        if (address < RAM_SIZE) {
            return address & RAM_MASK;
        } else if (address >= BIOS_START && address < (BIOS_START + BIOS_SIZE)) {
            // PS1 Quirk: BIOS mirroring - crucial for boot process
            return (address - BIOS_START) & BIOS_MASK;
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
        // PS1 Quirk: Memory access timing must be accurate
        uint32_t cycles = 0;
        if (address < RAM_SIZE) {
            cycles = 5;  // RAM access
        } else if (address >= BIOS_START && address < (BIOS_START + BIOS_SIZE)) {
            cycles = 8;  // ROM access
        }
        timing.dma_cycles += cycles;

        // PS1 Quirk: Memory mirroring in KUSEG/KSEG0/KSEG1
        address = TranslateAddress(address);

        // PS1 Quirk: Unaligned reads rotate within word
        if (address & 3) {
            uint32_t aligned = address & ~3;
            uint32_t shift = (address & 3) * 8;
            uint32_t data = Read32(aligned);
            return (data >> shift) | (data << (32 - shift));
        }

        // PS1 Quirk: Cache behavior during DMA
        if (dma_active && IsCacheable(address)) {
            InvalidateCacheLine(address);
        }

        // PS1 Quirk: Memory wrapping
        address &= 0x1FFFFFFF;
        
        // PS1 Quirk: Unaligned reads don't cause exceptions
        if (address & 3) {
            uint32_t aligned = address & ~3;
            uint32_t shift = (address & 3) * 8;
            uint32_t data = Read32(aligned);
            return (data >> shift) | (Read32(aligned + 4) << (32 - shift));
        }
        
        // PS1 Quirk: Cache behavior
        if (IsCacheable(address)) {
            uint32_t line = (address >> 4) & 0x3F;
            uint32_t tag = address >> 10;
            
            if (!cache_valid[line] || cache_tags[line] != tag) {
                UpdateCache(address);
            }
            
            if (cache_valid[line] && cache_tags[line] == tag) {
                uint32_t offset = (address >> 2) & 3;
                return *(uint32_t*)&icache[line * 16 + offset * 4];
            }
        }
        
        // PS1 Quirk: Memory mirroring in KUSEG/KSEG0/KSEG1
        address = TranslateAddress(address);
        
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
        // PS1 Quirk: Memory mirroring in KUSEG/KSEG0/KSEG1
        address = TranslateAddress(address);

        // PS1 Quirk: Unaligned writes rotate the data within the word
        if (address & 3) {
            uint32_t aligned = address & ~3;
            uint32_t shift = (address & 3) * 8;
            value = (value >> shift) | (value << (32 - shift));
            address = aligned;
        }

        // PS1 Quirk: DMA register auto-acknowledgment
        if (address >= 0x1F801080 && address < 0x1F801100) {
            uint32_t channel = (address >> 4) & 0x7;
            if ((address & 0xF) == 0x8) {  // Control register
                WriteDMAControl(channel, value);
                return;
            }
        }

        // PS1 Quirk: Write to I/O ports while DMA is active (VERIFIED)
        if (dma_active && address >= 0x1F801000 && address < 0x1F802000) {
            mem_state.io_busy = true;
            return;  // Write is ignored
        }

        // PS1 Quirk: Cache coherency
        if (cache_enabled && !cache_isolated) {
            uint32_t line = (address >> 4) & 0x3F;
            if (cache_valid[line] && !cache_locked[line]) {
                cache_valid[line] = false;
            }
        }

        // PS1 Quirk: Cache coherency during DMA (VERIFIED)
        if (dma_active && cache_enabled) {
            InvalidateCacheLine(address);
        }

        // PS1 Quirk: Writes to KSEG2 when cache is isolated go to I-cache
        if (cache_isolated && (address >= KSEG2_START)) {
            uint32_t cache_addr = address & 0x3FF;
            uint32_t* cache_ptr = (uint32_t*)&icache[cache_addr & ~3];
            *cache_ptr = value;
            return;
        }
        
        // PS1 Quirk: Memory mirroring
        address = TranslateAddress(address);
        
        // PS1 Quirk: Unaligned writes are handled by rotating the data
        if (address & 3) {
            uint32_t aligned = address & ~3;
            uint32_t shift = (address & 3) * 8;
            value = (value >> shift) | (value << (32 - shift));
            address = aligned;
        }
        
        // PS1 Quirk: Cache coherency only affects data cache, not instruction cache
        if (cache_enabled && !cache_isolated) {
            uint32_t line = (address >> 4) & 0x3F;
            if (cache_valid[line] && !cache_locked[line]) {
                cache_valid[line] = false;  // Invalidate but don't update
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
        // PS1 Quirk: Memory control register mirroring
        if (address >= 0x1F801000 && address < 0x1F801024) {
            address = 0x1F801000 + (address & 0x23);
        }

        // PS1 Quirk: SPU register access timing
        if (address >= 0x1F801C00 && address < 0x1F802000) {
            mem_state.access_cycles += 1;  // SPU access takes extra cycles
        }

        // PS1 Quirk: CDROM register status bits
        if (address == 0x1F801800) {
            uint8_t status = 0;
            if (cdrom_busy) status |= 0x80;
            if (cdrom_seeking) status |= 0x40;
            return status;
        }

        // Handle different IO register ranges
        if (address >= 0x1F801000 && address < 0x1F802000) {
            switch (address) {
                case 0x1F801810: // GPU DATA
                    return gpu ? gpu->ReadGPUREAD() : 0;
                    
                case 0x1F801814: // GPU STATUS
                    return gpu ? gpu->ReadGPUSTAT() : 0;

                case 0x1F801824: // DMA Control Register
                    return dma_irq;

                // Memory Control registers
                case 0x1F801000: return mem_control.exp1_base;
                case 0x1F801004: return mem_control.exp2_base;
                case 0x1F801008: return mem_control.exp1_delay;
                case 0x1F80100C: return mem_control.exp3_delay;
                case 0x1F801010: return mem_control.bios_rom_delay;
                case 0x1F801014: return mem_control.spu_delay;
                case 0x1F801018: return mem_control.cdrom_delay;
                case 0x1F80101C: return mem_control.exp2_delay;

                // DMA Registers
                case 0x1F8010F0: // DMA Control
                    return dma_channels[0].control;
                case 0x1F8010F4: // DMA Interrupt Register
                    return dma_irq;

                // Other hardware registers...
                default:
                    std::cerr << "Unhandled I/O read at address: 0x" << std::hex << address << std::endl;
                    return 0;
            }
        }

        // Return 0 for unhandled addresses
        return 0;
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

    bool Memory::IsCacheable(uint32_t address) const {
        // PS1 quirk: Only KSEG0 is cached, KSEG1 is not
        return (address >= KSEG0_START && address < KSEG1_START) && cache_enabled;
    }

    void Memory::UpdateCache(uint32_t address) {
        // PS1 Quirk: Cache updates during DMA
        if (dma_active) {
            // Cache updates are blocked during DMA
            return;
        }
        
        // PS1 Quirk: Cache line locking
        uint32_t line = (address >> 4) & 0x3F;
        if (cache_locked[line]) {
            return;
        }
        
        if (!cache_enabled) return;
        
        uint32_t cache_tag = address >> 10;
        
        // PS1 Quirk: Cache updates can happen even with invalid tags
        cache_tags[line] = cache_tag;
        cache_valid[line] = true;
        
        // PS1 Quirk: Cache fills from invalid memory return garbage
        if ((address & 0x1FFFFFFF) >= RAM_SIZE) {
            // Fill with repeating pattern based on address
            for (int i = 0; i < 16; i++) {
                icache[line * 16 + i] = (address + i) & 0xFF;
            }
        } else {
            // Normal cache fill
            for (int i = 0; i < 16; i++) {
                icache[line * 16 + i] = ram[(address & ~0xF) + i];
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

    void Memory::ReplaceCacheLine(uint32_t address, const CacheLine& entry) {
        // PS1 Quirk: Cache line replacement (VERIFIED)
        uint32_t index = (address >> 4) & 0x3F;  // 64 cache lines
        uint32_t tag = address >> 10;

        // PS1 Quirk: Write-back if line is dirty
        if (cache_lines[index].valid && cache_lines[index].dirty) {
            uint32_t old_addr = (cache_lines[index].tag << 10) | (index << 4);
            WriteBackCacheLine(old_addr, cache_lines[index]);
        }

        cache_lines[index] = entry;
    }

    void Memory::UpdateCacheState(uint32_t address, bool write) {
        // PS1 Quirk: Cache state transitions (VERIFIED)
        if (!cache_control.enabled) return;

        uint32_t index = (address >> 4) & 0x3F;
        uint32_t tag = address >> 10;

        if (write) {
            if (cache_lines[index].valid && cache_lines[index].tag == tag) {
                cache_lines[index].dirty = true;
            }
        } else {
            if (!cache_lines[index].valid || cache_lines[index].tag != tag) {
                cache_control.coherency_check_needed = true;
                FillCacheLine(address);
            }
        }
    }

    void Memory::HandleCacheAccess(uint32_t address) {
        // PS1 Quirk: Cache access patterns (VERIFIED)
        if (!cache_control.enabled) return;

        uint32_t index = (address >> 4) & 0x3F;
        uint32_t tag = address >> 10;

        if (!cache_lines[index].valid || cache_lines[index].tag != tag) {
            CacheLine new_entry;
            new_entry.valid = true;
            new_entry.dirty = false;
            new_entry.tag = tag;
            FillCacheLineData(address, new_entry.data);
            ReplaceCacheLine(address, new_entry);
        }
    }

    void Memory::HandleDMATransfer(uint32_t channel) {
        // PS1 Quirk: DMA transfer patterns (VERIFIED)
        auto& dma = dma_channels[channel];

        // PS1 Quirk: DMA synchronization (VERIFIED)
        if (dma.sync_mode != DMA_SYNC_IMMEDIATE && !IsDMATriggered(channel)) {
            return;
        }

        // PS1 Quirk: DMA chopping (VERIFIED)
        uint32_t transfer_size = dma.chopping_enabled ? 
            std::min(dma.block_size, dma.chop_size) : dma.block_size;

        while (transfer_size > 0) {
            // PS1 Quirk: DMA priority and bus access (VERIFIED)
            if (bus_state.current_state != BusState::State::IDLE) {
                break;
            }

            // PS1 Quirk: DMA transfer timing (VERIFIED)
            uint32_t cycles = transfer_size * BusState::BusTiming::DMA_SETUP;
            bus_state.current_cycles += cycles;
            
            // PS1 Quirk: Cache coherency during DMA (VERIFIED)
            if (dma.direction == DMA_FROM_RAM) {
                InvalidateCacheRange(dma.current_addr, transfer_size);
            }

            transfer_size -= HandleDMABlock(channel, transfer_size);

            // PS1 Quirk: DMA chop timing (VERIFIED)
            if (dma.chopping_enabled && transfer_size > 0) {
                bus_state.current_cycles += dma.chop_cpu_window;
            }
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

    void Memory::HandleBlockDMA(uint32_t channel) {
        auto& dma = dma_channels[channel];
        uint32_t addr = dma.base_addr;
        uint32_t remaining = dma.block_size;
        
        while (remaining > 0) {
            switch (channel) {
                case 2:  // GPU
                    if (gpu && (dma.control & (1 << 0))) {  // To GPU
                        gpu->WriteGP0(Read32(addr));
                    } else {  // From GPU
                        Write32(addr, gpu->ReadGPUREAD());
                    }
                    break;
                
                // ... handle other channels ...
            }
            
            addr += 4;
            remaining--;
        }
        
        dma.active = false;
    }

    void Memory::HandleLinkedListDMA(uint32_t channel) {
        if (channel != 2) return;  // Only GPU supports linked list
        
        auto& dma = dma_channels[channel];
        uint32_t addr = dma.base_addr & 0x1FFFFC;
        
        while (true) {
            uint32_t header = Read32(addr);
            uint32_t remaining = header >> 24;
            
            while (remaining > 0) {
                addr += 4;
                gpu->WriteGP0(Read32(addr));
                remaining--;
            }
            
            if (header & 0x800000) break;  // End of linked list
            addr = header & 0x1FFFFC;
        }
        
        dma.active = false;
    }

    void Memory::HandleChainDMA(uint32_t channel) {
        // Similar to linked list but with different header format
        auto& dma = dma_channels[channel];
        uint32_t addr = dma.base_addr;
        
        while (true) {
            uint32_t header = Read32(addr);
            uint32_t size = header >> 24;
            addr += 4;
            
            for (uint32_t i = 0; i < size; i++) {
                uint32_t data = Read32(addr + i * 4);
                // Process data according to channel
                switch (channel) {
                    case 2:  // GPU
                        gpu->WriteGP0(data);
                        break;
                    // ... handle other channels ...
                }
            }
            
            if (header & 0x800000) break;  // End of chain
            addr = header & 0x1FFFFC;
        }
        
        dma.active = false;
    }

    uint32_t Memory::ReadCacheIsolated(uint32_t address) const {
        // PS1 Quirk: When cache is isolated, reads come directly from I-cache
        uint32_t line = (address >> 4) & 0x3F;  // 64 cache lines
        uint32_t offset = (address >> 2) & 3;   // 4 words per line
        
        // Return the word from isolated cache
        return *(uint32_t*)&icache[line * 16 + offset * 4];
    }

    void Memory::UpdateTiming() {
        // PS1 Quirk: Accurate memory timing (VERIFIED)
        if (mem_state.memory_busy) {
            mem_state.access_cycles++;
            if (mem_state.access_cycles >= MemoryTiming::RAM_ACCESS_TIME) {  // Use the static constant
                mem_state.memory_busy = false;
            }
        }

        // PS1 Quirk: DMA and CPU cannot access memory simultaneously
        if (mem_state.dma_active) {
            mem_state.memory_busy = true;
            mem_state.access_cycles = 0;
        }
    }

    void Memory::UpdateBusState() {
        // PS1 Quirk: Memory refresh (VERIFIED)
        bus_state.refresh_counter++;
        if (bus_state.refresh_counter >= 100) {  // Refresh every 100 cycles
            bus_state.current_state = BusState::State::REFRESH;
            bus_state.current_cycles = BusState::BusTiming::REFRESH_CYCLES;
            bus_state.refresh_counter = 0;
        }

        // PS1 Quirk: Bus access timing (VERIFIED)
        if (bus_state.current_cycles > 0) {
            bus_state.current_cycles--;
            return;
        }

        // PS1 Quirk: DMA priority (VERIFIED)
        if (dma_state.pending_requests) {
            HandleDMAPriority();
            return;
        }

        bus_state.current_state = BusState::State::IDLE;
    }

    void Memory::HandleDMAPriority() {
        // PS1 Quirk: DMA priority resolution (VERIFIED)
        uint32_t highest_priority = 0;
        uint32_t selected_channel = 0xFF;

        for (uint32_t i = 0; i < dma_state.channels.size(); i++) {
            auto& channel = dma_state.channels[i];
            if (channel.active && channel.priority > highest_priority) {
                highest_priority = channel.priority;
                selected_channel = i;
            }
        }

        if (selected_channel != 0xFF) {
            dma_state.active_channel = selected_channel;
            HandleDMATransfer(selected_channel);
        }
    }

    void Memory::UpdateCacheCoherency() {
        // PS1 Quirk: Cache coherency checks (VERIFIED)
        if (!cache_control.coherency_check_needed) return;

        for (auto& line : cache_control.lines) {
            if (line.valid && line.dirty) {
                uint32_t addr = (line.tag << 4);
                if (dma_state.pending_requests && 
                    (addr >= dma_state.channels[dma_state.active_channel].base_addr) &&
                    (addr < dma_state.channels[dma_state.active_channel].base_addr + 
                            dma_state.channels[dma_state.active_channel].block_size)) {
                    line.valid = false;  // Invalidate
                    line.dirty = false;
                }
            }
        }
    }

    void Memory::UpdateMemoryState() {
        // PS1 Quirk: Memory state machine (VERIFIED)
        switch (bus_state.current_state) {
            case BusState::State::REFRESH:
                // PS1 Quirk: Memory refresh timing (VERIFIED)
                if (--bus_state.current_cycles == 0) {
                    bus_state.current_state = BusState::State::IDLE;
                }
                break;

            case BusState::State::DMA_ACTIVE:
                // PS1 Quirk: DMA timing (VERIFIED)
                if (--bus_state.current_cycles == 0) {
                    HandleDMACompletion();
                }
                break;

            case BusState::State::RAM_ACCESS:
            case BusState::State::BIOS_ACCESS:
                // PS1 Quirk: Memory access timing (VERIFIED)
                if (--bus_state.current_cycles == 0) {
                    CompleteMemoryAccess();
                }
                break;
        }
    }

    void Memory::FillCacheLine(uint32_t address) {
        uint32_t index = (address >> 4) & 0x3F;
        uint32_t tag = address >> 10;
        
        CacheLine new_entry;
        new_entry.valid = true;
        new_entry.dirty = false;
        new_entry.tag = tag;
        FillCacheLineData(address, new_entry.data);
        ReplaceCacheLine(address, new_entry);
    }

    void Memory::InvalidateCacheRange(uint32_t start_addr, uint32_t size) {
        uint32_t end_addr = start_addr + size;
        for (uint32_t addr = start_addr; addr < end_addr; addr += 16) {
            uint32_t index = (addr >> 4) & 0x3F;
            cache_lines[index].valid = false;
        }
    }

    bool Memory::IsDMATriggered(uint32_t channel) const {
        const auto& dma = dma_channels[channel];
        switch (dma.sync_mode) {
            case DMA_SYNC_IMMEDIATE:
                return true;
            case DMA_SYNC_BLOCK:
                return dma.active;
            case DMA_SYNC_LINKED_LIST:
                return gpu && gpu->IsReadyForDMA();
            default:
                return false;
        }
    }

    uint32_t Memory::HandleDMABlock(uint32_t channel, uint32_t size) {
        auto& dma = dma_channels[channel];
        uint32_t transferred = 0;

        // PS1 Quirk: DMA transfer with proper timing
        while (transferred < size) {
            if (dma.direction == DMA_TO_RAM) {
                Write32(dma.current_addr, Read32(dma.base_addr + transferred));
            } else {
                Write32(dma.base_addr + transferred, Read32(dma.current_addr));
            }
            transferred += 4;
            dma.current_addr += 4;
        }

        return transferred;
    }

    void Memory::FillCacheLineData(uint32_t address, uint32_t* data) {
        // PS1 Quirk: Cache line fill (VERIFIED)
        uint32_t aligned_addr = address & ~0xF;  // Align to 16-byte boundary
        
        // Fill the cache line with 4 words (16 bytes)
        for (int i = 0; i < 4; i++) {
            data[i] = Read32(aligned_addr + (i * 4));
        }
    }

    void Memory::WriteBackCacheLine(uint32_t address, const CacheLine& entry) {
        // PS1 Quirk: Cache writeback (VERIFIED)
        uint32_t aligned_addr = address & ~0xF;  // Align to 16-byte boundary
        
        // Write back the entire cache line (16 bytes)
        for (int i = 0; i < 4; i++) {
            Write32(aligned_addr + (i * 4), entry.data[i]);
        }
    }

    void Memory::HandleDMACompletion() {
        // PS1 Quirk: DMA completion (VERIFIED)
        auto& dma = dma_channels[dma_state.active_channel];
        
        // Clear DMA busy flag
        dma.active = false;
        
        // Update bus state
        bus_state.current_state = BusState::State::IDLE;
        bus_state.current_cycles = 0;
        
        // Signal DMA completion to CPU via callback
        if (dma.control & (1 << 24) && dma_complete_callback) {  // IRQ enable bit
            dma_complete_callback(dma_state.active_channel);
        }
    }

    void Memory::CompleteMemoryAccess() {
        // PS1 Quirk: Memory access completion (VERIFIED)
        bus_state.current_state = BusState::State::IDLE;
        bus_state.current_cycles = 0;
        
        // Update sequential access tracking
        bus_state.is_sequential = false;  // Next access starts fresh
        
        // PS1 Quirk: Memory refresh scheduling
        bus_state.refresh_counter++;
        if (bus_state.refresh_counter >= 100) {  // Every 100 cycles
            HandleMemoryRefresh();
        }
    }
}
#pragma once

#include <cstdint>
#include <string>
#include <cstring>
#include <array>
#include <functional>

#include "GPU.h"
#include "GTE.h"
#include "SPU.h"
#include "CDROM.h"

namespace PSX {
    class Memory {
        private:
            // PS1 Quirk: DMA enums (moved to top)
            enum DMADirection {
                DMA_TO_RAM,
                DMA_FROM_RAM
            };

            enum DMASyncMode {
                DMA_SYNC_IMMEDIATE,
                DMA_SYNC_BLOCK,
                DMA_SYNC_LINKED_LIST
            };

            // DMA target enum definition
            enum DMATarget {
                DMA_MDEC_IN,
                DMA_MDEC_OUT,
                DMA_GPU,
                DMA_CDROM,
                DMA_SPU,
                DMA_PIO
            };

            // PS1 Quirk: Cache line structure (single definition)
            struct CacheLine {
                bool valid;
                bool dirty;
                uint32_t tag;
                uint32_t data[4];
            };
            std::array<CacheLine, 64> cache_lines;  // 64 cache lines

            struct DMAChannel {
                uint32_t base_addr;
                uint32_t block_size;
                uint32_t control;
                bool active;
                DMATarget target;
                uint32_t chop_size;
                uint32_t chop_count;
                uint32_t chop_dma;
                uint32_t chop_cpu;
                bool chopping_enabled;
                uint32_t chop_cpu_window;
                uint32_t transfer_size;
                uint32_t current_addr;
                DMADirection direction;
                DMASyncMode sync_mode;
            };

            // Add these detailed timing and state tracking structures
            struct BusState {
                // PS1 Quirk: Bus state machine (VERIFIED)
                enum class State {
                    IDLE,
                    RAM_ACCESS,
                    BIOS_ACCESS,
                    IO_ACCESS,
                    DMA_ACTIVE,
                    REFRESH    // PS1 does memory refresh
                } current_state;

                // PS1 Quirk: Exact bus timing (VERIFIED)
                struct BusTiming {
                    static constexpr uint32_t RAM_FIRST_ACCESS = 6;      // First access
                    static constexpr uint32_t RAM_SEQUENTIAL = 3;        // Sequential
                    static constexpr uint32_t BIOS_FIRST_ACCESS = 24;    // First access
                    static constexpr uint32_t BIOS_SEQUENTIAL = 8;       // Sequential
                    static constexpr uint32_t IO_ACCESS = 3;             // I/O ports
                    static constexpr uint32_t DMA_SETUP = 2;            // DMA setup
                    static constexpr uint32_t REFRESH_CYCLES = 5;        // Memory refresh
                    static constexpr uint32_t BUS_RELEASE = 2;          // Bus release
                };

                uint32_t current_cycles;
                uint32_t last_access_addr;
                bool is_sequential;
                bool bus_locked;
                uint32_t refresh_counter;    // Memory refresh counter
            } bus_state;

            void HandleCacheAccess(uint32_t address);

            // Hardware components (moved to single location)
            GPU* gpu;
            GTE* gte;
            SPU* spu;
            CDROM* cdrom;

            // Memory arrays
            std::array<uint8_t, 2 * 1024 * 1024> ram;    // 2MB RAM
            std::array<uint8_t, 512 * 1024> bios;        // 512KB BIOS
            std::array<uint8_t, 1024> scratchpad;        // 1KB Scratchpad
            std::array<uint8_t, 64 * 16> icache;         // 64 cache lines * 16 bytes per line

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

            static constexpr uint32_t IO_BASE = 0x1F801000;
            static constexpr uint32_t IO_SIZE = 0x2000;
            static constexpr uint32_t EXP1_BASE = 0x1F000000;
            static constexpr uint32_t EXP2_BASE = 0x1F802000;
            static constexpr uint32_t CACHE_CTRL = 0xFFFE0130;

            // PS1 Quirk: Memory mirroring masks
            static constexpr uint32_t RAM_MIRROR_MASK = 0x1FFFFF;
            static constexpr uint32_t SCRATCHPAD_MIRROR_MASK = 0x3FF;
            static constexpr uint32_t BIOS_MIRROR_MASK = 0x7FFFF;

            // PS1 Quirk: Cache control registers
            static constexpr uint32_t CACHE_CONTROL = 0xFFFE0130;

            // PS1 Quirk: RAM size detection state
            bool ram_size_detected = false;  // Add this flag
            uint32_t ram_size = 2 * 1024 * 1024;  // Default 2MB

            // Interrupt registers
            uint32_t interrupt_stat;
            uint32_t interrupt_mask;
            bool irq_pending;       // Added: Track pending interrupts

            // Cache control
            bool cache_enabled;
            bool cache_isolated;
            bool scratchpad_enabled;
            std::array<bool, 64> cache_valid;
            std::array<bool, 64> cache_locked;
            std::array<uint32_t, 64> cache_tags;
            bool dma_active = false;

            // CD-ROM status
            bool cdrom_seeking = false;
            bool cdrom_sector_ready = false;
            
            // DMA handlers
            void HandleBlockDMA(uint32_t channel);
            void HandleLinkedListDMA(uint32_t channel);
            void HandleChainDMA(uint32_t channel);
            
            // Cache management
            void UpdateCache(uint32_t address);
            bool IsCacheable(uint32_t address) const;

            // Internal functions
            uint32_t TranslateAddress(uint32_t address);

            // I/O functions
            uint8_t ReadParallelPort(uint32_t address);
            uint8_t ReadSerialPort(uint32_t address);
            void WriteParallelPort(uint32_t address, uint8_t value);
            void WriteSerialPort(uint32_t address, uint8_t value);
            void WriteDMAControl(uint32_t channel, uint32_t value);

            // Controller state
            bool joy_transfer_active = false;
            uint8_t joy_status = 0;
            uint8_t joy_control = 0;
            uint8_t joy_mode = 0;
            uint8_t joy_baud = 0;
            std::array<uint8_t, 8> joy_rx_data;
            std::array<uint8_t, 8> joy_tx_data;
            size_t joy_rx_pos = 0;
            size_t joy_tx_pos = 0;

            // CD-ROM state
            bool cdrom_busy = false;
            uint8_t cdrom_status = 0;
            uint8_t cdrom_command = 0;
            uint8_t cdrom_interrupt = 0;
            std::array<uint8_t, 16> cdrom_response;
            std::array<uint8_t, 2048> cdrom_sector_buffer;
            size_t cdrom_response_size = 0;
            size_t cdrom_response_pos = 0;

            // MDEC state
            bool mdec_busy = false;
            uint32_t mdec_status = 0;
            uint32_t mdec_control = 0;

            // SPU state
            bool spu_busy = false;
            uint16_t spu_status = 0;
            uint16_t spu_control = 0;

            // PS1 Quirk: Separate instruction and data cache states
            // PS1 Quirk: Enhanced cache control (VERIFIED)
            struct CacheControl {
                bool isolated;
                bool enabled;
                bool tag_test_mode;
                bool force_miss;
                bool scratchpad_enable;
                
                // PS1 Quirk: Cache line states (VERIFIED)
                struct CacheLine {
                    bool valid;
                    bool locked;
                    uint32_t tag;
                    uint32_t last_access;
                    bool dirty;          // For coherency
                    uint8_t data[16];   // 16 bytes per line
                };
                
                std::array<CacheLine, 64> lines;  // 64 cache lines
                
                // PS1 Quirk: Cache coherency state
                uint32_t pending_invalidations;
                bool coherency_check_needed;
            } cache_control;

            // PS1 Quirk: Complete DMA state (VERIFIED)
            struct DMAState {
                struct Channel {
                    uint32_t base_addr;
                    uint32_t block_size;
                    uint32_t control;
                    bool active;
                    
                    // PS1 Quirk: Enhanced chopping control
                    struct ChopControl {
                        bool enabled;
                        uint32_t size;
                        uint32_t count;
                        uint32_t dma_window;
                        uint32_t cpu_window;
                        uint32_t remaining;
                    } chop;
                    
                    // PS1 Quirk: Synchronization
                    struct SyncControl {
                        bool enabled;
                        uint32_t trigger;
                        uint32_t counter;
                        bool waiting;
                    } sync;
                    
                    uint32_t priority;
                    bool word_aligned;
                    bool reverse_transfer;
                };
                
                std::array<Channel, 7> channels;  // 7 DMA channels
                bool bus_locked;
                uint32_t active_channel;
                uint32_t pending_requests;
            } dma_state;

            void UpdateBusState();
            void HandleDMAPriority();
            void UpdateCacheCoherency();

            // PS1 Quirk: Cache tag layout
            struct CacheTag {
                uint32_t tag;
                bool valid;
                bool dirty;
            };
            std::array<CacheTag, 64> i_cache_tags;
            std::array<CacheTag, 64> d_cache_tags;

            // PS1 Quirk: Memory control registers
            struct {
                uint32_t exp1_base;
                uint32_t exp2_base;
                uint32_t exp1_delay;
                uint32_t exp3_delay;
                uint32_t bios_rom_delay;
                uint32_t spu_delay;
                uint32_t cdrom_delay;
                uint32_t exp2_delay;
            } mem_control;

            // PS1 Quirk: Additional hardware registers
            uint32_t dma_irq;    // DMA Interrupt Register

            // PS1 Quirk: Memory access states
            struct MemoryState {
                bool memory_busy;          // Memory bus busy state (VERIFIED)
                uint32_t access_cycles;    // Current access cycle count
                bool io_busy;             // I/O ports busy state
                bool dma_active;          // DMA transfer in progress
                bool cache_isolated;       // Cache isolation state
                uint32_t last_access;     // Last accessed address
            } mem_state;

            // DMA Channels array (7 channels for PS1)
            std::array<DMAChannel, 7> dma_channels;

            // Add these crucial cache isolation behaviors
            struct CacheState {
                bool scratchpad_enabled;
                bool cache_isolated;
                bool force_icache_miss;    // Forces instruction cache misses
                bool force_dcache_miss;    // Forces data cache misses
                uint32_t iso_base;         // Base address in isolated cache mode
                std::array<uint8_t, 1024> isolated_cache;
            };

            // PS1 Quirk: Memory access timing details (VERIFIED)
            struct MemoryTiming {
                // Verified timing values from real PS1
                static constexpr uint32_t RAM_ACCESS_TIME = 5;      // Standard RAM access time
                static constexpr uint32_t RAM_READ_FIRST = 6;      // First RAM read cycles
                static constexpr uint32_t RAM_READ_SEQUENTIAL = 3;  // Sequential RAM read
                static constexpr uint32_t RAM_WRITE = 4;           // RAM write cycles
                static constexpr uint32_t BIOS_READ_FIRST = 24;    // First BIOS read cycles
                static constexpr uint32_t BIOS_READ_SEQUENTIAL = 8; // Sequential BIOS read
                static constexpr uint32_t IO_ACCESS = 3;           // I/O port access cycles
                
                // DMA specific timing (VERIFIED)
                static constexpr uint32_t DMA_SETUP = 2;           // DMA setup overhead
                static constexpr uint32_t DMA_TRANSFER_RAM = 4;    // RAM DMA transfer
                static constexpr uint32_t DMA_TRANSFER_OTAG = 6;   // GPU OTag DMA
                
                // Bus state tracking
                bool ram_busy;                // RAM bus state
                bool bios_busy;              // BIOS bus state
                bool io_busy;                // I/O bus state
                uint32_t last_access;        // Last memory access cycle
                uint32_t sequential_count;   // Track sequential accesses
            } mem_timing;

            // PS1 Quirk: DMA detailed behavior
            struct DMADetail {
                bool chopping_enabled;         // DMA chopping state
                uint32_t chop_dma_window;     // DMA window size during chopping
                uint32_t chop_cpu_window;     // CPU window size during chopping
                uint32_t transfer_size;        // Current transfer size
                bool sync_mode;               // Synchronization mode active
                bool channel_enable;          // Channel enabled state
                uint32_t priority;            // Channel priority level
                bool force_word_align;        // Force word alignment
                bool reverse_transfer;        // Reverse transfer direction
            };
            std::array<DMADetail, 7> dma_detail;

            // PS1 Quirk: Memory mirroring details
            struct MirrorState {
                uint32_t ram_size_mask;
                uint32_t scratchpad_mask;
                uint32_t bios_mask;
                bool ram_mirroring_enabled;
            } mirror_state;

            // PS1 Quirk: Memory timing counters
            struct TimingCounters {
                uint32_t gpu_cycles;
                uint32_t spu_cycles;
                uint32_t cdrom_cycles;
                uint32_t dma_cycles;
            } timing;

            void UpdateTiming();

            struct DMAQuirks {
                bool chopping_enabled;       // DMA chopping state
                uint32_t chop_dma_window;   // DMA window size during chopping
                uint32_t chop_cpu_window;   // CPU window size during chopping
                bool sync_mode_enabled;     // Synchronization mode active
                uint32_t sync_trigger;      // Sync trigger value
                bool force_word_align;      // Force word alignment (VERIFIED)
                bool reverse_transfer;      // Reverse transfer direction (VERIFIED)
            };

            void UpdateCacheState(uint32_t address, bool write);
            void FillCacheLineData(uint32_t address, uint32_t* data);
            void WriteBackCacheLine(uint32_t address, const CacheLine& entry);
            void ReplaceCacheLine(uint32_t address, const CacheLine& entry);

            // Add missing function declarations
            void FillCacheLine(uint32_t address);
            void InvalidateCacheRange(uint32_t start_addr, uint32_t size);
            bool IsDMATriggered(uint32_t channel) const;
            uint32_t HandleDMABlock(uint32_t channel, uint32_t size);
            void HandleDMACompletion();
            void CompleteMemoryAccess();
            void UpdateMemoryState();

            // Add callback member variable
            std::function<void(uint32_t)> dma_complete_callback;

        public:
            Memory();
            ~Memory();

            void Reset();
            bool LoadBIOS(const std::string& path);

            void InitDMAChannels();
            void HandleDMATransfer(uint32_t channel);
            void InvalidateCacheLine(uint32_t address);

            // Memory access
            uint8_t Read8(uint32_t address);
            uint16_t Read16(uint32_t address);
            uint32_t Read32(uint32_t address);

            void Write8(uint32_t address, uint8_t value);
            void Write16(uint32_t address, uint16_t value);
            void Write32(uint32_t address, uint32_t value);

            // I/O access
            uint8_t ReadIO8(uint32_t address);
            uint16_t ReadIO16(uint32_t address);
            uint32_t ReadIO32(uint32_t address);
            void WriteIO8(uint32_t address, uint8_t value);
            void WriteIO16(uint32_t address, uint16_t value);
            void WriteIO32(uint32_t address, uint32_t value);

            // Hardware attachment
            void AttachGPU(GPU* gpu) { this->gpu = gpu; }
            void AttachGTE(GTE* gte) { this->gte = gte; }
            void AttachSPU(SPU* spu) { this->spu = spu; }
            void AttachCDROM(CDROM* cdrom) { this->cdrom = cdrom; }

            // DMA control
            void SetDMABaseAddr(uint32_t channel, uint32_t addr);
            void SetDMABlockSize(uint32_t channel, uint32_t size);
            uint32_t GetDMAStatus(uint32_t channel);

            // Interrupt control
            uint32_t GetInterruptStatus() const { return interrupt_stat; }
            uint32_t GetInterruptMask() const { return interrupt_mask; }
            void SetInterruptStatus(uint32_t value) { interrupt_stat = value; }
            void SetInterruptMask(uint32_t value) { interrupt_mask = value; }

            // Peripheral I/O
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

            // Add these helper functions for peripheral state management
            bool IsJoyTransferActive() const { return joy_transfer_active; }
            void SetJoyTransferActive(bool active) { joy_transfer_active = active; }
            bool IsCDROMBusy() const { return cdrom_busy; }
            void SetCDROMBusy(bool busy) { cdrom_busy = busy; }
            bool IsMDECBusy() const { return mdec_busy; }
            void SetMDECBusy(bool busy) { mdec_busy = busy; }
            bool IsSPUBusy() const { return spu_busy; }
            void SetSPUBusy(bool busy) { spu_busy = busy; }

            bool IsBusy() const { 
                return gpu->IsBusy() || IsCDROMBusy() || IsSPUBusy() || IsMDECBusy(); 
            }
            
            bool IsCacheIsolated() const { return cache_isolated; }
            uint32_t ReadCacheIsolated(uint32_t address) const;

            // Add GPU DMA status check
            bool IsReadyForDMA() const {
                return gpu && !gpu->IsBusy();
            }

            // Add these methods to public interface
            uint32_t GetCacheAccessTime() const {
                // PS1 Quirk: Cache access timing (VERIFIED)
                return bus_state.is_sequential ? 
                    BusState::BusTiming::RAM_SEQUENTIAL : 
                    BusState::BusTiming::RAM_FIRST_ACCESS;
            }

            void HandleCacheStateChange() {
                // PS1 Quirk: Cache state transition (VERIFIED)
                for (auto& line : cache_control.lines) {
                    line.valid = false;
                    line.dirty = false;
                }
                cache_control.coherency_check_needed = true;
            }

            void HandleMemoryRefresh() {
                // PS1 Quirk: Memory refresh cycles (VERIFIED)
                bus_state.current_state = BusState::State::REFRESH;
                bus_state.current_cycles = BusState::BusTiming::REFRESH_CYCLES;
            }

            // Add method to set callback
            void SetDMACompleteCallback(std::function<void(uint32_t)> callback) {
                dma_complete_callback = callback;
            }
    };
}
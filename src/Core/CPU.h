#pragma once

#include <cstdint>
#include <string>
#include <cstring>
#include <iostream>
#include "Memory.h"
#include "GPU.h"
#include "GTE.h"

namespace PSX {
    class R3000A_CPU {
        private:
            typedef struct _CPU {
                uint32_t R0;                                        // ZR: Constant Zero
                uint32_t R1;                                        // AT: Reverse for the assembler
                uint32_t R2, R3;                                    // V0 - V1: Value for the results and expression evaluation
                uint32_t R4, R5, R6, R7;                            // A0 - A3: Argument registers
                uint32_t R8, R9, R10, R11, R12, R13, R14, R15;      // T0 - T7: Temporary registers
                uint32_t R16, R17, R18, R19, R20, R21, R22, R23;    // S0 - S7: Saved registers
                uint32_t R24, R25;                                  // T8 - T9: Saved temporary registers
                uint32_t R26, R27;                                  // K0 - K1: Kernel space registers
                uint32_t R28;                                       // GP: Global pointer
                uint32_t R29;                                       // SP: Stack pointer
                uint32_t R30;                                       // FP: Frame pointer
                uint32_t R31;                                       // RA: Return address
                uint32_t HI;                                        // HI: High result register
                uint32_t LO;                                        // LO: Low result register
                uint32_t PC;                                        // PC: Program counter
            } CPU, *PCPU;

            union Instruction {
                uint32_t raw;

                // R-Type format (Register)
                struct {
                    uint32_t funct:6;                               // Function code
                    uint32_t shamt:5;                               // Shift amount
                    uint32_t rd:5;                                  // Destination register
                    uint32_t rt:5;                                  // Source register
                    uint32_t rs:5;                                  // Source register
                    uint32_t opcode:6;                              // Opcode
                } r;

                // I-Type format (Immediate)
                struct {
                    uint32_t immediate:16;
                    uint32_t rt:5;
                    uint32_t rs:5;
                    uint32_t opcode:6;
                } i;

                // J-Type format (Jump)
                struct {
                    uint32_t target:26;
                    uint32_t opcode:6;
                } j;
            };

            PCPU cpu;

            // Internal CPU functions
            void Reset();
            uint32_t FetchInstruction();
            void DecodeAndExecute(uint32_t instruction);

            void ExecuteR(Instruction instruction);
            void ExecuteI(Instruction instruction);
            void ExecuteJ(Instruction instruction);

            void Op_ADD(uint8_t rd, uint8_t rs, uint8_t rt);
            void Op_ADDI(uint8_t rt, uint8_t rs, uint16_t immediate);
            void Op_AND(uint8_t rd, uint8_t rs, uint8_t rt);
            void Op_BEQ(uint8_t rs, uint8_t rt, uint16_t offset);
            void Op_J(uint32_t target);
            void Op_LW(uint8_t rt, uint8_t rs, uint16_t offset);
            void Op_SW(uint8_t rt, uint8_t rs, uint16_t offset);

            void Op_ADDU(uint8_t rd, uint8_t rs, uint8_t rt);
            void Op_ADDIU(uint8_t rt, uint8_t rs, uint16_t immediate);
            void Op_ANDI(uint8_t rt, uint8_t rs, uint16_t immediate);
            void Op_BNE(uint8_t rs, uint8_t rt, uint16_t offset);
            void Op_LUI(uint8_t rt, uint16_t immediate);
            void Op_NOR(uint8_t rd, uint8_t rs, uint8_t rt);
            void Op_OR(uint8_t rd, uint8_t rs, uint8_t rt);
            void Op_ORI(uint8_t rt, uint8_t rs, uint16_t immediate);
            void Op_SLT(uint8_t rd, uint8_t rs, uint8_t rt);
            void Op_SLTI(uint8_t rt, uint8_t rs, uint16_t immediate);
            void Op_SLL(uint8_t rd, uint8_t rt, uint8_t shamt);
            void Op_SRL(uint8_t rd, uint8_t rt, uint8_t shamt);
            void Op_SUB(uint8_t rd, uint8_t rs, uint8_t rt);
            void Op_SUBU(uint8_t rd, uint8_t rs, uint8_t rt);
            void Op_JR(uint8_t rs);

            void Op_SRA(uint8_t rd, uint8_t rt, uint8_t shamt);
            void Op_JALR(uint8_t rd, uint8_t rs);
            void Op_MULT(uint8_t rs, uint8_t rt);
            void Op_MULTU(uint8_t rs, uint8_t rt);
            void Op_DIV(uint8_t rs, uint8_t rt);
            void Op_DIVU(uint8_t rs, uint8_t rt);
            void Op_MFHI(uint8_t rd);
            void Op_MFLO(uint8_t rd);
            void Op_XOR(uint8_t rd, uint8_t rs, uint8_t rt);

            Memory* memory;
            GPU* gpu;
            GTE* gte;

            // Add interrupt handling
            void HandleInterrupt();
            void HandleException(uint32_t excode);
            bool CheckInterrupts();

            // Add missing CPU operations
            void Op_MTC0(uint8_t rt, uint8_t rd);  // Move To Coprocessor 0
            void Op_MFC0(uint8_t rt, uint8_t rd);  // Move From Coprocessor 0
            void Op_RFE();                         // Return From Exception
            void Op_SYSCALL();                     // System Call
            void Op_BREAK();                       // Break
            void Op_MTLO(uint8_t rt);
            void Op_MTHI(uint8_t rt);

            // Exception codes
            static constexpr uint32_t EXCEPTION_INT    = 0;   // Interrupt
            static constexpr uint32_t EXCEPTION_ADEL   = 4;   // Address error (load/fetch)
            static constexpr uint32_t EXCEPTION_ADES   = 5;   // Address error (store)
            static constexpr uint32_t EXCEPTION_IBE    = 6;   // Bus error (instruction fetch)
            static constexpr uint32_t EXCEPTION_DBE    = 7;   // Bus error (data load/store)
            static constexpr uint32_t EXCEPTION_SYSCALL = 8;  // System call
            static constexpr uint32_t EXCEPTION_BP     = 9;   // Breakpoint
            static constexpr uint32_t EXCEPTION_RI     = 10;  // Reserved instruction
            static constexpr uint32_t EXCEPTION_CPU    = 11;  // Coprocessor unusable
            static constexpr uint32_t EXCEPTION_OV     = 12;  // Arithmetic overflow

            // PS1 Quirk: Load delay slot handling (merged version)
            struct LoadDelay {
                uint8_t reg;
                uint32_t value;
                uint32_t old_value;
                bool active;
                bool is_load_linked;    // For LL/SC instructions
                uint32_t linked_address;
            } load_delay;

            // PS1 Quirk: Branch delay with prediction (merged version)
            struct BranchDelay {
                uint32_t target;
                bool active;
                bool predicted;
                uint32_t source_pc;
                bool is_likely;         // For branch likely instructions
                uint8_t branch_type;    // Track type of branch for timing
            } branch_delay;

            // PS1 Quirk: COP0 state with detailed fields (merged version)
            struct COP0State {
                uint32_t SR;           // Status Register
                uint32_t CAUSE;        // Cause Register
                uint32_t EPC;          // Exception Program Counter
                uint32_t PRID;         // Processor ID
                uint32_t BadVAddr;     // Bad Virtual Address
                uint32_t DCIC;         // Debug and Cache Isolation Control
                uint32_t BPC;          // Breakpoint PC
                uint32_t BDA;          // Breakpoint Data Address
                uint32_t JUMPDEST;     // Jump Destination
                uint32_t BDAM;         // Data Address Breakpoint Mask
                uint32_t BPCM;         // PC Breakpoint Mask
            } cop0;

            // PS1 Quirk: Pipeline state tracking
            struct Pipeline {
                uint32_t current_pc;
                uint32_t next_pc;
                bool branch_taken;
                bool delay_slot;
                bool cache_miss;
                uint32_t stall_cycles;
                bool in_exception;
                bool in_interrupt;
                uint32_t last_branch_target;
                uint8_t branch_likely_mode;
            } pipeline;

            // PS1 Quirk: Cache state tracking
            struct CacheState {
                bool icache_enabled;
                bool dcache_enabled;
                bool cache_isolated;
                uint32_t last_tag;
                bool last_hit;
                uint8_t refill_counter;
                uint32_t line_state[64];  // Track state of each cache line
                bool scratchpad_enabled;
                uint32_t cache_control;
            } cache_state;

            // Add these declarations
            void Op_LB(uint8_t rt, uint8_t rs, uint16_t offset);
            void Op_LBU(uint8_t rt, uint8_t rs, uint16_t offset);
            void Op_LHU(uint8_t rt, uint8_t rs, uint16_t offset);
            void Op_SH(uint8_t rt, uint8_t rs, uint16_t offset);
            void Op_BGTZ(uint8_t rs, uint16_t offset);

            // Add these member variables
            uint32_t current_instruction;
            
            // Add missing method declarations
            void ExecuteCOP0(uint32_t instruction);
            
            // Add missing operation declarations
            void Op_SB(uint8_t rt, uint8_t rs, uint16_t offset);
            void Op_JAL(uint32_t target);
            void Op_BLEZ(uint8_t rs, uint16_t offset);

            // COP0 helper functions
            uint32_t GetCOP0Register(uint8_t reg) const;
            void SetCOP0Register(uint8_t reg, uint32_t value);

            // PS1 Exception Vector addresses
            static constexpr uint32_t EXCEPTION_VECTOR_BEV0 = 0x80000080;
            static constexpr uint32_t EXCEPTION_VECTOR_BEV1 = 0xBFC00180;

            // PS1 Status Register bit positions
            static constexpr uint32_t SR_BEV = 22;  // Boot Exception Vector
            static constexpr uint32_t SR_EXL = 1;   // Exception Level
            static constexpr uint32_t SR_IE = 0;    // Interrupt Enable

            // PS1 Cause Register bit positions
            static constexpr uint32_t CAUSE_BD = 31;  // Branch Delay slot

        public:
            R3000A_CPU();
            ~R3000A_CPU();

            // CPU Control functions
            void Initialize();
            void Step();
            void Run();
            void Stop();

            // Register access functions
            uint32_t GetRegister(uint8_t reg) const;
            void SetRegister(uint8_t reg, uint32_t value);
            uint32_t GetPC() const {
                return this->cpu->PC;
            }

            void SetPC(uint32_t PC) {
                this->cpu->PC = PC;
            }

            // Add new methods for handling PS1-specific behaviors
            void HandleCacheIsolation();
            void HandleScratchpadAccess();
            void UpdatePipeline();
            bool CheckBreakpoints();
            void HandleDelaySlot();
            void UpdateMemoryTiming();
            bool IsInDelaySlot() const { return pipeline.delay_slot; }
            bool IsCacheIsolated() const { return cache_state.cache_isolated; }
    };
}
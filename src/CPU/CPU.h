#pragma once

#include <cstdint>
#include <string>
#include <cstring>

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
    };
}
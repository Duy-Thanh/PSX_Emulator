#include "CPU.h"

#include "Memory.h"

namespace PSX {
    R3000A_CPU::R3000A_CPU() {
        this->cpu = new CPU();
        this->Initialize();
    }

    R3000A_CPU::~R3000A_CPU() {
        if (this->cpu) {
            delete this->cpu;
            this->cpu = NULL;
        }
    }

    void R3000A_CPU::Initialize() {
        this->Reset();
    }

    void R3000A_CPU::Reset() {
        std::memset(this->cpu, 0, sizeof(CPU));

        this->cpu->PC = 0xBFC00000;

        this->cpu->R0 = 0;
    }

    uint32_t R3000A_CPU::GetRegister(uint8_t reg) const {
        if (reg >= 32) return 0;

        return ((uint32_t*)this->cpu)[reg];
    }

    void R3000A_CPU::SetRegister(uint8_t reg, uint32_t value) {
        if (reg >= 32) return;
        if (reg == 0) return;

        ((uint32_t*)this->cpu)[reg] = value;
    }

    void R3000A_CPU::Step() {
        uint32_t instruction = this->FetchInstruction();
        this->DecodeAndExecute(instruction);

        this->cpu->PC += 4;
    }

    uint32_t R3000A_CPU::FetchInstruction() {

    }

    void R3000A_CPU::DecodeAndExecute(uint32_t instruction) {
        Instruction inst;
        inst.raw = instruction;

        switch (inst.r.opcode) {
            case 0x00:
                this->ExecuteR(inst);
                break;
            case 0x08:
                this->ExecuteI(inst);
                break;
            case 0x02:
            case 0x03:
                this->ExecuteJ(inst);
                break;

            default:
                this->ExecuteI(inst);
                break;
        }
    }

    void R3000A_CPU::ExecuteR(Instruction instruction) {
        switch (instruction.r.funct) {
            case 0x20:
                this->Op_ADD(instruction.r.rd, instruction.r.rs, instruction.r.rt);
                break;
            case 0x24:
                this->Op_AND(instruction.r.rd, instruction.r.rs, instruction.r.rt);
                break;
        }
    }

    void R3000A_CPU::ExecuteI(Instruction instruction) {
        switch (instruction.i.opcode) {
            case 0x08:
                this->Op_ADDI(instruction.i.rt, instruction.i.rs, instruction.i.immediate);
                break;
            case 0x04:
                this->Op_BEQ(instruction.i.rs, instruction.i.rt, instruction.i.immediate);
                break;
            case 0x23:
                this->Op_LW(instruction.i.rt, instruction.i.rs, instruction.i.immediate);
                break;
            case 0x2B:
                this->Op_SW(instruction.i.rt, instruction.i.rs, instruction.i.immediate);
                break;
        }
    }

    void R3000A_CPU::ExecuteJ(Instruction instruction) {
        switch (instruction.j.opcode) {
            case 0x02:
                this->Op_J(instruction.j.target);
                break;
        }
    }

    // INSTRUCTION FUNCTIONS

    // Implement some basic instructions
    void R3000A_CPU::Op_ADD(uint8_t rd, uint8_t rs, uint8_t rt) {
        uint32_t result = this->GetRegister(rs) + this->GetRegister(rt);
        this->SetRegister(rd, result);
    }

    void R3000A_CPU::Op_ADDI(uint8_t rt, uint8_t rs, uint16_t imm) {
        // Sign extend immediate value
        int32_t signExtImm = (int16_t)imm;
        uint32_t result = GetRegister(rs) + signExtImm;
        SetRegister(rt, result);
    }

    void R3000A_CPU::Op_AND(uint8_t rd, uint8_t rs, uint8_t rt) {
        uint32_t result = this->GetRegister(rs) & this->GetRegister(rt);
        this->SetRegister(rd, result);
    }

    void R3000A_CPU::Op_BEQ(uint8_t rs, uint8_t rt, uint16_t offset) {
        if (this->GetRegister(rs) == this->GetRegister(rt)) {
            // Branch target = PC + 4 + (offset << 2)
            int32_t signExtOffset = (int16_t)offset;
            this->cpu->PC += (signExtOffset << 2);
        }
    }

    void R3000A_CPU::Op_J(uint32_t target) {
        // Jump target = (PC & 0xF0000000) | (target << 2)
        this->cpu->PC = (this->cpu->PC & 0xF0000000) | (target << 2);
    }

    void R3000A_CPU::Op_LW(uint8_t rt, uint8_t rs, uint16_t offset) {
        // TODO: Implement memory system
        // uint32_t address = GetRegister(rs) + (int16_t)offset;
        // SetRegister(rt, ReadMemory32(address));
    }

    void R3000A_CPU::Op_SW(uint8_t rt, uint8_t rs, uint16_t offset) {
        // TODO: Implement memory system
        // uint32_t address = this->GetRegister(rs) + (int16_t)offset;
        // this->WriteMemory32(address, this->GetRegister(rt));
    }

    void R3000A_CPU::Run() {
        while (true) {
            this->Step();
        }
    }

    void R3000A_CPU::Stop() {

    }
}
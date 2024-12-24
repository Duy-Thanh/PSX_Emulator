#include "CPU.h"

#include "Memory.h"

namespace PSX {
    R3000A_CPU::R3000A_CPU() {
        this->cpu = new CPU();
        this->memory = new Memory();
        this->gpu = new GPU();
        this->gte = new GTE();
        
        // Connect components
        this->memory->AttachGPU(this->gpu);
        this->memory->AttachGTE(this->gte);
        
        this->Initialize();
    }

    R3000A_CPU::~R3000A_CPU() {
        if (this->cpu) {
            delete this->cpu;
            this->cpu = nullptr;
        }
        if (this->memory) {
            delete this->memory;
            this->memory = nullptr;
        }
        if (this->gpu) {
            delete this->gpu;
            this->gpu = nullptr;
        }
        if (this->gte) {
            delete this->gte;
            this->gte = nullptr;
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
        // Save current PC for branch delay slot handling
        uint32_t current_pc = cpu->PC;
        
        // Fetch instruction from memory
        uint32_t instruction = FetchInstruction();
        
        // Increment PC before execution (important for proper branch delay slot handling)
        cpu->PC += 4;
        
        // Decode and execute instruction
        DecodeAndExecute(instruction);
    }

    uint32_t R3000A_CPU::FetchInstruction() {
        return memory->Read32(cpu->PC);
    }

    void R3000A_CPU::DecodeAndExecute(uint32_t instruction) {
        Instruction instr;
        instr.raw = instruction;
        
        // Decode based on opcode
        switch (instr.r.opcode) {
            case 0x00:  // SPECIAL
                ExecuteR(instr);
                break;
            
            case 0x02:  // J
                Op_J(instr.j.target);
                break;
            
            case 0x04:  // BEQ
                Op_BEQ(instr.i.rs, instr.i.rt, instr.i.immediate);
                break;
            
            case 0x05:  // BNE
                Op_BNE(instr.i.rs, instr.i.rt, instr.i.immediate);
                break;
            
            case 0x08:  // ADDI
                Op_ADDI(instr.i.rt, instr.i.rs, instr.i.immediate);
                break;
            
            case 0x09:  // ADDIU
                Op_ADDIU(instr.i.rt, instr.i.rs, instr.i.immediate);
                break;
            
            case 0x0C:  // ANDI
                Op_ANDI(instr.i.rt, instr.i.rs, instr.i.immediate);
                break;
            
            case 0x0D:  // ORI
                Op_ORI(instr.i.rt, instr.i.rs, instr.i.immediate);
                break;
            
            case 0x0F:  // LUI
                Op_LUI(instr.i.rt, instr.i.immediate);
                break;
            
            case 0x23:  // LW
                Op_LW(instr.i.rt, instr.i.rs, instr.i.immediate);
                break;
            
            case 0x2B:  // SW
                Op_SW(instr.i.rt, instr.i.rs, instr.i.immediate);
                break;
            
            default:
                std::cerr << "Unhandled opcode: 0x" << std::hex << instr.r.opcode << std::endl;
                break;
        }
    }

    void R3000A_CPU::ExecuteR(Instruction instr) {
        switch (instr.r.funct) {
            case 0x00:  // SLL
                Op_SLL(instr.r.rd, instr.r.rt, instr.r.shamt);
                break;
            
            case 0x02:  // SRL
                Op_SRL(instr.r.rd, instr.r.rt, instr.r.shamt);
                break;
            
            case 0x08:  // JR
                Op_JR(instr.r.rs);
                break;
            
            case 0x20:  // ADD
                Op_ADD(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            
            case 0x21:  // ADDU
                Op_ADDU(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            
            case 0x24:  // AND
                Op_AND(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            
            case 0x27:  // NOR
                Op_NOR(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            
            case 0x2A:  // SLT
                Op_SLT(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            
            case 0x03:  // SRA - Shift Right Arithmetic
                Op_SRA(instr.r.rd, instr.r.rt, instr.r.shamt);
                break;
            
            case 0x09:  // JALR - Jump And Link Register
                Op_JALR(instr.r.rd, instr.r.rs);
                break;
            
            case 0x22:  // SUB - Subtract
                Op_SUB(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            
            case 0x23:  // SUBU - Subtract Unsigned
                Op_SUBU(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            
            case 0x25:  // OR - Logical OR
                Op_OR(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            
            case 0x26:  // XOR - Logical XOR
                Op_XOR(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            
            case 0x18:  // MULT - Multiply
                Op_MULT(instr.r.rs, instr.r.rt);
                break;
            
            case 0x19:  // MULTU - Multiply Unsigned
                Op_MULTU(instr.r.rs, instr.r.rt);
                break;
            
            case 0x1A:  // DIV - Divide
                Op_DIV(instr.r.rs, instr.r.rt);
                break;
            
            case 0x1B:  // DIVU - Divide Unsigned
                Op_DIVU(instr.r.rs, instr.r.rt);
                break;
            
            case 0x10:  // MFHI - Move From HI
                Op_MFHI(instr.r.rd);
                break;
            
            case 0x12:  // MFLO - Move From LO
                Op_MFLO(instr.r.rd);
                break;
            
            default:
                std::cerr << "Unhandled R-type function: 0x" << std::hex << instr.r.funct << std::endl;
                break;
        }
    }

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
        if (GetRegister(rs) == GetRegister(rt)) {
            int32_t signExtOffset = (int16_t)offset;
            // Branch target = PC + (offset << 2)
            // -4 to account for the PC increment in Step()
            cpu->PC += (signExtOffset << 2) - 4;
        }
    }

    void R3000A_CPU::Op_J(uint32_t target) {
        // Jump target = (PC & 0xF0000000) | (target << 2)
        uint32_t jump_addr = (cpu->PC & 0xF0000000) | (target << 2);
        // Set next PC after delay slot
        cpu->PC = jump_addr - 4;  // -4 because PC will be incremented in next Step()
    }

    void R3000A_CPU::Op_LW(uint8_t rt, uint8_t rs, uint16_t offset) {
        uint32_t address = GetRegister(rs) + (int16_t)offset;
        SetRegister(rt, this->memory->Read32(address));
    }

    void R3000A_CPU::Op_SW(uint8_t rt, uint8_t rs, uint16_t offset) {
        uint32_t address = GetRegister(rs) + (int16_t)offset;
        this->memory->Write32(address, GetRegister(rt));
    }

    void R3000A_CPU::Run() {
        while (true) {
            this->Step();
        }
    }

    void R3000A_CPU::Stop() {

    }

    void R3000A_CPU::Op_ADDU(uint8_t rd, uint8_t rs, uint8_t rt) {
        uint32_t result = GetRegister(rs) + GetRegister(rt);
        SetRegister(rd, result);
    }

    void R3000A_CPU::Op_ADDIU(uint8_t rt, uint8_t rs, uint16_t imm) {
        uint32_t result = GetRegister(rs) + (uint32_t)(int32_t)(int16_t)imm;
        SetRegister(rt, result);
    }

    void R3000A_CPU::Op_ANDI(uint8_t rt, uint8_t rs, uint16_t imm) {
        uint32_t result = GetRegister(rs) & imm;
        SetRegister(rt, result);
    }

    void R3000A_CPU::Op_BNE(uint8_t rs, uint8_t rt, uint16_t offset) {
        if (GetRegister(rs) != GetRegister(rt)) {
            int32_t signExtOffset = (int16_t)offset;
            // Branch target = PC + (offset << 2)
            // -4 to account for the PC increment in Step()
            cpu->PC += (signExtOffset << 2) - 4;
        }
    }

    void R3000A_CPU::Op_LUI(uint8_t rt, uint16_t imm) {
        SetRegister(rt, imm << 16);
    }

    void R3000A_CPU::Op_NOR(uint8_t rd, uint8_t rs, uint8_t rt) {
        uint32_t result = ~(GetRegister(rs) | GetRegister(rt));
        SetRegister(rd, result);
    }

    void R3000A_CPU::Op_OR(uint8_t rd, uint8_t rs, uint8_t rt) {
        uint32_t result = GetRegister(rs) | GetRegister(rt);
        SetRegister(rd, result);
    }

    void R3000A_CPU::Op_ORI(uint8_t rt, uint8_t rs, uint16_t imm) {
        uint32_t result = GetRegister(rs) | imm;
        SetRegister(rt, result);
    }

    void R3000A_CPU::Op_SLT(uint8_t rd, uint8_t rs, uint8_t rt) {
        int32_t s = (int32_t)GetRegister(rs);
        int32_t t = (int32_t)GetRegister(rt);
        SetRegister(rd, s < t ? 1 : 0);
    }

    void R3000A_CPU::Op_SLTI(uint8_t rt, uint8_t rs, uint16_t imm) {
        int32_t s = (int32_t)GetRegister(rs);
        int32_t i = (int16_t)imm;
        SetRegister(rt, s < i ? 1 : 0);
    }

    void R3000A_CPU::Op_SLL(uint8_t rd, uint8_t rt, uint8_t shamt) {
        uint32_t result = GetRegister(rt) << shamt;
        SetRegister(rd, result);
    }

    void R3000A_CPU::Op_SRL(uint8_t rd, uint8_t rt, uint8_t shamt) {
        uint32_t result = GetRegister(rt) >> shamt;
        SetRegister(rd, result);
    }

    void R3000A_CPU::Op_SUB(uint8_t rd, uint8_t rs, uint8_t rt) {
        uint32_t result = GetRegister(rs) - GetRegister(rt);
        SetRegister(rd, result);
    }

    void R3000A_CPU::Op_SUBU(uint8_t rd, uint8_t rs, uint8_t rt) {
        uint32_t result = GetRegister(rs) - GetRegister(rt);
        SetRegister(rd, result);
    }

    void R3000A_CPU::Op_JR(uint8_t rs) {
        uint32_t jump_addr = GetRegister(rs);
        // Set next PC after delay slot
        cpu->PC = jump_addr - 4;  // -4 because PC will be incremented in next Step()
    }

    void R3000A_CPU::Op_SRA(uint8_t rd, uint8_t rt, uint8_t shamt) {
        int32_t value = (int32_t)GetRegister(rt);
        SetRegister(rd, value >> shamt);
    }

    void R3000A_CPU::Op_JALR(uint8_t rd, uint8_t rs) {
        uint32_t return_addr = cpu->PC + 4;
        uint32_t jump_addr = GetRegister(rs);
        SetRegister(rd, return_addr);
        cpu->PC = jump_addr - 4;  // -4 because PC will be incremented in next Step()
    }

    void R3000A_CPU::Op_MULT(uint8_t rs, uint8_t rt) {
        int64_t result = (int64_t)(int32_t)GetRegister(rs) * (int64_t)(int32_t)GetRegister(rt);
        cpu->LO = (uint32_t)result;
        cpu->HI = (uint32_t)(result >> 32);
    }

    void R3000A_CPU::Op_DIV(uint8_t rs, uint8_t rt) {
        int32_t dividend = (int32_t)GetRegister(rs);
        int32_t divisor = (int32_t)GetRegister(rt);
        
        if (divisor != 0) {
            cpu->LO = dividend / divisor;
            cpu->HI = dividend % divisor;
        }
    }
}
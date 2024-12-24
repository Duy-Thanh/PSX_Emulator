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
        cpu = new CPU();
        Reset();
    }

    void R3000A_CPU::Reset() {
        memset(cpu, 0, sizeof(CPU));
        cpu->PC = 0xBFC00000;  // BIOS entry point
        cop0.SR = 0;
        cop0.CAUSE = 0;
        cop0.EPC = 0;
        cop0.PRID = 0x00000002;  // R3000A processor
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
        // PS1 Quirk: Handle load delay slots
        uint32_t old_load_reg = load_delay.reg;
        uint32_t old_load_value = load_delay.value;
        bool had_load = load_delay.active;
        
        load_delay.active = false;  // Clear for next instruction
        
        // PS1 Quirk: Save current PC for exception handling
        uint32_t current_pc = cpu->PC;
        
        // Fetch and execute next instruction
        uint32_t instr = FetchInstruction();
        
        // PS1 Quirk: Handle exceptions in delay slots
        bool in_delay_slot = branch_delay.active;
        
        try {
            DecodeAndExecute(instr);
        } catch (const std::exception&) {
            // PS1 Quirk: Exception in delay slot handling
            if (in_delay_slot) {
                cop0.EPC = current_pc - 4;  // Point to branch instruction
                cop0.CAUSE |= (1 << 31);    // Set BD (Branch Delay) bit
            } else {
                cop0.EPC = current_pc;
            }
            throw;  // Re-throw the exception
        }
        
        // PS1 Quirk: Apply load delay after executing delay slot
        if (had_load) {
            SetRegister(old_load_reg, old_load_value);
        }
        
        // PS1 Quirk: Handle branch delay slots
        if (branch_delay.active) {
            cpu->PC = branch_delay.target;
            branch_delay.active = false;
        } else {
            cpu->PC += 4;
        }
    }

    uint32_t R3000A_CPU::FetchInstruction() {
        return memory->Read32(cpu->PC);
    }

    void R3000A_CPU::DecodeAndExecute(uint32_t instruction) {
        // PS1 Quirk: NOP instruction (0x00000000) must be handled specially
        if (instruction == 0) {
            return;  // True NOP, do nothing
        }
        
        // PS1 Quirk: Reserved instruction exception
        if ((instruction & 0xFC00003F) == 0x0C00003F) {
            HandleException(EXCEPTION_RI);
            return;
        }
        
        Instruction instr;
        instr.raw = instruction;
        
        // PS1 specific: Coprocessor instructions when COP0 is disabled still execute!
        // This is a real quirk of the PS1's R3000A
        if ((instr.r.opcode >= 0x10 && instr.r.opcode <= 0x13) && 
            !(cop0.SR & 0x10000000)) {
            // Don't generate exception, let it execute anyway!
        }

        switch (instr.r.opcode) {
            case 0x00:  // SPECIAL
                ExecuteR(instr);
                break;
            
            case 0x10:  // COP0
                if (instr.r.rs == 0x00) {  // MFC0
                    Op_MFC0(instr.r.rt, instr.r.rd);
                } else if (instr.r.rs == 0x04) {  // MTC0
                    Op_MTC0(instr.r.rt, instr.r.rd);
                } else if (instr.r.funct == 0x10) {  // RFE
                    Op_RFE();
                }
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
            case 0x03:  // SRA
                Op_SRA(instr.r.rd, instr.r.rt, instr.r.shamt);
                break;
            case 0x08:  // JR
                Op_JR(instr.r.rs);
                break;
            case 0x09:  // JALR
                Op_JALR(instr.r.rd, instr.r.rs);
                break;
            case 0x18:  // MULT
                Op_MULT(instr.r.rs, instr.r.rt);
                break;
            case 0x19:  // MULTU
                Op_MULTU(instr.r.rs, instr.r.rt);
                break;
            case 0x1A:  // DIV
                Op_DIV(instr.r.rs, instr.r.rt);
                break;
            case 0x1B:  // DIVU
                Op_DIVU(instr.r.rs, instr.r.rt);
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
            case 0x25:  // OR
                Op_OR(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            case 0x26:  // XOR
                Op_XOR(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            case 0x27:  // NOR
                Op_NOR(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            case 0x2A:  // SLT
                Op_SLT(instr.r.rd, instr.r.rs, instr.r.rt);
                break;
            default:
                HandleException(10);  // Reserved Instruction
                break;
        }
    }

    void R3000A_CPU::ExecuteI(Instruction instr) {
        switch (instr.i.opcode) {
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
            case 0x04:  // BEQ
                Op_BEQ(instr.i.rs, instr.i.rt, instr.i.immediate);
                break;
            case 0x05:  // BNE
                Op_BNE(instr.i.rs, instr.i.rt, instr.i.immediate);
                break;
            case 0x0A:  // SLTI
                Op_SLTI(instr.i.rt, instr.i.rs, instr.i.immediate);
                break;
            default:
                HandleException(10);  // Reserved Instruction
                break;
        }
    }

    void R3000A_CPU::Op_ADD(uint8_t rd, uint8_t rs, uint8_t rt) {
        // PS1 Quirk: Overflow checking
        int32_t s = (int32_t)GetRegister(rs);
        int32_t t = (int32_t)GetRegister(rt);
        int64_t result = (int64_t)s + (int64_t)t;
        
        if (result > INT32_MAX || result < INT32_MIN) {
            HandleException(EXCEPTION_OV);
            return;
        }
        
        SetRegister(rd, (uint32_t)result);
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
        // PS1 Quirk: Branch comparison uses values AFTER the delay slot
        bool will_branch = GetRegister(rs) == GetRegister(rt);
        
        if (will_branch) {
            int32_t signExtOffset = (int16_t)offset;
            // PS1 Quirk: Branch target calculation includes current PC
            branch_delay.target = cpu->PC + (signExtOffset << 2);
            branch_delay.active = true;
        }
        // PS1 Quirk: Next instruction is ALWAYS delay slot, even if branch not taken
    }

    void R3000A_CPU::Op_J(uint32_t target) {
        // PS1 quirk: Jump target calculation
        uint32_t jump_addr = (cpu->PC & 0xF0000000) | (target << 2);
        branch_delay.target = jump_addr;
        branch_delay.active = true;
    }

    void R3000A_CPU::Op_LW(uint8_t rt, uint8_t rs, uint16_t offset) {
        uint32_t address = GetRegister(rs) + (int16_t)offset;
        
        // PS1 Quirk: Load delay slot behavior
        load_delay.reg = rt;
        load_delay.old_value = GetRegister(rt);  // Save old value
        load_delay.value = memory->Read32(address);
        load_delay.active = true;
        
        // PS1 Quirk: Reading from load delay slot target register
        // returns the old value during the delay slot
    }

    void R3000A_CPU::Op_SW(uint8_t rt, uint8_t rs, uint16_t offset) {
        uint32_t address = GetRegister(rs) + (int16_t)offset;
        this->memory->Write32(address, GetRegister(rt));
    }

    void R3000A_CPU::Run() {
        while (true) {
            // Check for interrupts before executing next instruction
            if (CheckInterrupts()) {
                HandleInterrupt();
            }
            
            Step();
        }
    }

    void R3000A_CPU::Stop() {
        // Implementation for stopping CPU execution
        // Could set a flag that's checked in the Run loop
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
            // PS1 quirk: Branch target calculation includes current PC
            branch_delay.target = cpu->PC + (signExtOffset << 2);
            branch_delay.active = true;
        }
        // PS1 quirk: Even if branch not taken, next instruction is delay slot
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

    void R3000A_CPU::Op_SLTI(uint8_t rt, uint8_t rs, uint16_t immediate) {
        int32_t s = (int32_t)GetRegister(rs);
        int32_t imm = (int16_t)immediate;  // Sign extend
        SetRegister(rt, s < imm ? 1 : 0);
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
        int32_t n = (int32_t)GetRegister(rs);
        int32_t d = (int32_t)GetRegister(rt);
        
        // PS1 Quirk: Division by zero behavior
        if (d == 0) {
            cpu->HI = (uint32_t)n;
            cpu->LO = (n >= 0) ? 0xFFFFFFFF : 1;
            return;
        }
        
        // PS1 Quirk: Special case for INT_MIN / -1
        if (n == INT32_MIN && d == -1) {
            cpu->HI = 0;
            cpu->LO = INT32_MIN;  // Result remains INT_MIN
            return;
        }
        
        // Normal division
        cpu->HI = (uint32_t)(n % d);
        cpu->LO = (uint32_t)(n / d);
    }

    void R3000A_CPU::Op_MULTU(uint8_t rs, uint8_t rt) {
        uint64_t result = (uint64_t)GetRegister(rs) * (uint64_t)GetRegister(rt);
        cpu->HI = (result >> 32) & 0xFFFFFFFF;
        cpu->LO = result & 0xFFFFFFFF;
    }

    void R3000A_CPU::Op_DIVU(uint8_t rs, uint8_t rt) {
        uint32_t n = GetRegister(rs);
        uint32_t d = GetRegister(rt);
        
        if (d == 0) {
            // Division by zero
            cpu->HI = n;
            cpu->LO = 0xFFFFFFFF;
        } else {
            cpu->HI = n % d;
            cpu->LO = n / d;
        }
    }

    void R3000A_CPU::Op_MFHI(uint8_t rd) {
        SetRegister(rd, cpu->HI);
    }

    void R3000A_CPU::Op_MFLO(uint8_t rd) {
        SetRegister(rd, cpu->LO);
    }

    void R3000A_CPU::Op_XOR(uint8_t rd, uint8_t rs, uint8_t rt) {
        uint32_t result = GetRegister(rs) ^ GetRegister(rt);
        SetRegister(rd, result);
    }

    void R3000A_CPU::HandleInterrupt() {
        // Save current PC to EPC
        cop0.EPC = cpu->PC;
        
        // Update Status Register
        cop0.SR |= 0x2;  // Set EXL bit
        
        // Jump to interrupt vector
        cpu->PC = 0x80000080;
    }

    void R3000A_CPU::HandleException(uint32_t excode) {
        // PS1 quirk: Exception handling in branch delay slots
        if (branch_delay.active) {
            cop0.EPC = cpu->PC - 4;  // Point to branch instruction
            cop0.CAUSE |= (1 << 31); // Set BD (Branch Delay) bit
        } else {
            cop0.EPC = cpu->PC - 4;
        }

        // PS1 specific: Status register handling
        uint32_t mode = cop0.SR & 0x3F;  // Save current mode
        cop0.SR &= ~0x3F;                // Clear mode bits
        cop0.SR |= (mode << 2);          // Shift mode bits
        
        cop0.CAUSE = (cop0.CAUSE & ~0x7C) | (excode << 2);
        
        // PS1 quirk: Always jump to fixed exception vector
        cpu->PC = 0x80000080;
        
        // Clear delay slots
        load_delay.active = false;
        branch_delay.active = false;
    }

    bool R3000A_CPU::CheckInterrupts() {
        if (!(cop0.SR & 0x1)) return false;  // Interrupts disabled
        if (!(cop0.SR & 0x401)) return false;  // IE or IEc bits not set
        
        uint32_t pending = memory->GetInterruptStatus() & memory->GetInterruptMask();
        if (!pending) return false;
        
        cop0.CAUSE &= ~0xFF00;  // Clear pending interrupt bits
        cop0.CAUSE |= (pending << 8);  // Set new pending interrupts
        
        return true;
    }

    void R3000A_CPU::Op_MTC0(uint8_t rt, uint8_t rd) {
        uint32_t value = GetRegister(rt);
        
        switch (rd) {
            case 12:  // Status Register
                // PS1 Quirk: Only certain bits are writable
                cop0.SR = (cop0.SR & 0xF0000000) | (value & 0x0FFFFFFF);
                break;
            
            case 13:  // Cause Register
                // PS1 Quirk: Only software interrupt bits are writable
                cop0.CAUSE = (cop0.CAUSE & ~0x300) | (value & 0x300);
                break;
            
            default:
                // Other registers behave normally
                *(&cop0.SR + rd) = value;
        }
    }

    void R3000A_CPU::Op_MFC0(uint8_t rt, uint8_t rd) {
        uint32_t value = 0;
        switch (rd) {
            case 12: value = cop0.SR; break;
            case 13: value = cop0.CAUSE; break;
            case 14: value = cop0.EPC; break;
            case 15: value = cop0.PRID; break;
        }
        SetRegister(rt, value);
    }

    void R3000A_CPU::Op_SYSCALL() {
        HandleException(EXCEPTION_SYSCALL);
    }

    void R3000A_CPU::Op_BREAK() {
        HandleException(EXCEPTION_BP);
    }

    void R3000A_CPU::Op_MTHI(uint8_t rs) {
        cpu->HI = GetRegister(rs);
    }

    void R3000A_CPU::Op_MTLO(uint8_t rs) {
        cpu->LO = GetRegister(rs);
    }

    void R3000A_CPU::Op_RFE() {
        // Return from Exception
        // Restore interrupt enable bits
        uint32_t mode = cop0.SR & 0x3F;  // Current mode bits
        cop0.SR &= ~0xF;                 // Clear current mode bits
        cop0.SR |= (mode >> 2);          // Restore previous mode bits
    }

    void R3000A_CPU::Op_LB(uint8_t rt, uint8_t rs, uint16_t offset) {
        uint32_t addr = GetRegister(rs) + (int16_t)offset;
        
        // PS1 Quirk: Load delay slot behavior
        load_delay.reg = rt;
        load_delay.old_value = GetRegister(rt);
        // PS1 Quirk: Sign extension for byte loads
        load_delay.value = (int8_t)memory->Read8(addr);
        load_delay.active = true;
    }

    void R3000A_CPU::Op_LBU(uint8_t rt, uint8_t rs, uint16_t offset) {
        uint32_t addr = GetRegister(rs) + (int16_t)offset;
        
        // PS1 Quirk: Load delay slot behavior
        load_delay.reg = rt;
        load_delay.old_value = GetRegister(rt);
        load_delay.value = memory->Read8(addr);  // No sign extension
        load_delay.active = true;
    }

    void R3000A_CPU::Op_LHU(uint8_t rt, uint8_t rs, uint16_t offset) {
        uint32_t addr = GetRegister(rs) + (int16_t)offset;
        
        // PS1 Quirk: Unaligned halfword access
        if (addr & 1) {
            HandleException(EXCEPTION_ADEL);
            return;
        }
        
        // PS1 Quirk: Load delay slot behavior
        load_delay.reg = rt;
        load_delay.old_value = GetRegister(rt);
        load_delay.value = memory->Read16(addr);
        load_delay.active = true;
    }

    void R3000A_CPU::Op_SH(uint8_t rt, uint8_t rs, uint16_t offset) {
        uint32_t addr = GetRegister(rs) + (int16_t)offset;
        
        // PS1 Quirk: Unaligned halfword access
        if (addr & 1) {
            HandleException(EXCEPTION_ADES);
            return;
        }
        
        memory->Write16(addr, GetRegister(rt) & 0xFFFF);
    }

    void R3000A_CPU::Op_BGTZ(uint8_t rs, uint16_t offset) {
        // PS1 Quirk: Branch comparison uses current register value
        if ((int32_t)GetRegister(rs) > 0) {
            int32_t signExtOffset = (int16_t)offset;
            branch_delay.target = cpu->PC + (signExtOffset << 2);
            branch_delay.active = true;
        }
    }
}
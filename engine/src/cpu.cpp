#include "retroshelf/cpu.hpp"
#include "retroshelf/ps2_ee_memory_map.hpp"

#include <sstream>
#include <vector>

namespace retroshelf {
namespace {
// Implement sign-propagating right shift using unsigned operations so the
// interpreter does not depend on implementation-defined signed shifts.
std::uint64_t arithmetic_shift_right(std::uint64_t value, unsigned amount) {
    amount &= 0x3FU;
    if (amount == 0U) return value;
    std::uint64_t result = value >> amount;
    if ((value & (std::uint64_t{1} << 63U)) != 0U)
        result |= (~std::uint64_t{0}) << (64U - amount);
    return result;
}
} // namespace

Cpu::Cpu() { reset(); }
void Cpu::reset() {
    gpr_.fill(0); gpr_high_.fill(0); memory_.fill(0); active_ram_ = nullptr; active_ram_base_ = 0; active_ee_memory_ = nullptr; pc_ = 0; next_pc_ = 4;
    delay_slot_pending_ = false; delay_branch_pc_ = 0; cop0_regs_.fill(0); status_ = 0; cause_ = 0; epc_ = 0; bad_vaddr_ = 0; hi_ = 0; lo_ = 0;
}
std::uint64_t Cpu::reg(std::size_t index) const { return index < RegisterCount ? gpr_[index] : 0; }
void Cpu::set_reg(std::size_t index, std::uint64_t value) {
    if (index > 0 && index < RegisterCount) gpr_[index] = value;
    gpr_[0] = 0;
    gpr_high_[0] = 0;
}
std::uint64_t Cpu::reg_high(std::size_t index) const { return index < RegisterCount ? gpr_high_[index] : 0; }
void Cpu::set_reg_high(std::size_t index, std::uint64_t value) {
    if (index > 0 && index < RegisterCount) gpr_high_[index] = value;
    gpr_high_[0] = 0;
}
std::uint64_t Cpu::pc() const { return pc_; }
void Cpu::set_pc(std::uint64_t value) { pc_ = value; next_pc_ = value + 4; delay_slot_pending_ = false; }
std::uint32_t Cpu::cop0_status() const { return status_; }
std::uint32_t Cpu::cop0_cause() const { return cause_; }
std::uint64_t Cpu::cop0_epc() const { return epc_; }
std::uint64_t Cpu::cop0_bad_vaddr() const { return bad_vaddr_; }
bool Cpu::in_delay_slot() const { return delay_slot_pending_; }
std::uint64_t Cpu::next_pc() const { return next_pc_; }
std::uint64_t Cpu::hi() const { return hi_; }
std::uint64_t Cpu::lo() const { return lo_; }

void Cpu::raise_exception(std::uint32_t code, std::uint64_t fault_pc, bool in_delay_slot) {
    if ((status_ & StatusExl) == 0U) {
        if (in_delay_slot) { epc_ = delay_branch_pc_; cause_ |= CauseBd; }
        else { epc_ = fault_pc; cause_ &= ~CauseBd; }
    }
    cause_ = (cause_ & ~(0x1FU << 2U)) | (code << 2U);
    status_ |= StatusExl;
    pc_ = GeneralExceptionVector;
    next_pc_ = pc_ + 4;
    delay_slot_pending_ = false;
}

bool Cpu::step(std::uint32_t instruction, std::string* error) {
    const auto opcode = (instruction >> 26U) & 0x3FU;
    const auto rs = (instruction >> 21U) & 0x1FU;
    const auto rt = (instruction >> 16U) & 0x1FU;
    const auto rd = (instruction >> 11U) & 0x1FU;
    const auto funct = instruction & 0x3FU;
    const auto imm = static_cast<std::int16_t>(instruction & 0xFFFFU);
    const auto simm = static_cast<std::int64_t>(imm);
    const auto sreg = [this](std::size_t i) { return static_cast<std::int64_t>(reg(i)); };

    const std::uint64_t cur_pc = pc_;
    const bool in_delay = delay_slot_pending_;
    const std::uint64_t branch_base = pc_ + 4; // branch/jump targets are relative to the delay slot

    const bool is_jr = opcode == 0x00 && (funct == 0x08 || funct == 0x09);
    // REGIMM also contains immediate trap instructions (TGEI/TGEIU/TLTI/
    // TLTIU/TEQI/TNEI). Only its branch encodings create delay slots.
    const bool is_regimm_branch = opcode == 0x01 &&
        (rt == 0x00 || rt == 0x01 || rt == 0x02 || rt == 0x03 ||
         rt == 0x10 || rt == 0x11 || rt == 0x12 || rt == 0x13);
    const bool is_blez_family_branch =
        (opcode == 0x06U || opcode == 0x07U ||
         opcode == 0x16U || opcode == 0x17U) && rt == 0U;
    const bool is_branch = is_regimm_branch ||
                           (opcode >= 0x02 && opcode <= 0x05) ||
                           is_blez_family_branch ||
                           (opcode >= 0x14 && opcode <= 0x15) || is_jr;
    if (is_branch && in_delay) {
        if (error) *error = "Branch or jump inside a branch delay slot is not supported";
        return false;
    }

    // Advance: the next instruction comes from next_pc_ (which a branch may redirect).
    pc_ = next_pc_;
    next_pc_ = pc_ + 4;
    delay_slot_pending_ = false;

    const auto unsupported = [&]() {
        if (error) {
            std::ostringstream out;
            out << "Unsupported instruction opcode=0x" << std::hex << opcode << " funct=0x" << funct;
            *error = out.str();
        }
        raise_exception(ExcReservedInstruction, cur_pc, in_delay);
        return false;
    };
    // Schedule a branch: delay slot executes next, then control moves to target (if taken).
    const auto branch = [&](bool taken, std::uint64_t target) {
        delay_slot_pending_ = true;
        delay_branch_pc_ = cur_pc;
        if (taken) next_pc_ = target;
        return true;
    };
    // Branch-likely instructions annul the delay slot when the condition is false.
    const auto branch_likely = [&](bool taken, std::uint64_t target) {
        if (taken) return branch(true, target);
        pc_ = cur_pc + 8;
        next_pc_ = cur_pc + 12;
        delay_slot_pending_ = false;
        return true;
    };
    const std::uint64_t rel_target = branch_base + static_cast<std::uint64_t>(simm * 4);

    switch (opcode) {
    case 0x00:
        switch (funct) {
        case 0x0C: // SYSCALL
            if (error) *error = "EE SYSCALL exception";
            raise_exception(ExcSyscall, cur_pc, in_delay);
            return false;
        case 0x0D: // BREAK
            if (error) *error = "EE BREAK exception";
            raise_exception(ExcBreakpoint, cur_pc, in_delay);
            return false;
        case 0x0F: return true; // SYNC: ordering barrier; no concurrent EE model yet
        case 0x0A: if (reg(rt) == 0) set_reg(rd, reg(rs)); return true; // MOVZ
        case 0x0B: if (reg(rt) != 0) set_reg(rd, reg(rs)); return true; // MOVN
        case 0x10: set_reg(rd, hi_); return true; // MFHI
        case 0x11: hi_ = reg(rs); return true; // MTHI
        case 0x12: set_reg(rd, lo_); return true; // MFLO
        case 0x13: lo_ = reg(rs); return true; // MTLO
        case 0x18: { // MULT: signed 32-bit product into HI:LO
            const auto product = static_cast<std::int64_t>(static_cast<std::int32_t>(reg(rs))) *
                                 static_cast<std::int64_t>(static_cast<std::int32_t>(reg(rt)));
            const auto bits = static_cast<std::uint64_t>(product);
            // The 32-bit MULT instruction sign-extends each 32-bit
            // half into the EE's 64-bit HI/LO registers.
            lo_ = static_cast<std::uint64_t>(static_cast<std::int64_t>(
                static_cast<std::int32_t>(static_cast<std::uint32_t>(bits))));
            hi_ = static_cast<std::uint64_t>(static_cast<std::int64_t>(
                static_cast<std::int32_t>(static_cast<std::uint32_t>(bits >> 32U))));
            return true;
        }
        case 0x1A: { // DIV: signed 32-bit quotient in LO, remainder in HI
            const auto dividend = static_cast<std::int32_t>(reg(rs));
            const auto divisor = static_cast<std::int32_t>(reg(rt));
            std::int32_t quotient = 0;
            std::int32_t remainder = dividend;
            if (divisor == 0) {
                // Deterministic legacy-MIPS convention for divide by zero.
                quotient = dividend < 0 ? 1 : -1;
            } else if (dividend == INT32_MIN && divisor == -1) {
                quotient = INT32_MIN;
                remainder = 0;
            } else {
                quotient = dividend / divisor;
                remainder = dividend % divisor;
            }
            lo_ = static_cast<std::uint64_t>(static_cast<std::int64_t>(quotient));
            hi_ = static_cast<std::uint64_t>(static_cast<std::int64_t>(remainder));
            return true;
        }
        case 0x1B: { // DIVU: unsigned 32-bit quotient in LO, remainder in HI
            const auto dividend = static_cast<std::uint32_t>(reg(rs));
            const auto divisor = static_cast<std::uint32_t>(reg(rt));
            if (divisor == 0U) {
                lo_ = 0xFFFFFFFFU;
                hi_ = dividend;
            } else {
                lo_ = dividend / divisor;
                hi_ = dividend % divisor;
            }
            return true;
        }
        case 0x19: { // MULTU: unsigned 32-bit product into HI:LO
            const auto product = static_cast<std::uint64_t>(static_cast<std::uint32_t>(reg(rs))) *
                                 static_cast<std::uint64_t>(static_cast<std::uint32_t>(reg(rt)));
            lo_ = static_cast<std::uint32_t>(product);
            hi_ = static_cast<std::uint32_t>(product >> 32U);
            return true;
        }
        case 0x1C: // DMULT: signed 64-bit product into HI:LO
        case 0x1D: { // DMULTU: unsigned 64-bit product into HI:LO
            const std::uint64_t lhs = reg(rs);
            const std::uint64_t rhs = reg(rt);
            const std::uint64_t lhs_low = static_cast<std::uint32_t>(lhs);
            const std::uint64_t lhs_high = lhs >> 32U;
            const std::uint64_t rhs_low = static_cast<std::uint32_t>(rhs);
            const std::uint64_t rhs_high = rhs >> 32U;
            const std::uint64_t low_product = lhs_low * rhs_low;
            const std::uint64_t cross_a = lhs_high * rhs_low;
            const std::uint64_t cross_b = lhs_low * rhs_high;
            const std::uint64_t low_with_cross = low_product + (cross_a << 32U);
            const std::uint64_t low = low_with_cross + (cross_b << 32U);
            std::uint64_t high = lhs_high * rhs_high + (cross_a >> 32U) +
                                 (cross_b >> 32U) +
                                 (low_with_cross < low_product ? 1U : 0U) +
                                 (low < low_with_cross ? 1U : 0U);
            if (funct == 0x1C) {
                if ((lhs >> 63U) != 0U) high -= rhs;
                if ((rhs >> 63U) != 0U) high -= lhs;
            }
            lo_ = low;
            hi_ = high;
            return true;
        }
        case 0x1E: { // DDIV: signed 64-bit quotient in LO, remainder in HI
            const auto dividend = static_cast<std::int64_t>(reg(rs));
            const auto divisor = static_cast<std::int64_t>(reg(rt));
            if (divisor == 0) {
                lo_ = dividend < 0 ? 1U : UINT64_MAX;
                hi_ = static_cast<std::uint64_t>(dividend);
            } else if (dividend == INT64_MIN && divisor == -1) {
                lo_ = static_cast<std::uint64_t>(INT64_MIN);
                hi_ = 0;
            } else {
                lo_ = static_cast<std::uint64_t>(dividend / divisor);
                hi_ = static_cast<std::uint64_t>(dividend % divisor);
            }
            return true;
        }
        case 0x1F: { // DDIVU: unsigned 64-bit quotient in LO, remainder in HI
            const std::uint64_t dividend = reg(rs);
            const std::uint64_t divisor = reg(rt);
            if (divisor == 0U) {
                lo_ = UINT64_MAX;
                hi_ = dividend;
            } else {
                lo_ = dividend / divisor;
                hi_ = dividend % divisor;
            }
            return true;
        }
        case 0x20: { // ADD: signed 32-bit addition traps on overflow
            const auto lhs = static_cast<std::int64_t>(static_cast<std::int32_t>(reg(rs)));
            const auto rhs = static_cast<std::int64_t>(static_cast<std::int32_t>(reg(rt)));
            const auto sum = lhs + rhs;
            if (sum < INT32_MIN || sum > INT32_MAX) {
                if (error) *error = "Signed ADD overflow";
                raise_exception(ExcOverflow, cur_pc, in_delay);
                return false;
            }
            set_reg(rd, static_cast<std::uint64_t>(sum));
            return true;
        }
        case 0x21: { // ADDU: 32-bit result, sign-extended to the EE GPR width
            const auto result = static_cast<std::uint32_t>(reg(rs)) + static_cast<std::uint32_t>(reg(rt));
            set_reg(rd, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(result))));
            return true;
        }
        case 0x22: { // SUB: signed 32-bit subtraction traps on overflow
            const auto lhs = static_cast<std::int64_t>(static_cast<std::int32_t>(reg(rs)));
            const auto rhs = static_cast<std::int64_t>(static_cast<std::int32_t>(reg(rt)));
            const auto difference = lhs - rhs;
            if (difference < INT32_MIN || difference > INT32_MAX) {
                if (error) *error = "Signed SUB overflow";
                raise_exception(ExcOverflow, cur_pc, in_delay);
                return false;
            }
            set_reg(rd, static_cast<std::uint64_t>(difference));
            return true;
        }
        case 0x23: { // SUBU: 32-bit result, sign-extended to the EE GPR width
            const auto result = static_cast<std::uint32_t>(reg(rs)) - static_cast<std::uint32_t>(reg(rt));
            set_reg(rd, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(result))));
            return true;
        }
        case 0x2C: { // DADD: signed 64-bit addition with overflow trap
            const std::uint64_t lhs = reg(rs);
            const std::uint64_t rhs = reg(rt);
            const std::uint64_t sum = lhs + rhs;
            constexpr std::uint64_t SignBit = std::uint64_t{1} << 63U;
            if (((lhs ^ sum) & (rhs ^ sum) & SignBit) != 0U) {
                if (error) *error = "Signed DADD overflow";
                raise_exception(ExcOverflow, cur_pc, in_delay);
                return false;
            }
            set_reg(rd, sum);
            return true;
        }
        case 0x2D: set_reg(rd, reg(rs) + reg(rt)); return true; // DADDU (64-bit, non-trapping)
        case 0x2E: { // DSUB: signed 64-bit subtraction with overflow trap
            const std::uint64_t lhs = reg(rs);
            const std::uint64_t rhs = reg(rt);
            const std::uint64_t difference = lhs - rhs;
            constexpr std::uint64_t SignBit = std::uint64_t{1} << 63U;
            if (((lhs ^ rhs) & (lhs ^ difference) & SignBit) != 0U) {
                if (error) *error = "Signed DSUB overflow";
                raise_exception(ExcOverflow, cur_pc, in_delay);
                return false;
            }
            set_reg(rd, difference);
            return true;
        }
        case 0x2F: set_reg(rd, reg(rs) - reg(rt)); return true; // DSUBU (64-bit, non-trapping)
        case 0x24: set_reg(rd, reg(rs) & reg(rt)); return true; // AND
        case 0x25: set_reg(rd, reg(rs) | reg(rt)); return true; // OR
        case 0x26: set_reg(rd, reg(rs) ^ reg(rt)); return true; // XOR
        case 0x27: set_reg(rd, ~(reg(rs) | reg(rt))); return true; // NOR
        case 0x2A: set_reg(rd, sreg(rs) < sreg(rt) ? 1U : 0U); return true; // SLT
        case 0x2B: set_reg(rd, reg(rs) < reg(rt) ? 1U : 0U); return true; // SLTU
        case 0x00: set_reg(rd, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(static_cast<std::uint32_t>(reg(rt) << ((instruction >> 6U) & 0x1FU)))))); return true; // SLL/NOP
        case 0x02: set_reg(rd, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(static_cast<std::uint32_t>(reg(rt)) >> ((instruction >> 6U) & 0x1FU))))); return true; // SRL
        case 0x03: set_reg(rd, arithmetic_shift_right(static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(reg(rt)))), (instruction >> 6U) & 0x1FU)); return true; // SRA
        case 0x14: set_reg(rd, reg(rt) << (reg(rs) & 0x3FU)); return true; // DSLLV
        case 0x16: set_reg(rd, reg(rt) >> (reg(rs) & 0x3FU)); return true; // DSRLV
        case 0x17: set_reg(rd, arithmetic_shift_right(reg(rt), static_cast<unsigned>(reg(rs) & 0x3FU))); return true; // DSRAV
        case 0x38: set_reg(rd, reg(rt) << ((instruction >> 6U) & 0x1FU)); return true; // DSLL
        case 0x3A: set_reg(rd, reg(rt) >> ((instruction >> 6U) & 0x1FU)); return true; // DSRL
        case 0x3B: set_reg(rd, arithmetic_shift_right(reg(rt), (instruction >> 6U) & 0x1FU)); return true; // DSRA
        case 0x3C: set_reg(rd, reg(rt) << (((instruction >> 6U) & 0x1FU) + 32U)); return true; // DSLL32
        case 0x3E: set_reg(rd, reg(rt) >> (((instruction >> 6U) & 0x1FU) + 32U)); return true; // DSRL32
        case 0x3F: set_reg(rd, arithmetic_shift_right(reg(rt), ((instruction >> 6U) & 0x1FU) + 32U)); return true; // DSRA32
        case 0x04: set_reg(rd, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(static_cast<std::uint32_t>(reg(rt) << (reg(rs) & 0x1FU)))))); return true; // SLLV
        case 0x06: set_reg(rd, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(static_cast<std::uint32_t>(reg(rt)) >> (reg(rs) & 0x1FU))))); return true; // SRLV
        case 0x07: set_reg(rd, arithmetic_shift_right(static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(reg(rt)))), static_cast<unsigned>(reg(rs) & 0x1FU))); return true; // SRAV
        case 0x08: return branch(true, reg(rs)); // JR
        case 0x09: { // JALR
            const auto target = reg(rs); // read before writing rd (rd may equal rs)
            set_reg(rd, cur_pc + 8);
            return branch(true, target);
        }
        case 0x30: case 0x31: case 0x32: case 0x33: case 0x34: case 0x36: { // TGE/TGEU/TLT/TLTU/TEQ/TNE
            const bool trap = funct == 0x30 ? sreg(rs) >= sreg(rt) :
                              funct == 0x31 ? reg(rs) >= reg(rt) :
                              funct == 0x32 ? sreg(rs) < sreg(rt) :
                              funct == 0x33 ? reg(rs) < reg(rt) :
                              funct == 0x34 ? reg(rs) == reg(rt) : reg(rs) != reg(rt);
            if (!trap) return true;
            if (error) *error = "EE register trap exception";
            raise_exception(ExcTrap, cur_pc, in_delay);
            return false;
        }
        default: break;
        }
        break;
    case 0x1C: { // MMI0 packed 32-bit word arithmetic (PADDW / PSUBW).
        const unsigned sub_op = (instruction >> 6U) & 0x1FU;
        if (funct != 0x08U || (sub_op != 0U && sub_op != 1U &&
                               sub_op != 2U && sub_op != 4U && sub_op != 5U &&
                               sub_op != 6U && sub_op != 8U && sub_op != 9U &&
                               sub_op != 10U && sub_op != 12U &&
                               sub_op != 14U && sub_op != 16U)) return unsupported();
        const std::uint64_t lhs_low = reg(rs);
        const std::uint64_t lhs_high = reg_high(rs);
        const std::uint64_t rhs_low = reg(rt);
        const std::uint64_t rhs_high = reg_high(rt);
        const unsigned lane_bits = (sub_op == 8U || sub_op == 9U || sub_op == 10U || sub_op == 16U) ? 8U :
                                   (sub_op == 4U || sub_op == 5U || sub_op == 6U || sub_op == 14U) ? 16U : 32U;
        const bool compare_greater = sub_op == 2U || sub_op == 6U || sub_op == 10U;
        const bool compare_equal = sub_op == 12U || sub_op == 14U || sub_op == 16U;
        const std::uint64_t lane_mask = (std::uint64_t{1} << lane_bits) - 1U;
        const unsigned lane_count = 64U / lane_bits;
        const auto packed = [sub_op, lane_bits, lane_mask, lane_count, compare_greater, compare_equal](
                                std::uint64_t lhs, std::uint64_t rhs) {
            std::uint64_t result = 0;
            for (unsigned i = 0; i < lane_count; ++i) {
                const unsigned shift = i * lane_bits;
                const std::uint64_t a = (lhs >> shift) & lane_mask;
                const std::uint64_t b = (rhs >> shift) & lane_mask;
                std::uint64_t value = 0;
                if (compare_greater) {
                    const std::uint64_t sign_bit = std::uint64_t{1} << (lane_bits - 1U);
                    value = ((a ^ sign_bit) > (b ^ sign_bit)) ? lane_mask : 0U;
                } else if (compare_equal) {
                    value = (a == b) ? lane_mask : 0U;
                } else {
                    value = (sub_op == 0U || sub_op == 4U || sub_op == 8U)
                                ? (a + b) & lane_mask
                                : (a - b) & lane_mask;
                }
                result |= value << shift;
            }
            return result;
        };
        const std::uint64_t result_low = packed(lhs_low, rhs_low);
        const std::uint64_t result_high = packed(lhs_high, rhs_high);
        set_reg(rd, result_low);
        set_reg_high(rd, result_high);
        return true;
    }
    case 0x10: { // Minimal COP0 support: MFC0, MTC0 and ERET.
        const auto read_cop0 = [this](std::uint32_t index, std::uint64_t& value) {
            switch (index) {
            case 0: case 2: case 3: case 4: case 5: case 6: case 7:
            case 9: case 10: case 11: case 16: case 23: case 24:
            case 25: case 28: case 29: case 30:
                value = cop0_regs_[index]; return true;
            case 1: value = cop0_regs_[index]; return true; // Random is a placeholder until cycle timing is modelled.
            case 15: value = cop0_regs_[index]; return true; // PRId placeholder
            case 12: value = status_; return true; // Status
            case 13: value = cause_; return true;  // Cause
            case 8: value = bad_vaddr_; return true; // BadVAddr (read-only)
            case 14: value = epc_; return true;    // EPC
            default: return false;
            }
        };
        if (rs == 0x00) { // MFC0 rt, rd
            std::uint64_t value = 0;
            if (!read_cop0(rd, value)) return unsupported();
            set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(value))));
            return true;
        }
        if (rs == 0x04) { // MTC0 rt, rd
            switch (rd) {
            case 0: case 2: case 3: case 4: case 5: case 6: case 7:
            case 9: case 10: case 11: case 16: case 23: case 24:
            case 25: case 28: case 29: case 30:
                cop0_regs_[rd] = static_cast<std::uint32_t>(reg(rt)); return true;
            case 12: status_ = static_cast<std::uint32_t>(reg(rt)); return true;
            case 13: { // Only software interrupt-pending bits IP0/IP1 are writable in Cause.
                constexpr std::uint32_t SoftwareInterruptPending = 0x00000300U;
                cause_ = (cause_ & ~SoftwareInterruptPending) |
                         (static_cast<std::uint32_t>(reg(rt)) & SoftwareInterruptPending);
                return true;
            }
            case 14: epc_ = reg(rt); return true;
            default: return unsupported();
            }
        }
        if (rs == 0x10 && funct == 0x18) { // ERET: ERL/ErrorEPC takes priority over EXL/EPC.
            if ((status_ & StatusErl) != 0U) {
                pc_ = cop0_regs_[30]; // ErrorEPC
                next_pc_ = pc_ + 4;
                status_ &= ~StatusErl;
            } else {
                pc_ = epc_;
                next_pc_ = epc_ + 4;
                status_ &= ~StatusExl;
            }
            delay_slot_pending_ = false;
            return true;
        }
        return unsupported();
    }
    case 0x01: { // REGIMM branches and immediate trap instructions.
        if (rt == 0x08 || rt == 0x09 || rt == 0x0A ||
            rt == 0x0B || rt == 0x0C || rt == 0x0E) {
            const auto immediate_unsigned = static_cast<std::uint64_t>(simm);
            const bool trap = rt == 0x08 ? sreg(rs) >= simm :
                              rt == 0x09 ? reg(rs) >= immediate_unsigned :
                              rt == 0x0A ? sreg(rs) < simm :
                              rt == 0x0B ? reg(rs) < immediate_unsigned :
                              rt == 0x0C ? reg(rs) == immediate_unsigned :
                                           reg(rs) != immediate_unsigned;
            if (!trap) return true;
            if (error) *error = "EE immediate trap exception";
            raise_exception(ExcTrap, cur_pc, in_delay);
            return false;
        }
        if (rt != 0x00 && rt != 0x01 && rt != 0x02 && rt != 0x03 &&
            rt != 0x10 && rt != 0x11 && rt != 0x12 && rt != 0x13) return unsupported();
        const bool taken = (rt & 1U) ? sreg(rs) >= 0 : sreg(rs) < 0;
        const bool likely = (rt & 0x02U) != 0U;
        const bool link = (rt & 0x10U) != 0U;
        if (link && (!likely || taken)) set_reg(31, cur_pc + 8);
        return likely ? branch_likely(taken, rel_target) : branch(taken, rel_target);
    }
    case 0x02: case 0x03: { // J / JAL
        if (opcode == 0x03) set_reg(31, cur_pc + 8);
        return branch(true, (branch_base & ~0x0FFFFFFFULL) | (static_cast<std::uint64_t>(instruction & 0x03FFFFFFU) << 2U));
    }
    case 0x04: return branch(reg(rs) == reg(rt), rel_target); // BEQ
    case 0x05: return branch(reg(rs) != reg(rt), rel_target); // BNE
    case 0x06:
        if (rt != 0U) return unsupported();
        return branch(sreg(rs) <= 0, rel_target); // BLEZ
    case 0x07:
        if (rt != 0U) return unsupported();
        return branch(sreg(rs) > 0, rel_target); // BGTZ
    case 0x14: return branch_likely(reg(rs) == reg(rt), rel_target); // BEQL
    case 0x15: return branch_likely(reg(rs) != reg(rt), rel_target); // BNEL
    case 0x16:
        if (rt != 0U) return unsupported();
        return branch_likely(sreg(rs) <= 0, rel_target); // BLEZL
    case 0x17:
        if (rt != 0U) return unsupported();
        return branch_likely(sreg(rs) > 0, rel_target); // BGTZL
    case 0x18: { // DADDI: signed 64-bit addition with overflow exception
        const std::uint64_t lhs = reg(rs);
        const std::uint64_t rhs = static_cast<std::uint64_t>(simm);
        const std::uint64_t sum = lhs + rhs;
        constexpr std::uint64_t SignBit = std::uint64_t{1} << 63U;
        if (((lhs ^ sum) & (rhs ^ sum) & SignBit) != 0U) {
            if (error) *error = "Signed DADDI overflow";
            raise_exception(ExcOverflow, cur_pc, in_delay);
            return false;
        }
        set_reg(rt, sum);
        return true;
    }
    case 0x19: // DADDIU: 64-bit addition, no overflow exception
        set_reg(rt, reg(rs) + static_cast<std::uint64_t>(simm));
        return true;
    case 0x08: { // ADDI: signed 32-bit immediate addition traps on overflow
        const auto lhs = static_cast<std::int64_t>(static_cast<std::int32_t>(reg(rs)));
        const auto sum = lhs + simm;
        if (sum < INT32_MIN || sum > INT32_MAX) {
            if (error) *error = "Signed ADDI overflow";
            raise_exception(ExcOverflow, cur_pc, in_delay);
            return false;
        }
        set_reg(rt, static_cast<std::uint64_t>(sum));
        return true;
    }
    case 0x09: // ADDIU: 32-bit wrapping addition, result sign-extended to 64 bits
        set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(
                        static_cast<std::int32_t>(static_cast<std::uint32_t>(reg(rs)) +
                                                  static_cast<std::uint32_t>(simm)))));
        return true;
    case 0x0A: // SLTI
        set_reg(rt, sreg(rs) < simm ? 1U : 0U);
        return true;
    case 0x0B: // SLTIU (the immediate is sign-extended, then compared unsigned)
        set_reg(rt, reg(rs) < static_cast<std::uint64_t>(simm) ? 1U : 0U);
        return true;
    case 0x0C: // ANDI
        set_reg(rt, reg(rs) & static_cast<std::uint16_t>(instruction & 0xFFFFU));
        return true;
    case 0x0D: // ORI
        set_reg(rt, reg(rs) | static_cast<std::uint16_t>(instruction & 0xFFFFU));
        return true;
    case 0x0E: // XORI
        set_reg(rt, reg(rs) ^ static_cast<std::uint16_t>(instruction & 0xFFFFU));
        return true;
    case 0x0F: { // LUI: construct a 32-bit value and sign-extend to 64 bits
        const auto value = static_cast<std::uint32_t>((instruction & 0xFFFFU) << 16U);
        set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(value))));
        return true;
    }
    case 0x2F: return true; // CACHE: no data/instruction cache is modeled in this functional interpreter.
    case 0x33: return true; // PREF: prefetch hint; no effect without a modeled cache hierarchy.
    case 0x22: case 0x26: case 0x2A: case 0x2E: { // LWL/LWR/SWL/SWR (little-endian word merges)
        const bool store = opcode == 0x2A || opcode == 0x2E;
        const std::uint64_t address = reg(rs) + static_cast<std::uint64_t>(simm);
        const std::uint64_t aligned = address & ~std::uint64_t{3};
        const unsigned lane = static_cast<unsigned>(address & 3U);
        const auto fault = [&](const char* message) {
            if (error) *error = message;
            bad_vaddr_ = address;
            raise_exception(store ? ExcAddressErrorStore : ExcAddressErrorLoad, cur_pc, in_delay);
            return false;
        };
        if (active_ee_memory_ == nullptr)
            return fault("LWL/LWR/SWL/SWR require the mapped EE memory interface");
        if (aligned > 0xFFFFFFFFULL)
            return fault("EE RAM effective address exceeds 32-bit address space");
        std::uint64_t memory_value = 0;
        if (!active_ee_memory_->read(static_cast<std::uint32_t>(aligned), 4, memory_value, nullptr))
            return fault("Unmapped EE RAM word-merge access");
        const std::uint32_t mem = static_cast<std::uint32_t>(memory_value);
        const std::uint32_t old_rt = static_cast<std::uint32_t>(reg(rt));
        std::uint32_t result = old_rt;
        if (opcode == 0x22) { // LWL: memory bytes up to lane merge into the high end of rt
            const unsigned shift = (3U - lane) * 8U;
            const std::uint32_t mask = 0xFFFFFFFFU << shift;
            result = (old_rt & ~mask) | (mem << shift);
            set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(result))));
            return true;
        }
        if (opcode == 0x26) { // LWR: memory bytes from lane through the end merge into low end
            const unsigned shift = lane * 8U;
            const std::uint32_t mask = 0xFFFFFFFFU >> shift;
            result = (old_rt & ~mask) | (mem >> shift);
            set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(result))));
            return true;
        }
        if (opcode == 0x2A) { // SWL: store high-end rt bytes into memory bytes 0..lane
            const unsigned shift = (3U - lane) * 8U;
            const std::uint32_t mask = 0xFFFFFFFFU >> shift;
            result = (mem & ~mask) | (old_rt >> shift);
        } else { // SWR: store low-end rt bytes into memory bytes lane..3
            const unsigned shift = lane * 8U;
            const std::uint32_t mask = 0xFFFFFFFFU << shift;
            result = (mem & ~mask) | (old_rt << shift);
        }
        if (!active_ee_memory_->write(static_cast<std::uint32_t>(aligned), 4, result, nullptr))
            return fault("Unmapped EE RAM word-merge store");
        return true;
    }
    case 0x1A: case 0x1B: case 0x2C: case 0x2D: { // LDL/LDR/SDL/SDR (little-endian doubleword merges)
        const bool store = opcode == 0x2C || opcode == 0x2D;
        const std::uint64_t address = reg(rs) + static_cast<std::uint64_t>(simm);
        const std::uint64_t aligned = address & ~std::uint64_t{7};
        const unsigned lane = static_cast<unsigned>(address & 7U);
        const auto fault = [&](const char* message) {
            if (error) *error = message;
            bad_vaddr_ = address;
            raise_exception(store ? ExcAddressErrorStore : ExcAddressErrorLoad, cur_pc, in_delay);
            return false;
        };
        if (active_ee_memory_ == nullptr)
            return fault("LDL/LDR/SDL/SDR require the mapped EE memory interface");
        if (aligned > 0xFFFFFFFFULL)
            return fault("EE RAM doubleword-merge address exceeds 32-bit address space");
        std::uint64_t mem = 0;
        if (!active_ee_memory_->read(static_cast<std::uint32_t>(aligned), 8, mem, nullptr))
            return fault("Unmapped EE RAM doubleword-merge access");
        const std::uint64_t old_rt = reg(rt);
        std::uint64_t result = old_rt;
        if (opcode == 0x1A) { // LDL: merge memory toward the most-significant end
            const unsigned shift = (7U - lane) * 8U;
            const std::uint64_t mask = UINT64_MAX << shift;
            result = (old_rt & ~mask) | (mem << shift);
            set_reg(rt, result);
            return true;
        }
        if (opcode == 0x1B) { // LDR: merge memory toward the least-significant end
            const unsigned shift = lane * 8U;
            const std::uint64_t mask = UINT64_MAX >> shift;
            result = (old_rt & ~mask) | (mem >> shift);
            set_reg(rt, result);
            return true;
        }
        if (opcode == 0x2C) { // SDL: store most-significant register bytes into low memory lanes
            const unsigned shift = (7U - lane) * 8U;
            const std::uint64_t mask = UINT64_MAX >> shift;
            result = (mem & ~mask) | (old_rt >> shift);
        } else { // SDR: store least-significant register bytes into high memory lanes
            const unsigned shift = lane * 8U;
            const std::uint64_t mask = UINT64_MAX << shift;
            result = (mem & ~mask) | (old_rt << shift);
        }
        if (!active_ee_memory_->write(static_cast<std::uint32_t>(aligned), 8, result, nullptr))
            return fault("Unmapped EE RAM doubleword-merge store");
        return true;
    }
    case 0x1E: case 0x1F: { // LQ/SQ: 128-bit aligned EE GPR transfers
        const bool store = opcode == 0x1F;
        const std::uint64_t address = reg(rs) + static_cast<std::uint64_t>(simm);
        const auto fault = [&](const char* message) {
            if (error) *error = message;
            bad_vaddr_ = address;
            raise_exception(store ? ExcAddressErrorStore : ExcAddressErrorLoad, cur_pc, in_delay);
            return false;
        };
        if (active_ee_memory_ == nullptr)
            return fault("LQ/SQ require the mapped EE memory interface");
        if (address > 0xFFFFFFFFULL)
            return fault("EE LQ/SQ effective address exceeds 32-bit address space");
        if ((address & 0xFU) != 0U)
            return fault("Misaligned EE LQ/SQ access");
        if (store) {
            if (!active_ee_memory_->write128(static_cast<std::uint32_t>(address),
                                              reg(rt), reg_high(rt), nullptr))
                return fault("Unmapped EE LQ/SQ store");
        } else {
            std::uint64_t low = 0, high = 0;
            if (!active_ee_memory_->read128(static_cast<std::uint32_t>(address),
                                             low, high, nullptr))
                return fault("Unmapped EE LQ/SQ load");
            set_reg(rt, low);
            set_reg_high(rt, high);
        }
        return true;
    }
    case 0x20: case 0x21: case 0x23: case 0x24: case 0x25: case 0x27:
    case 0x28: case 0x29: case 0x2B: case 0x37: case 0x3F: { // LB/LH/LW/LBU/LHU/LWU/SB/SH/SW/LD/SD
        const bool store = opcode == 0x28 || opcode == 0x29 || opcode == 0x2B || opcode == 0x3F;
        const std::size_t width = (opcode == 0x20 || opcode == 0x24 || opcode == 0x28) ? 1U :
                                  (opcode == 0x21 || opcode == 0x25 || opcode == 0x29) ? 2U :
                                  (opcode == 0x37 || opcode == 0x3F) ? 8U : 4U;
        const std::uint64_t address = reg(rs) + static_cast<std::uint64_t>(simm);
        std::uint8_t* bytes = nullptr;
        std::size_t offset = 0;
        const auto fault = [&](const char* message) {
            if (error) *error = message;
            bad_vaddr_ = address;
            raise_exception(store ? ExcAddressErrorStore : ExcAddressErrorLoad, cur_pc, in_delay);
            return false;
        };
        if (active_ee_memory_ != nullptr) {
            // The current EE memory-map API accepts 32-bit guest addresses. Never
            // truncate a wider effective address into a different mapped location.
            if (address > 0xFFFFFFFFULL)
                return fault("EE RAM effective address exceeds 32-bit address space");
            std::uint64_t value = 0;
            if (store) {
                if (!active_ee_memory_->write(static_cast<std::uint32_t>(address), width, reg(rt), nullptr))
                    return fault("Unaligned or unmapped EE RAM store");
            } else {
                if (!active_ee_memory_->read(static_cast<std::uint32_t>(address), width, value, nullptr))
                    return fault("Unaligned or unmapped EE RAM load");
                if (opcode == 0x20) set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int8_t>(value))));
                else if (opcode == 0x21) set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int16_t>(value))));
                else if (opcode == 0x23) set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(value))));
                else set_reg(rt, value);
            }
            return true;
        } else if (active_ram_ != nullptr) {
            if (address < active_ram_base_) return fault("Data access is below external RAM");
            const std::uint64_t relative = address - active_ram_base_;
            if ((address & (width - 1U)) != 0U || relative > active_ram_->size() ||
                active_ram_->size() - static_cast<std::size_t>(relative) < width)
                return fault("Unaligned or out-of-range external RAM access");
            offset = static_cast<std::size_t>(relative);
            bytes = active_ram_->data();
        } else {
            if ((address & (width - 1U)) != 0U || address > TestMemorySize - width)
                return fault("Unaligned or out-of-range test RAM access");
            offset = static_cast<std::size_t>(address);
            bytes = memory_.data();
        }

        if (store) {
            const std::uint64_t value = reg(rt);
            for (std::size_t i = 0; i < width; ++i)
                bytes[offset + i] = static_cast<std::uint8_t>((value >> (8U * i)) & 0xFFU);
        } else {
            std::uint64_t value = 0;
            for (std::size_t i = 0; i < width; ++i)
                value |= static_cast<std::uint64_t>(bytes[offset + i]) << (8U * i);
            if (opcode == 0x20) set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int8_t>(value))));
            else if (opcode == 0x21) set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int16_t>(value))));
            else if (opcode == 0x23) set_reg(rt, static_cast<std::uint64_t>(static_cast<std::int64_t>(static_cast<std::int32_t>(value))));
            else set_reg(rt, value); // LBU/LHU/LWU zero-extend; LD preserves all 64 bits
        }
        return true;
    }
    default: break;
    }
    return unsupported();
}

bool Cpu::step_from_ram(std::vector<std::uint8_t>& ram,
                        std::uint32_t ram_base,
                        std::string* error) {
    if ((pc_ & 3U) != 0U) {
        if (error) *error = "Unaligned instruction fetch";
        bad_vaddr_ = pc_;
        raise_exception(ExcAddressErrorLoad, pc_, delay_slot_pending_);
        return false;
    }
    if (pc_ < ram_base) {
        if (error) *error = "Instruction fetch is outside RAM";
        bad_vaddr_ = pc_;
        raise_exception(ExcAddressErrorLoad, pc_, delay_slot_pending_);
        return false;
    }
    const std::uint64_t offset = pc_ - ram_base;
    if (offset > ram.size() ||
        ram.size() - static_cast<std::size_t>(offset) < sizeof(std::uint32_t)) {
        if (error) *error = "Instruction fetch is outside RAM";
        bad_vaddr_ = pc_;
        raise_exception(ExcAddressErrorLoad, pc_, delay_slot_pending_);
        return false;
    }
    const auto index = static_cast<std::size_t>(offset);
    const std::uint32_t instruction =
        static_cast<std::uint32_t>(ram[index]) |
        (static_cast<std::uint32_t>(ram[index + 1]) << 8U) |
        (static_cast<std::uint32_t>(ram[index + 2]) << 16U) |
        (static_cast<std::uint32_t>(ram[index + 3]) << 24U);
    if (error) error->clear();
    active_ram_ = &ram;
    active_ram_base_ = ram_base;
    const bool succeeded = step(instruction, error);
    active_ram_ = nullptr;
    active_ram_base_ = 0;
    return succeeded;
}

bool Cpu::step_from_ee_memory(Ps2EeMemoryMap& memory, std::string* error) {
    if ((pc_ & 3U) != 0U) {
        if (error) *error = "Unaligned instruction fetch";
        bad_vaddr_ = pc_;
        raise_exception(ExcAddressErrorLoad, pc_, delay_slot_pending_);
        return false;
    }
    // The EE map translates 32-bit virtual addresses. Reject high bits rather
    // than silently fetching from the address produced by truncation.
    if (pc_ > 0xFFFFFFFFULL) {
        if (error) *error = "Instruction fetch exceeds 32-bit EE address space";
        bad_vaddr_ = pc_;
        raise_exception(ExcAddressErrorLoad, pc_, delay_slot_pending_);
        return false;
    }
    std::uint64_t fetched = 0;
    if (!memory.read(static_cast<std::uint32_t>(pc_), 4, fetched, error)) {
        bad_vaddr_ = pc_;
        if (error && error->empty()) *error = "Instruction fetch is outside mapped EE RAM";
        // An instruction-fetch bus/address fault must enter the EE exception
        // path just like an unaligned PC; callers can inspect EPC/Cause and
        // execute the BIOS exception handler instead of merely stopping.
        raise_exception(ExcAddressErrorLoad, pc_, delay_slot_pending_);
        return false;
    }
    active_ee_memory_ = &memory;
    const bool succeeded = step(static_cast<std::uint32_t>(fetched), error);
    active_ee_memory_ = nullptr;
    return succeeded;
}

} // namespace retroshelf

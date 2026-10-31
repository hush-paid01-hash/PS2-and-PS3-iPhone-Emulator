#include "retroshelf/cpu.hpp"
#include "retroshelf/ps2_ee_memory_map.hpp"
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

int main() {
    retroshelf::Cpu cpu;
    int failures = 0;
    const auto check = [&failures](bool ok, const char* label) {
        if (!ok) { std::cerr << "FAIL: " << label << "\n"; ++failures; }
        else { std::cout << "PASS: " << label << "\n"; }
    };

    constexpr std::uint32_t addiu7 = (0x09U << 26U) | (8U << 16U) | 7U;
    constexpr std::uint32_t addiu5 = (0x09U << 26U) | (9U << 16U) | 5U;
    constexpr std::uint32_t addu = (8U << 21U) | (9U << 16U) | (10U << 11U) | 0x21U;
    check(cpu.step(addiu7), "ADDIU executes");
    check(cpu.reg(8) == 7, "ADDIU writes expected value");
    check(cpu.reg(0) == 0, "register zero stays zero");
    check(cpu.step(addiu5), "second ADDIU executes");
    check(cpu.step(addu), "ADDU executes");
    check(cpu.reg(10) == 12, "ADDU computes 7 + 5");
    check(cpu.pc() == 12, "PC advances four bytes per instruction");

    cpu.reset();
    check(cpu.reg(8) == 0 && cpu.pc() == 0, "reset clears CPU state");

    // The EE's 128-bit GPR model is currently represented as low/high halves.
    // Both halves of register zero must remain hardwired to zero.
    cpu.set_reg_high(0, 0xDEADBEEFCAFEBABEULL);
    check(cpu.reg_high(0) == 0,
          "high half of GPR zero remains hardwired to zero");
    cpu.set_reg(0, 0x0123456789ABCDEFULL);
    check(cpu.reg(0) == 0,
          "low half of GPR zero remains hardwired to zero");
    cpu.set_reg(4, 0x1111222233334444ULL);
    cpu.set_reg_high(4, 0xAAAABBBBCCCCDDDDULL);
    check(cpu.reg(4) == 0x1111222233334444ULL &&
              cpu.reg_high(4) == 0xAAAABBBBCCCCDDDDULL,
          "ordinary GPR low and high halves retain independent values");
    cpu.reset();
    check(cpu.reg(4) == 0 && cpu.reg_high(4) == 0,
          "reset clears both halves of extended GPR state");

    cpu.set_reg(11, 16);
    cpu.set_reg(12, 0x12345678);
    constexpr std::uint32_t sw = (0x2BU << 26U) | (11U << 21U) | (12U << 16U);
    constexpr std::uint32_t lw = (0x23U << 26U) | (11U << 21U) | (13U << 16U);
    check(cpu.step(sw), "SW stores a word in test RAM");
    check(cpu.step(lw), "LW loads a word from test RAM");
    check(cpu.reg(13) == 0x12345678, "LW returns the stored word");

    cpu.set_reg(11, 3);
    std::string error;
    check(!cpu.step(lw, &error), "misaligned LW is rejected");
    check(!error.empty(), "invalid memory access explains why");

    cpu.set_reg(11, retroshelf::Cpu::TestMemorySize);
    error.clear();
    check(!cpu.step(sw, &error), "out-of-range SW is rejected");

    cpu.reset();
    check(cpu.reg(11) == 0 && cpu.pc() == 0, "reset clears registers and PC");
    error.clear();
    check(!cpu.step(0xFFFFFFFFU, &error), "unsupported instruction is rejected");
    check(!error.empty(), "unsupported instruction explains why");

    using retroshelf::Cpu;
    const auto itype = [](std::uint32_t op, std::uint32_t rs, std::uint32_t rt, std::int16_t imm) {
        return (op << 26U) | (rs << 21U) | (rt << 16U) | static_cast<std::uint16_t>(imm);
    };
    const std::uint32_t nop = 0;
    const std::uint32_t set1 = itype(0x09, 0, 20, 1); // ADDIU r20,r0,1 (delay-slot marker)
    const std::uint32_t bad_lw = itype(0x23, 0, 21, 3); // misaligned LW -> AdEL

    const auto rtype = [](std::uint32_t rs, std::uint32_t rt, std::uint32_t rd, std::uint32_t funct) {
        return (rs << 21U) | (rt << 16U) | (rd << 11U) | funct;
    };

    // EE trap instructions raise ExcCode 13 only when their comparison matches.
    cpu.reset(); cpu.set_reg(1, static_cast<std::uint64_t>(-2LL)); cpu.set_reg(2, 3);
    error.clear();
    check(!cpu.step(rtype(1, 2, 0, 0x32), &error) && !error.empty() &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcTrap,
          "TLT raises a trap when the signed comparison is true");
    cpu.reset(); cpu.set_reg(1, static_cast<std::uint64_t>(-3LL)); cpu.set_reg(2, static_cast<std::uint64_t>(-2LL));
    check(cpu.step(rtype(1, 2, 0, 0x30)) && cpu.pc() == 4,
          "TGE continues when its signed comparison is false");
    cpu.reset(); cpu.set_reg(1, UINT64_MAX); cpu.set_reg(2, 3);
    check(!cpu.step(rtype(1, 2, 0, 0x31)) &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcTrap,
          "TGEU compares full-width operands as unsigned values");
    cpu.reset(); cpu.set_reg(1, 0x1234); cpu.set_reg(2, 0x1234);
    check(!cpu.step(rtype(1, 2, 0, 0x34)),
          "TEQ raises a trap for equal register operands");
    cpu.reset(); cpu.set_reg(1, 0x1234); cpu.set_reg(2, 0x5678);
    check(!cpu.step(rtype(1, 2, 0, 0x36)) &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcTrap,
          "TNE raises a trap for unequal register operands");

    cpu.reset(); cpu.set_reg(1, 5);
    check(!cpu.step(itype(0x01, 1, 0x08, 4)) &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcTrap,
          "TGEI traps when the signed register is at least the immediate");
    cpu.reset(); cpu.set_reg(1, static_cast<std::uint64_t>(-2LL));
    check(!cpu.step(itype(0x01, 1, 0x0A, -1)) &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcTrap,
          "TLTI compares against a sign-extended immediate");
    cpu.reset(); cpu.set_reg(1, 3);
    check(!cpu.step(itype(0x01, 1, 0x0B, -1)) &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcTrap,
          "TLTIU compares unsigned against the sign-extended immediate");
    cpu.reset(); cpu.set_reg(1, UINT64_MAX);
    check(!cpu.step(itype(0x01, 1, 0x09, -1)) &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcTrap,
          "TGEIU treats the sign-extended immediate as an unsigned value");
    cpu.reset(); cpu.set_reg(1, 7);
    check(!cpu.step(itype(0x01, 1, 0x0C, 7)),
          "TEQI traps when the register equals the immediate");
    cpu.reset(); cpu.set_reg(1, 7);
    check(!cpu.step(itype(0x01, 1, 0x0E, 8)),
          "TNEI traps when the register differs from the immediate");
    cpu.reset(); cpu.set_reg(1, 7);
    check(cpu.step(itype(0x01, 1, 0x0E, 7)) && cpu.pc() == 4,
          "TNEI continues when the register equals the immediate");

    // A trap in a branch delay slot records the branch address and Cause.BD.
    cpu.reset(); cpu.set_pc(0x100);
    check(cpu.step(itype(0x04, 0, 0, 2)), "trap-delay test schedules a branch delay slot");
    error.clear();
    check(!cpu.step(rtype(1, 2, 0, 0x34), &error) &&
              cpu.cop0_epc() == 0x100 && (cpu.cop0_cause() & Cpu::CauseBd) != 0U &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcTrap,
          "register trap in a delay slot preserves EPC and sets Cause.BD");

    // REGIMM's immediate traps are not branches and must execute in a delay slot.
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 5);
    check(cpu.step(itype(0x04, 0, 0, 2)),
          "immediate-trap delay test schedules a branch delay slot");
    error.clear();
    check(!cpu.step(itype(0x01, 1, 0x08, 4), &error) &&
              cpu.cop0_epc() == 0x100 && (cpu.cop0_cause() & Cpu::CauseBd) != 0U &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcTrap,
          "TGEI executes in a delay slot and raises an architected trap");

    // COP0 EPC round-trip and ERET return-path regression.
    // MTC0 r1,EPC; MTC0 r2,Status; ERET must restore PC and clear EXL.
    cpu.reset();
    cpu.set_reg(1, 0x80001234ULL);
    cpu.set_reg(2, Cpu::StatusExl);
    constexpr std::uint32_t cop0_test_mtc0_epc = (0x10U << 26U) | (0x04U << 21U) |
                                       (1U << 16U) | (14U << 11U);
    constexpr std::uint32_t cop0_test_mtc0_status = (0x10U << 26U) | (0x04U << 21U) |
                                          (2U << 16U) | (12U << 11U);
    constexpr std::uint32_t cop0_test_mfc0_epc = (0x10U << 26U) | (14U << 11U) |
                                       (3U << 16U);
    constexpr std::uint32_t cop0_test_eret = (0x10U << 26U) | (0x10U << 21U) | 0x18U;
    check(cpu.step(cop0_test_mtc0_epc), "MTC0 writes the exception program counter");
    check(cpu.step(cop0_test_mtc0_status) && cpu.cop0_status() == Cpu::StatusExl,
          "MTC0 writes Status.EXL");
    check(cpu.step(cop0_test_mfc0_epc) && cpu.reg(3) == 0xFFFFFFFF80001234ULL,
          "MFC0 reads the exception program counter");
    check(cpu.step(cop0_test_eret) && cpu.pc() == 0x80001234ULL &&
              cpu.next_pc() == 0x80001238ULL &&
              (cpu.cop0_status() & Cpu::StatusExl) == 0U &&
              !cpu.in_delay_slot(),
          "ERET returns to EPC and clears EXL without a delay slot");

    // Error-level return has priority over EXL/EPC: ERL selects ErrorEPC and
    // clears only ERL, leaving EXL unchanged if it was also set.
    cpu.reset();
    cpu.set_reg(1, 0xBFC00100ULL);
    cpu.set_reg(2, Cpu::StatusErl | Cpu::StatusExl);
    constexpr std::uint32_t cop0_test_mtc0_error_epc =
        (0x10U << 26U) | (0x04U << 21U) | (1U << 16U) | (30U << 11U);
    check(cpu.step(cop0_test_mtc0_error_epc),
          "MTC0 writes ErrorEPC for error-level return");
    check(cpu.step(cop0_test_mtc0_status),
          "MTC0 sets both Status.ERL and Status.EXL");
    check(cpu.step(cop0_test_eret) && cpu.pc() == 0xBFC00100ULL &&
              cpu.next_pc() == 0xBFC00104ULL &&
              (cpu.cop0_status() & Cpu::StatusErl) == 0U &&
              (cpu.cop0_status() & Cpu::StatusExl) != 0U,
          "ERET selects ErrorEPC and clears ERL before considering EXL");

    // Immediate logical operations zero-extend their 16-bit masks.
    cpu.reset(); cpu.set_reg(1, 0x12345678);
    check(cpu.step(itype(0x0C, 1, 2, 0x00FF)) && cpu.reg(2) == 0x78,
          "ANDI zero-extends mask and clears other bits");
    check(cpu.step(itype(0x0E, 1, 3, 0x00FF)) && cpu.reg(3) == 0x12345687,
          "XORI zero-extends mask and toggles bits");

    // CACHE/PREF are hints in this functional interpreter: no cache hierarchy is modeled.
    cpu.reset(); cpu.set_reg(1, 0x1000);
    check(cpu.step(itype(0x2F, 1, 0x1A, 0x20)) && cpu.pc() == 4,
          "CACHE instruction is accepted as a no-op without a cache model");
    check(cpu.step(itype(0x33, 1, 0, 0x40)) && cpu.pc() == 8,
          "PREF hint is accepted as a no-op without a cache model");

    // Signed and unsigned comparisons, including a negative sign-extended immediate.
    cpu.reset(); cpu.set_reg(1, static_cast<std::uint64_t>(-1));
    check(cpu.step(itype(0x0A, 1, 2, 1)) && cpu.reg(2) == 1,
          "SLTI compares signed register values");
    check(cpu.step(itype(0x0A, 1, 3, -2)) && cpu.reg(3) == 0,
          "SLTI handles negative immediates");
    cpu.reset(); cpu.set_reg(1, 1);
    check(cpu.step(itype(0x0B, 1, 2, -1)) && cpu.reg(2) == 1,
          "SLTIU sign-extends immediate before unsigned comparison");
    cpu.set_reg(1, static_cast<std::uint64_t>(-1));
    check(cpu.step(itype(0x0B, 1, 3, -1)) && cpu.reg(3) == 0,
          "SLTIU handles unsigned equality at all-ones");

    // Register-register logical, subtract, and compare instructions.
    cpu.reset(); cpu.set_reg(1, 0xF0F0); cpu.set_reg(2, 0x0FF0);
    check(cpu.step(rtype(1, 2, 3, 0x24)) && cpu.reg(3) == 0x00F0, "AND computes bitwise intersection");
    check(cpu.step(rtype(1, 2, 4, 0x26)) && cpu.reg(4) == 0xFF00, "XOR computes bitwise difference");
    check(cpu.step(rtype(1, 2, 5, 0x27)) && cpu.reg(5) == ~std::uint64_t(0xFFF0), "NOR complements bitwise OR");
    check(cpu.step(rtype(1, 2, 6, 0x23)) && cpu.reg(6) == 0xE100, "SUBU subtracts without signed overflow traps");
    check(cpu.step(rtype(1, 2, 7, 0x2A)) && cpu.reg(7) == 0, "SLT uses signed comparison");
    cpu.set_reg(1, static_cast<std::uint64_t>(-1)); cpu.set_reg(2, 1);
    check(cpu.step(rtype(1, 2, 8, 0x2A)) && cpu.reg(8) == 1, "SLT treats high-bit values as negative");
    check(cpu.step(rtype(1, 2, 9, 0x2B)) && cpu.reg(9) == 0, "SLTU compares register values unsigned");

    // Arithmetic shifts must propagate the sign bit without relying on the
    // implementation-defined behavior of right-shifting signed integers.
    cpu.reset(); cpu.set_reg(1, 0x80000000U);
    check(cpu.step((1U << 16U) | (2U << 11U) | (1U << 6U) | 0x03U) &&
              cpu.reg(2) == 0xFFFFFFFFC0000000ULL,
          "SRA sign-extends the 32-bit operand and propagates its sign bit");
    cpu.reset(); cpu.set_reg(1, 0x80000000U); cpu.set_reg(3, 1);
    check(cpu.step((3U << 21U) | (1U << 16U) | (4U << 11U) | 0x07U) &&
              cpu.reg(4) == 0xFFFFFFFFC0000000ULL,
          "SRAV sign-extends the 32-bit operand for variable arithmetic shift");
    cpu.reset(); cpu.set_reg(1, 0x8000000000000000ULL);
    check(cpu.step((1U << 16U) | (2U << 11U) | (1U << 6U) | 0x3BU) &&
              cpu.reg(2) == 0xC000000000000000ULL,
          "DSRA propagates the 64-bit sign bit");
    cpu.reset(); cpu.set_reg(1, 0x8000000000000000ULL); cpu.set_reg(3, 1);
    check(cpu.step((3U << 21U) | (1U << 16U) | (4U << 11U) | 0x17U) &&
              cpu.reg(4) == 0xC000000000000000ULL,
          "DSRAV propagates the sign bit for variable 64-bit shifts");
    cpu.reset(); cpu.set_reg(1, 0x8000000000000000ULL);
    check(cpu.step((1U << 16U) | (2U << 11U) | 0x3FU) &&
              cpu.reg(2) == 0xFFFFFFFF80000000ULL,
          "DSRA32 propagates the sign bit for shifts of 32 through 63");

    // BEQ taken: delay slot runs, then target.
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 5); cpu.set_reg(2, 5);
    check(cpu.step(itype(0x04, 1, 2, 3)), "BEQ taken executes");
    check(cpu.in_delay_slot() && cpu.pc() == 0x104, "BEQ leaves PC at delay slot");
    check(cpu.step(set1) && cpu.reg(20) == 1, "delay slot instruction executes");
    check(cpu.pc() == 0x104 + 12 && !cpu.in_delay_slot(), "BEQ taken goes to PC+4+offset*4");

    // BEQ not taken: delay slot still runs, falls through.
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 5); cpu.set_reg(2, 6);
    cpu.step(itype(0x04, 1, 2, 3)); cpu.step(set1);
    check(cpu.reg(20) == 1 && cpu.pc() == 0x108, "BEQ not taken falls through after delay slot");

    // BNE taken backward.
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 1);
    cpu.step(itype(0x05, 1, 0, -2)); cpu.step(nop);
    check(cpu.pc() == 0x104 - 8, "BNE taken with negative offset");

    // BNE untaken.
    cpu.reset(); cpu.set_pc(0x100);
    cpu.step(itype(0x05, 0, 0, 5)); cpu.step(nop);
    check(cpu.pc() == 0x108, "BNE not taken falls through");

    // BLEZ / BGTZ / BLTZ / BGEZ
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 0);
    cpu.step(itype(0x06, 1, 0, 4)); cpu.step(nop);
    check(cpu.pc() == 0x104 + 16, "BLEZ taken on zero");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 0);
    cpu.step(itype(0x07, 1, 0, 4)); cpu.step(nop);
    check(cpu.pc() == 0x108, "BGTZ not taken on zero");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, static_cast<std::uint64_t>(-1));
    cpu.step(itype(0x01, 1, 0, 4)); cpu.step(nop);
    check(cpu.pc() == 0x104 + 16, "BLTZ taken on negative");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, static_cast<std::uint64_t>(-1));
    cpu.step(itype(0x01, 1, 1, 4)); cpu.step(nop);
    check(cpu.pc() == 0x108, "BGEZ not taken on negative");

    // Branch-likely instructions annul their delay slot when not taken.
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 5); cpu.set_reg(2, 6); cpu.set_reg(20, 0);
    check(cpu.step(itype(0x14, 1, 2, 3)) && !cpu.in_delay_slot() && cpu.pc() == 0x108,
          "BEQL not taken skips its delay slot");
    check(cpu.step(set1) && cpu.reg(20) == 1 && cpu.pc() == 0x10C,
          "BEQL annulled slot is skipped and execution resumes at PC+8");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 5); cpu.set_reg(2, 5);
    check(cpu.step(itype(0x14, 1, 2, 3)) && cpu.in_delay_slot(), "BEQL taken schedules delay slot");
    check(cpu.step(set1) && cpu.reg(20) == 1 && cpu.pc() == 0x110,
          "BEQL taken executes delay slot then branches");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 5); cpu.set_reg(2, 5);
    cpu.step(itype(0x15, 1, 2, 2));
    check(!cpu.in_delay_slot() && cpu.pc() == 0x108, "BNEL not taken annuls delay slot");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 0);
    cpu.step(itype(0x16, 1, 0, 2));
    check(cpu.in_delay_slot(), "BLEZL taken schedules delay slot");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 0);
    cpu.step(itype(0x17, 1, 0, 2));
    check(!cpu.in_delay_slot() && cpu.pc() == 0x108, "BGTZL not taken annuls delay slot");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, static_cast<std::uint64_t>(-1));
    cpu.step(itype(0x01, 1, 0x02, 2));
    check(cpu.in_delay_slot(), "BLTZL taken schedules delay slot");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, static_cast<std::uint64_t>(-1)); cpu.set_reg(31, 0xBAD);
    cpu.step(itype(0x01, 1, 0x13, 2));
    check(!cpu.in_delay_slot() && cpu.pc() == 0x108 && cpu.reg(31) == 0xBAD,
          "BGEZALL not taken annuls slot and does not write link");

    // BGEZAL links even when untaken; BLTZAL taken links to PC+8.
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, static_cast<std::uint64_t>(-1));
    cpu.step(itype(0x01, 1, 0x11, 4)); cpu.step(nop);
    check(cpu.reg(31) == 0x108 && cpu.pc() == 0x108, "BGEZAL untaken still writes link");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, static_cast<std::uint64_t>(-1));
    cpu.step(itype(0x01, 1, 0x10, 4)); cpu.step(nop);
    check(cpu.reg(31) == 0x108 && cpu.pc() == 0x104 + 16, "BLTZAL taken links and branches");

    // J / JAL
    cpu.reset(); cpu.set_pc(0x100);
    cpu.step((0x02U << 26U) | (0x40U)); cpu.step(set1);
    check(cpu.reg(20) == 1 && cpu.pc() == 0x100, "J jumps to 26-bit target << 2 after delay slot");
    cpu.reset(); cpu.set_pc(0x200);
    cpu.step((0x03U << 26U) | 0x10U); cpu.step(nop);
    check(cpu.reg(31) == 0x208 && cpu.pc() == 0x40, "JAL links PC+8 and jumps");

    // JR / JALR
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(5, 0x300);
    cpu.step((5U << 21U) | 0x08U); cpu.step(set1);
    check(cpu.reg(20) == 1 && cpu.pc() == 0x300, "JR jumps to register after delay slot");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(5, 0x300);
    cpu.step((5U << 21U) | (6U << 11U) | 0x09U); cpu.step(nop);
    check(cpu.reg(6) == 0x108 && cpu.pc() == 0x300, "JALR links PC+8 and jumps");
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(5, 0x300);
    cpu.step((5U << 21U) | (5U << 11U) | 0x09U); cpu.step(nop);
    check(cpu.pc() == 0x300 && cpu.reg(5) == 0x108, "JALR with rd==rs uses old target");

    // Branches and jumps inside a delay slot are rejected before changing
    // the pending target or performing link-register side effects.
    cpu.reset(); cpu.set_pc(0x100);
    cpu.step(itype(0x04, 0, 0, 3));
    error.clear();
    check(!cpu.step(itype(0x04, 0, 0, 3), &error) && !error.empty(),
          "conditional branch in delay slot is rejected");

    cpu.reset(); cpu.set_pc(0x100);
    cpu.step(itype(0x04, 0, 0, 3));
    error.clear();
    check(!cpu.step((0x02U << 26U) | 0x40U, &error) && !error.empty() &&
              cpu.pc() == 0x104 && cpu.in_delay_slot(),
          "J in delay slot is rejected without overwriting pending branch");

    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(31, 0xBAD);
    cpu.step(itype(0x04, 0, 0, 3));
    error.clear();
    check(!cpu.step((0x03U << 26U) | 0x40U, &error) && !error.empty() &&
              cpu.pc() == 0x104 && cpu.in_delay_slot() && cpu.reg(31) == 0xBAD,
          "JAL in delay slot is rejected without writing the link register");

    // Exceptions: normal instruction sets BD=0, EPC=instruction.
    cpu.reset(); cpu.set_pc(0x100);
    check(!cpu.step(bad_lw, &error), "faulting LW reports failure");
    check(cpu.cop0_epc() == 0x100 && (cpu.cop0_cause() & Cpu::CauseBd) == 0, "non-delay-slot fault: EPC=PC, BD=0");
    check(((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcAddressErrorLoad, "AdEL exception code");
    check(cpu.pc() == Cpu::GeneralExceptionVector && (cpu.cop0_status() & Cpu::StatusExl) != 0, "exception vectors and sets EXL");

    // Fault in a taken-branch delay slot: EPC = branch address, BD=1, branch cancelled.
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 5); cpu.set_reg(2, 5);
    cpu.step(itype(0x04, 1, 2, 3));
    check(!cpu.step(bad_lw, &error), "fault in delay slot reports failure");
    check(cpu.cop0_epc() == 0x100 && (cpu.cop0_cause() & Cpu::CauseBd) != 0, "delay-slot fault: EPC=branch, BD=1");
    check(cpu.pc() == Cpu::GeneralExceptionVector && !cpu.in_delay_slot(), "delay-slot fault discards pending branch");

    // Fault in an untaken-branch delay slot, and in a JR delay slot (store, AdES).
    cpu.reset(); cpu.set_pc(0x200); cpu.set_reg(1, 1);
    cpu.step(itype(0x04, 1, 0, 3));
    cpu.step(0x0000003DU, &error);
    check(cpu.cop0_epc() == 0x200 && (cpu.cop0_cause() & Cpu::CauseBd) != 0 &&
          ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcReservedInstruction, "RI in untaken-branch delay slot sets BD");
    cpu.reset(); cpu.set_pc(0x300); cpu.set_reg(5, 0x400); cpu.set_reg(6, 3);
    cpu.step((5U << 21U) | 0x08U);
    cpu.step(itype(0x2B, 6, 0, 0));
    check(cpu.cop0_epc() == 0x300 && (cpu.cop0_cause() & Cpu::CauseBd) != 0 &&
          ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcAddressErrorStore, "AdES in JR delay slot sets BD, EPC=JR");

    // Delay-slot fault followed by an ordinary instruction: handler instruction runs normally.
    cpu.reset(); cpu.set_pc(0x100); cpu.step(bad_lw);
    check(cpu.step(set1) && cpu.reg(20) == 1 && cpu.pc() == Cpu::GeneralExceptionVector + 4, "execution continues at exception vector");

    // Fixed and variable 32-bit shift operations.
    const auto shift = [](std::uint32_t rs, std::uint32_t rt, std::uint32_t rd,
                          std::uint32_t shamt, std::uint32_t funct) {
        return (rs << 21U) | (rt << 16U) | (rd << 11U) | (shamt << 6U) | funct;
    };
    cpu.reset(); cpu.set_reg(2, 0x00000001U);
    check(cpu.step(shift(0, 2, 3, 4, 0x00)) && cpu.reg(3) == 0x10,
          "SLL shifts left by encoded amount");
    cpu.set_reg(2, 0x80000000U);
    check(cpu.step(shift(0, 2, 4, 1, 0x02)) && cpu.reg(4) == 0x40000000ULL,
          "SRL shifts logically and sign-extends 32-bit result");
    check(cpu.step(shift(0, 2, 5, 1, 0x03)) && cpu.reg(5) == 0xFFFFFFFFC0000000ULL,
          "SRA preserves sign while shifting right");
    cpu.set_reg(1, 3); cpu.set_reg(2, 0x10);
    check(cpu.step(shift(1, 2, 6, 0, 0x04)) && cpu.reg(6) == 0x80,
          "SLLV uses low five bits of rs as shift amount");
    cpu.set_reg(2, 0x80000000U);
    check(cpu.step(shift(1, 2, 7, 0, 0x06)) && cpu.reg(7) == 0x10000000ULL,
          "SRLV shifts right by variable amount");
    check(cpu.step(shift(1, 2, 8, 0, 0x07)) && cpu.reg(8) == 0xFFFFFFFFF0000000ULL,
          "SRAV shifts right arithmetically by variable amount");
    cpu.set_reg(1, 0x20); cpu.set_reg(2, 0x10);
    check(cpu.step(shift(1, 2, 9, 0, 0x04)) && cpu.reg(9) == 0x10,
          "SLLV masks shift amount to five bits");


    // Byte and halfword accesses: sign extension for LB/LH, zero extension for LBU/LHU.
    cpu.reset(); cpu.set_reg(11, 32); cpu.set_reg(12, 0x80);
    check(cpu.step(itype(0x28, 11, 12, 0)), "SB stores a byte");
    check(cpu.step(itype(0x20, 11, 13, 0)) &&
          cpu.reg(13) == 0xFFFFFFFFFFFFFF80ULL, "LB sign-extends negative byte");
    check(cpu.step(itype(0x24, 11, 14, 0)) && cpu.reg(14) == 0x80,
          "LBU zero-extends byte");
    cpu.set_reg(12, 0x8001);
    check(cpu.step(itype(0x29, 11, 12, 2)), "SH stores a halfword");
    check(cpu.step(itype(0x21, 11, 15, 2)) &&
          cpu.reg(15) == 0xFFFFFFFFFFFF8001ULL, "LH sign-extends negative halfword");
    check(cpu.step(itype(0x25, 11, 16, 2)) && cpu.reg(16) == 0x8001,
          "LHU zero-extends halfword");
    cpu.set_reg(11, 33); error.clear();
    check(!cpu.step(itype(0x21, 11, 17, 0), &error) && !error.empty(),
          "misaligned LH faults");
    cpu.reset(); cpu.set_reg(11, 0); cpu.set_reg(12, 0x1234);
    check(cpu.step(itype(0x29, 11, 12, 0)), "SH can store to aligned test RAM");
    check(cpu.step(itype(0x25, 11, 13, 0)) && cpu.reg(13) == 0x1234,
          "LHU reads halfword stored by SH");

    // 32-bit unsigned word and full 64-bit doubleword memory operations.
    cpu.reset(); cpu.set_reg(11, 64); cpu.set_reg(12, 0xFFFFFFFF80000001ULL);
    check(cpu.step(itype(0x2B, 11, 12, 0)), "SW stores low 32 bits of a 64-bit register");
    check(cpu.step(itype(0x27, 11, 13, 0)) && cpu.reg(13) == 0x80000001ULL,
          "LWU zero-extends a word");
    check(cpu.step(itype(0x23, 11, 14, 0)) && cpu.reg(14) == 0xFFFFFFFF80000001ULL,
          "LW sign-extends the same word");

    cpu.reset(); cpu.set_reg(11, 72); cpu.set_reg(12, 0xFEDCBA9876543210ULL);
    check(cpu.step(itype(0x3F, 11, 12, 0)), "SD stores a full 64-bit doubleword");
    check(cpu.step(itype(0x37, 11, 13, 0)) && cpu.reg(13) == 0xFEDCBA9876543210ULL,
          "LD restores all 64 bits");
    cpu.set_reg(11, 76); error.clear();
    check(!cpu.step(itype(0x37, 11, 14, 0), &error) && !error.empty(),
          "misaligned LD faults");
    cpu.reset(); cpu.set_reg(11, Cpu::TestMemorySize - 4); cpu.set_reg(12, 0x1122334455667788ULL);
    error.clear();
    check(!cpu.step(itype(0x3F, 11, 12, 0), &error) && !error.empty(),
          "out-of-range SD faults without writing past test RAM");

    // MIPS III/EE 32-bit results must be sign-extended to the 64-bit GPR width.
    cpu.reset(); cpu.set_reg(1, 0x7FFFFFFFULL); cpu.set_reg(2, 1);
    check(cpu.step(rtype(1, 2, 3, 0x21)) && cpu.reg(3) == 0xFFFFFFFF80000000ULL,
          "ADDU sign-extends a 32-bit result");
    cpu.reset(); cpu.set_reg(1, 0x80000000ULL); cpu.set_reg(2, 1);
    check(cpu.step(rtype(1, 2, 3, 0x23)) && cpu.reg(3) == 0x7FFFFFFFULL,
          "SUBU sign-extends a positive 32-bit result");
    cpu.reset();
    check(cpu.step(itype(0x0F, 0, 4, 0x8000)) && cpu.reg(4) == 0xFFFFFFFF80000000ULL,
          "LUI sign-extends its 32-bit result on the EE");

    // EE 64-bit immediate arithmetic: sign extension and overflow behavior.
    cpu.reset(); cpu.set_reg(1, 0x100000000ULL);
    check(cpu.step(itype(0x19, 1, 2, -1)) && cpu.reg(2) == 0xFFFFFFFFULL,
          "DADDIU adds a sign-extended immediate in 64 bits");
    cpu.reset(); cpu.set_reg(1, 0x7FFFFFFFFFFFFFF0ULL);
    check(cpu.step(itype(0x18, 1, 2, 15)) && cpu.reg(2) == 0x7FFFFFFFFFFFFFFFULL,
          "DADDI handles signed 64-bit addition without overflow");
    cpu.reset(); cpu.set_reg(1, 0x7FFFFFFFFFFFFFFFULL); cpu.set_reg(2, 123);
    error.clear();
    check(!cpu.step(itype(0x18, 1, 2, 1), &error) && !error.empty(),
          "DADDI traps on signed 64-bit positive overflow");
    check(cpu.reg(2) == 123, "DADDI overflow leaves destination register unchanged");
    check(((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcOverflow,
          "DADDI overflow records the overflow exception code");
    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL);
    check(cpu.step(itype(0x19, 1, 2, 1)) && cpu.reg(2) == 0,
          "DADDIU wraps around without an overflow exception");

    // 32-bit multiply/divide edge cases: overflow guards and zero divisors.
    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFF80000000ULL); cpu.set_reg(2, 0xFFFFFFFFFFFFFFFFULL);
    check(cpu.step(rtype(1, 2, 0, 0x1A)) &&
          cpu.lo() == 0xFFFFFFFF80000000ULL && cpu.hi() == 0,
          "DIV handles INT32_MIN divided by -1 without host overflow");
    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFFFFFFF85ULL); cpu.set_reg(2, 0);
    check(cpu.step(rtype(1, 2, 0, 0x1A)) &&
          cpu.lo() == 1 && cpu.hi() == 0xFFFFFFFFFFFFFF85ULL,
          "DIV defines signed divide-by-zero HI/LO values");
    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFU); cpu.set_reg(2, 2);
    check(cpu.step(rtype(1, 2, 0, 0x1B)) && cpu.lo() == 0x7FFFFFFFULL && cpu.hi() == 1,
          "DIVU uses only the low 32 bits of each operand");

    cpu.reset(); cpu.set_reg(1, 0x12345678FFFFFFFFULL); cpu.set_reg(2, 0);
    check(cpu.step(rtype(1, 2, 0, 0x1B)) &&
          cpu.lo() == 0xFFFFFFFFULL && cpu.hi() == 0xFFFFFFFFULL,
          "DIVU by zero returns the 32-bit all-ones quotient and low-word dividend remainder");
    cpu.reset(); cpu.set_reg(1, 7); cpu.set_reg(2, 0);
    check(cpu.step(rtype(1, 2, 0, 0x1A)) &&
          cpu.lo() == 0xFFFFFFFFFFFFFFFFULL && cpu.hi() == 7,
          "DIV by zero with a nonnegative dividend returns minus one and preserves the remainder");
    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFFFFFFFF9ULL); cpu.set_reg(2, 0);
    check(cpu.step(rtype(1, 2, 0, 0x1A)) &&
          cpu.lo() == 1 && cpu.hi() == 0xFFFFFFFFFFFFFFF9ULL,
          "DIV by zero with a negative dividend returns one and sign-extends the remainder");
    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFU); cpu.set_reg(2, 2);
    check(cpu.step(rtype(1, 2, 0, 0x18)) &&
          cpu.lo() == 0xFFFFFFFFFFFFFFFEULL && cpu.hi() == 0xFFFFFFFFFFFFFFFFULL,
          "MULT sign-extends both 32-bit halves of a negative product");
    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFU); cpu.set_reg(2, 2);
    check(cpu.step(rtype(1, 2, 0, 0x19)) && cpu.lo() == 0xFFFFFFFEULL && cpu.hi() == 1,
          "MULTU zero-extends the low half of the same unsigned product");
    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFU); cpu.set_reg(2, 0xFFFFFFFFU);
    check(cpu.step(rtype(1, 2, 0, 0x19)) && cpu.lo() == 1 && cpu.hi() == 0xFFFFFFFEULL,
          "MULTU returns both halves of the 32-bit unsigned product");

    // 64-bit division must preserve signed quotient/remainder and unsigned full-width values.
    cpu.reset(); cpu.set_reg(1, static_cast<std::uint64_t>(-100)); cpu.set_reg(2, 9);
    check(cpu.step(rtype(1, 2, 0, 0x1E)) &&
          cpu.lo() == static_cast<std::uint64_t>(static_cast<std::int64_t>(-11)) &&
          cpu.hi() == static_cast<std::uint64_t>(static_cast<std::int64_t>(-1)),
          "DDIV returns a signed 64-bit quotient and remainder");
    cpu.reset(); cpu.set_reg(1, 0xFEDCBA9876543210ULL); cpu.set_reg(2, 0);
    check(cpu.step(rtype(1, 2, 0, 0x1F)) && cpu.lo() == UINT64_MAX &&
          cpu.hi() == 0xFEDCBA9876543210ULL,
          "DDIVU defines divide-by-zero HI/LO values without losing dividend bits");
    cpu.reset(); cpu.set_reg(1, 0xFEDCBA9876543210ULL); cpu.set_reg(2, 0x100000001ULL);
    check(cpu.step(rtype(1, 2, 0, 0x1F)) &&
          cpu.lo() == 0xFEDCBA97ULL && cpu.hi() == 0x77777779ULL,
          "DDIVU divides and returns remainder using full 64-bit operands");

    // HI/LO transfer instructions preserve the full 64-bit EE register values.
    cpu.reset();
    cpu.set_reg(1, 0xFEDCBA9876543210ULL);
    cpu.set_reg(2, 0x8123456789ABCDEFULL);
    check(cpu.step(rtype(1, 0, 0, 0x11)) && cpu.hi() == 0xFEDCBA9876543210ULL,
          "MTHI copies all 64 source bits into HI");
    check(cpu.step(rtype(0, 0, 3, 0x10)) && cpu.reg(3) == 0xFEDCBA9876543210ULL,
          "MFHI copies the full HI value to a GPR");
    check(cpu.step(rtype(2, 0, 0, 0x13)) && cpu.lo() == 0x8123456789ABCDEFULL,
          "MTLO copies all 64 source bits into LO");
    check(cpu.step(rtype(0, 0, 4, 0x12)) && cpu.reg(4) == 0x8123456789ABCDEFULL,
          "MFLO copies the full LO value to a GPR");
    check(cpu.reg(0) == 0, "HI/LO transfers never modify register zero");

    // EE signed 64-bit register arithmetic must trap on overflow.
    cpu.reset(); cpu.set_reg(1, 0x7FFFFFFFFFFFFFFFULL); cpu.set_reg(2, 1);
    error.clear();
    check(!cpu.step(rtype(1, 2, 3, 0x2C), &error) && !error.empty(),
          "DADD traps on signed 64-bit positive overflow");
    check(cpu.reg(3) == 0 && ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcOverflow,
          "DADD overflow preserves destination and raises overflow exception");
    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL); cpu.set_reg(2, 1);
    check(cpu.step(rtype(1, 2, 3, 0x2C)) && cpu.reg(3) == 0,
          "DADD handles valid negative-to-zero addition");
    cpu.reset(); cpu.set_reg(1, 0x8000000000000000ULL); cpu.set_reg(2, 1);
    error.clear();
    check(!cpu.step(rtype(1, 2, 3, 0x2E), &error) && !error.empty(),
          "DSUB traps on signed 64-bit negative overflow");
    check(cpu.reg(3) == 0 && ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcOverflow,
          "DSUB overflow preserves destination and raises overflow exception");
    cpu.reset(); cpu.set_reg(1, 7); cpu.set_reg(2, 5);
    check(cpu.step(rtype(1, 2, 3, 0x2E)) && cpu.reg(3) == 2,
          "DSUB computes a valid signed 64-bit difference");
    cpu.reset(); cpu.set_reg(1, 0x8000000000000000ULL); cpu.set_reg(2, 1);
    check(cpu.step(rtype(1, 2, 3, 0x2F)) && cpu.reg(3) == 0x7FFFFFFFFFFFFFFFULL,
          "DSUBU wraps without a signed-overflow exception");

    // COP0 boot/exception control: MFC0, MTC0 and ERET.
    const auto cop0 = [](std::uint32_t move, std::uint32_t rt, std::uint32_t rd) {
        return (0x10U << 26U) | (move << 21U) | (rt << 16U) | (rd << 11U);
    };
    cpu.reset(); cpu.set_reg(2, 0x1234);
    check(cpu.step(cop0(4, 2, 12)), "MTC0 writes the Status register");
    check(cpu.step(cop0(0, 3, 12)) && cpu.reg(3) == 0x1234,
          "MFC0 reads Status and sign-extends its 32-bit value");
    cpu.reset(); cpu.set_pc(0x100);
    check(!cpu.step(bad_lw, &error) && cpu.pc() == Cpu::GeneralExceptionVector,
          "fault enters the general exception vector before ERET test");
    cpu.set_reg(4, 0x200);
    check(cpu.step(cop0(4, 4, 14)), "MTC0 updates EPC");
    check(cpu.step((0x10U << 26U) | (0x10U << 21U) | 0x18U),
          "ERET returns from an exception");
    check(cpu.pc() == 0x200 && (cpu.cop0_status() & Cpu::StatusExl) == 0,
          "ERET resumes at EPC and clears Status.EXL");

    // SYNC is a valid instruction even before the memory-ordering model exists.
    cpu.reset();
    check(cpu.step(rtype(0, 0, 0, 0x0F)), "SYNC executes as a memory-ordering no-op");

    // EE conditional moves and variable 64-bit shifts.
    cpu.reset(); cpu.set_reg(1, 0x123456789ABCDEF0ULL); cpu.set_reg(2, 0);
    check(cpu.step(rtype(1, 2, 3, 0x0A)) && cpu.reg(3) == cpu.reg(1),
          "MOVZ copies source when condition register is zero");
    cpu.set_reg(3, 99);
    check(cpu.step(rtype(1, 2, 3, 0x0B)) && cpu.reg(3) == 99,
          "MOVN leaves destination unchanged when condition register is zero");
    cpu.set_reg(2, 1);
    check(cpu.step(rtype(1, 2, 3, 0x0B)) && cpu.reg(3) == cpu.reg(1),
          "MOVN copies source when condition register is nonzero");
    cpu.reset(); cpu.set_reg(1, 3); cpu.set_reg(2, 40);
    check(cpu.step(rtype(1, 2, 3, 0x14)) && cpu.reg(3) == 320,
          "DSLLV shifts left by the low six bits of the shift register");
    cpu.set_reg(1, 0x8000000000000000ULL); cpu.set_reg(2, 4);
    check(cpu.step(rtype(2, 1, 4, 0x16)) && cpu.reg(4) == 0x0800000000000000ULL,
          "DSRLV performs a logical 64-bit right shift");
    cpu.set_reg(2, 63);
    check(cpu.step(rtype(2, 1, 5, 0x17)) && cpu.reg(5) == 0xFFFFFFFFFFFFFFFFULL,
          "DSRAV sign-fills a negative value when shifted by 63");

    cpu.reset(); cpu.set_reg(1, 100); cpu.set_reg(2, 9);
    check(cpu.step(rtype(1, 2, 0, 0x1F)) && cpu.lo() == 11 && cpu.hi() == 1,
          "DDIVU computes quotient and remainder");

    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL); cpu.set_reg(2, 2);
    check(cpu.step(rtype(1, 2, 0, 0x1D)) && cpu.lo() == 0xFFFFFFFFFFFFFFFEULL && cpu.hi() == 1,
          "DMULTU produces the full unsigned 128-bit product");
    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL); cpu.set_reg(2, 2);
    check(cpu.step(rtype(1, 2, 0, 0x1C)) && cpu.lo() == 0xFFFFFFFFFFFFFFFEULL && cpu.hi() == 0xFFFFFFFFFFFFFFFFULL,
          "DMULT sign-corrects the high half for a negative operand");
    cpu.reset(); cpu.set_reg(1, static_cast<std::uint64_t>(-100LL)); cpu.set_reg(2, 9);
    check(cpu.step(rtype(1, 2, 0, 0x1E)) && cpu.lo() == static_cast<std::uint64_t>(-11LL) && cpu.hi() == static_cast<std::uint64_t>(-1LL),
          "DDIV returns signed quotient and remainder");

    cpu.reset(); cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL); cpu.set_reg(2, 0xFFFFFFFFFFFFFFFFULL);
    check(cpu.step(rtype(1, 2, 0, 0x1D)) && cpu.lo() == 1 && cpu.hi() == 0xFFFFFFFFFFFFFFFEULL,
          "DMULTU handles a product with carries across both 32-bit limbs");
    cpu.reset(); cpu.set_reg(1, 0x8000000000000000ULL); cpu.set_reg(2, 0xFFFFFFFFFFFFFFFFULL);
    check(cpu.step(rtype(1, 2, 0, 0x1E)) && cpu.lo() == 0x8000000000000000ULL && cpu.hi() == 0,
          "DDIV handles signed minimum divided by negative one without host overflow");
    cpu.reset(); cpu.set_reg(1, 123); cpu.set_reg(2, 0);
    check(cpu.step(rtype(1, 2, 0, 0x1F)) && cpu.lo() == 0xFFFFFFFFFFFFFFFFULL && cpu.hi() == 123,
          "DDIVU handles division by zero deterministically");

    cpu.reset(); cpu.set_reg(1, static_cast<std::uint64_t>(-3LL)); cpu.set_reg(2, static_cast<std::uint64_t>(-7LL));
    check(cpu.step(rtype(1, 2, 0, 0x1C)) && cpu.lo() == 21 && cpu.hi() == 0,
          "DMULT produces the correct positive product for two negative operands");
    cpu.reset(); cpu.set_reg(1, 3); cpu.set_reg(2, static_cast<std::uint64_t>(-7LL));
    check(cpu.step(rtype(1, 2, 0, 0x1C)) &&
              cpu.lo() == 0xFFFFFFFFFFFFFFEBULL &&
              cpu.hi() == 0xFFFFFFFFFFFFFFFFULL,
          "DMULT sign-corrects the high half when only the right operand is negative");
    cpu.reset(); cpu.set_reg(1, 0x8000000000000000ULL); cpu.set_reg(2, static_cast<std::uint64_t>(-1LL));
    check(cpu.step(rtype(1, 2, 0, 0x1C)) &&
              cpu.lo() == 0x8000000000000000ULL && cpu.hi() == 0,
          "DMULT handles the signed minimum multiplied by negative one");
    cpu.reset(); cpu.set_reg(1, 0x123456789ABCDEF0ULL); cpu.set_reg(2, 0x10);
    check(cpu.step(rtype(1, 2, 0, 0x1F)) && cpu.lo() == 0x0123456789ABCDEFULL && cpu.hi() == 0,
          "DDIVU divides a 64-bit dividend without truncating it to 32 bits");

    cpu.reset(); cpu.set_reg(1, 100); cpu.set_reg(2, static_cast<std::uint64_t>(-9LL));
    check(cpu.step(rtype(1, 2, 0, 0x1E)) && cpu.lo() == static_cast<std::uint64_t>(-11LL) && cpu.hi() == 1,
          "DDIV returns quotient and remainder with a negative divisor");
    cpu.reset(); cpu.set_reg(1, static_cast<std::uint64_t>(-123LL)); cpu.set_reg(2, 0);
    check(cpu.step(rtype(1, 2, 0, 0x1E)) && cpu.lo() == 1 && cpu.hi() == static_cast<std::uint64_t>(-123LL),
          "DDIV handles division by zero without invoking host division");

    // J/JAL take the high address region from PC+4, including the 256 MiB boundary.
    cpu.reset(); cpu.set_pc(0x0FFFFFFCULL);
    check(cpu.step((0x02U << 26U) | 0x00000003U), "J at a 256 MiB boundary schedules its delay slot");
    check(cpu.step(nop) && cpu.pc() == 0x1000000CULL,
          "J uses the upper address bits of PC+4 across the 256 MiB boundary");
    cpu.reset(); cpu.set_pc(0x8FFFFFFCULL);
    check(cpu.step((0x03U << 26U) | 0x00000002U), "JAL at a high-address boundary executes");
    check(cpu.step(nop) && cpu.pc() == 0x90000008ULL && cpu.reg(31) == 0x90000004ULL,
          "JAL preserves the high address region and writes PC+8 link");

    // An unaligned instruction fetch outside a delay slot records the faulting PC
    // and clears Cause.BD. set_pc intentionally clears any pending delay-slot state.
    std::vector<std::uint8_t> fetch_ram(0x200, 0);
    cpu.reset(); cpu.set_pc(0x105);
    error.clear();
    check(!cpu.step_from_ram(fetch_ram, 0, &error) && !error.empty(),
          "unaligned instruction fetch is rejected");
    check(cpu.pc() == Cpu::GeneralExceptionVector && cpu.cop0_epc() == 0x105 &&
          (cpu.cop0_cause() & Cpu::CauseBd) == 0U &&
          ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcAddressErrorLoad,
          "non-delay-slot fetch fault records faulting PC, clears Cause.BD, and raises AdEL");

    // Exercise the separate EE-memory instruction-fetch path as well as raw RAM.
    retroshelf::Ps2EeMemoryMap ee_memory(0x1000);
    error.clear();
    check(ee_memory.write(0x100, 4, addiu7, &error), "write instruction into EE mapped RAM");
    cpu.reset(); cpu.set_pc(0x100);
    check(cpu.step_from_ee_memory(ee_memory, &error) && cpu.reg(8) == 7,
          "EE-memory instruction fetch executes a mapped instruction");
    cpu.reset(); cpu.set_pc(0x101); error.clear();
    check(!cpu.step_from_ee_memory(ee_memory, &error) && !error.empty(),
          "EE-memory instruction fetch rejects an unaligned PC");
    check(cpu.pc() == Cpu::GeneralExceptionVector && cpu.cop0_epc() == 0x101 &&
          (cpu.cop0_cause() & Cpu::CauseBd) == 0U &&
          ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcAddressErrorLoad,
          "EE-memory unaligned fetch raises AdEL with correct EPC and Cause.BD");
    cpu.reset(); cpu.set_pc(0x100000000ULL); error.clear();
    check(!cpu.step_from_ee_memory(ee_memory, &error) && !error.empty(),
          "EE-memory fetch rejects addresses wider than 32 bits without truncation");
    check(cpu.pc() == Cpu::GeneralExceptionVector && cpu.cop0_epc() == 0x100000000ULL &&
          ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcAddressErrorLoad,
          "wide EE fetch fault enters AdEL exception path without truncating EPC");
    cpu.reset(); cpu.set_pc(0x30000000ULL); error.clear();
    check(!cpu.step_from_ee_memory(ee_memory, &error) && !error.empty(),
          "EE-memory fetch rejects unmapped guest address");
    check(cpu.pc() == Cpu::GeneralExceptionVector && cpu.cop0_epc() == 0x30000000ULL &&
          ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcAddressErrorLoad,
          "unmapped EE fetch enters AdEL exception path and records EPC");


    // EE-mapped byte/halfword loads and stores must use little-endian lanes,
    // extend signed values correctly, and preserve neighboring bytes.
    const auto run_ee_instruction = [&](std::uint32_t instruction) {
        error.clear();
        if (!ee_memory.write(0x300, 4, instruction, &error)) return false;
        cpu.set_pc(0x300);
        return cpu.step_from_ee_memory(ee_memory, &error);
    };
    check(ee_memory.write(0x200, 4, 0x80FF7F01U, &error),
          "seed EE RAM for byte and halfword tests");
    cpu.reset();
    cpu.set_reg(1, 0x200);
    check(run_ee_instruction(itype(0x20, 1, 2, 2)) &&
              cpu.reg(2) == 0xFFFFFFFFFFFFFFFFULL,
          "LB sign-extends a high-bit byte from EE RAM");
    check(run_ee_instruction(itype(0x24, 1, 3, 2)) && cpu.reg(3) == 0xFFU,
          "LBU zero-extends a high-bit byte from EE RAM");
    check(run_ee_instruction(itype(0x21, 1, 4, 2)) &&
              cpu.reg(4) == 0xFFFFFFFFFFFF80FFULL,
          "LH sign-extends a high-bit halfword from EE RAM");
    check(run_ee_instruction(itype(0x25, 1, 5, 2)) && cpu.reg(5) == 0x80FFU,
          "LHU zero-extends a high-bit halfword from EE RAM");

    cpu.set_reg(6, 0xAAU);
    check(run_ee_instruction(itype(0x28, 1, 6, 1)),
          "SB writes one byte into EE RAM");
    std::uint64_t memory_value = 0;
    check(ee_memory.read(0x200, 4, memory_value, &error) &&
              memory_value == 0x80FFAA01U,
          "SB preserves adjacent EE RAM bytes");
    check(ee_memory.write(0x200, 4, 0xAABBCCDDU, &error),
          "seed EE RAM for halfword store");
    cpu.set_reg(7, 0x1234U);
    check(run_ee_instruction(itype(0x29, 1, 7, 2)),
          "SH writes a halfword into EE RAM");
    check(ee_memory.read(0x200, 4, memory_value, &error) &&
              memory_value == 0x1234CCDDU,
          "SH writes little-endian halfword and preserves adjacent bytes");

    cpu.reset();
    cpu.set_reg(1, 0x200);
    error.clear();
    check(!run_ee_instruction(itype(0x21, 1, 8, 1)) && !error.empty(),
          "misaligned LH faults on EE RAM");
    check(cpu.pc() == Cpu::GeneralExceptionVector &&
              cpu.cop0_epc() == 0x300 &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcAddressErrorLoad,
          "misaligned LH raises AdEL with the instruction PC in EPC");

    check(cpu.cop0_bad_vaddr() == 0x201,
          "misaligned LH records its effective address in BadVAddr");
    const std::uint32_t mfc0_badvaddr =
        (0x10U << 26U) | (2U << 16U) | (8U << 11U);
    check(cpu.step(mfc0_badvaddr) && cpu.reg(2) == 0x201,
          "MFC0 reads BadVAddr after an address exception");

    // A nested exception while EXL is already set updates ExcCode but preserves
    // the original EPC and Cause.BD until the handler returns with ERET.
    cpu.reset(); cpu.set_pc(0x100); cpu.set_reg(1, 5); cpu.set_reg(2, 5);
    check(cpu.step(itype(0x04, 1, 2, 3)), "nested-exception test schedules a branch delay slot");
    check(!cpu.step(bad_lw, &error), "first exception occurs in the branch delay slot");
    check(cpu.cop0_epc() == 0x100 && (cpu.cop0_cause() & Cpu::CauseBd) != 0U,
          "first exception records branch EPC and Cause.BD");
    cpu.set_pc(0x80000200U);
    check(!cpu.step(0x0000000CU), "SYSCALL triggers a nested exception while EXL is set");
    check(cpu.cop0_epc() == 0x100 && (cpu.cop0_cause() & Cpu::CauseBd) != 0U,
          "nested exception preserves the original EPC and Cause.BD");
    check(((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcSyscall,
          "nested exception still updates Cause.ExcCode");
    cpu.set_reg(3, 0x80000300U);
    const std::uint32_t nested_mtc0_epc =
        (0x10U << 26U) | (0x04U << 21U) | (3U << 16U) | (14U << 11U);
    check(cpu.step(nested_mtc0_epc), "nested handler can update EPC before ERET");
    check(cpu.step((0x10U << 26U) | (0x10U << 21U) | 0x18U),
          "ERET returns from the nested exception");
    cpu.set_pc(0x500);
    check(!cpu.step(0x0000000CU), "a new exception occurs after ERET clears EXL");
    check(cpu.cop0_epc() == 0x500 && (cpu.cop0_cause() & Cpu::CauseBd) == 0U,
          "exception after ERET records a fresh EPC and clears stale Cause.BD");

    // Minimal COP0 state transfers and ERET allow an exception handler to return.
    cpu.reset();
    cpu.set_pc(0x400);
    const std::uint32_t mfc0_status = (0x10U << 26U) | (2U << 16U) | (12U << 11U);
    const std::uint32_t mtc0_epc = (0x10U << 26U) | (0x04U << 21U) | (3U << 16U) | (14U << 11U);
    const std::uint32_t eret = (0x10U << 26U) | (0x10U << 21U) | 0x18U;
    cpu.set_reg(3, 0x80000200U);
    check(cpu.step(mtc0_epc), "MTC0 writes EPC");
    check(cpu.cop0_epc() == 0x80000200U, "MTC0 EPC value is retained");
    cpu.set_reg(4, Cpu::StatusExl);
    check(cpu.step((0x10U << 26U) | (0x04U << 21U) | (4U << 16U) | (12U << 11U)),
          "MTC0 writes Status");
    check(cpu.step(mfc0_status) && cpu.reg(2) == Cpu::StatusExl,
          "MFC0 reads Status");
    check(cpu.step(eret), "ERET executes");
    check(cpu.pc() == 0x80000200U && cpu.next_pc() == 0x80000204U &&
              (cpu.cop0_status() & Cpu::StatusExl) == 0U && !cpu.in_delay_slot(),
          "ERET restores EPC and clears EXL");


    // EE doubleword loads/stores preserve all 64 bits and enforce alignment.
    check(ee_memory.write(0x208, 8, 0, &error),
          "initialize EE RAM for doubleword access tests");
    cpu.reset();
    cpu.set_reg(1, 0x200);
    cpu.set_reg(9, 0x0123456789ABCDEFULL);
    check(run_ee_instruction(itype(0x3F, 1, 9, 8)),
          "SD stores a doubleword into EE mapped RAM");
    check(ee_memory.read(0x208, 8, memory_value, &error) &&
              memory_value == 0x0123456789ABCDEFULL,
          "SD preserves the full 64-bit value in little-endian EE RAM");
    check(run_ee_instruction(itype(0x37, 1, 10, 8)) &&
              cpu.reg(10) == 0x0123456789ABCDEFULL,
          "LD reloads all 64 bits from EE mapped RAM");
    error.clear();
    check(!run_ee_instruction(itype(0x37, 1, 11, 1)) && !error.empty(),
          "misaligned LD faults on EE RAM");
    check(cpu.pc() == Cpu::GeneralExceptionVector &&
              cpu.cop0_epc() == 0x300 &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcAddressErrorLoad,
          "misaligned LD raises AdEL and records the instruction PC");
    cpu.reset();
    cpu.set_reg(1, 0x200);
    cpu.set_reg(12, 0xFEDCBA9876543210ULL);
    error.clear();
    check(!run_ee_instruction(itype(0x3F, 1, 12, 1)) && !error.empty(),
          "misaligned SD faults on EE RAM");
    check(cpu.pc() == Cpu::GeneralExceptionVector &&
              cpu.cop0_epc() == 0x300 &&
              ((cpu.cop0_cause() >> 2U) & 0x1FU) == Cpu::ExcAddressErrorStore,
          "misaligned SD raises AdES and records the instruction PC");


    // Little-endian unaligned word merges must preserve unaffected GPR/RAM lanes.
    const std::uint32_t lwl_expected[] = {
        0x11BBCCDDU, 0x2211CCDDU, 0x332211DDU, 0x44332211U
    };
    const std::uint32_t lwr_expected[] = {
        0x44332211U, 0xAA443322U, 0xAABB4433U, 0xAABBCC44U
    };
    for (std::uint32_t lane = 0; lane < 4; ++lane) {
        check(ee_memory.write(0x200, 4, 0x44332211U, &error),
              "seed EE RAM for LWL/LWR lane test");
        cpu.reset();
        cpu.set_reg(1, 0x200);
        cpu.set_reg(2, 0xAABBCCDDU);
        check(run_ee_instruction(itype(0x22, 1, 2, static_cast<std::int16_t>(lane))) &&
                  static_cast<std::uint32_t>(cpu.reg(2)) == lwl_expected[lane],
              "LWL merges the correct little-endian byte lane");
        cpu.reset();
        cpu.set_reg(1, 0x200);
        cpu.set_reg(2, 0xAABBCCDDU);
        check(run_ee_instruction(itype(0x26, 1, 2, static_cast<std::int16_t>(lane))) &&
                  static_cast<std::uint32_t>(cpu.reg(2)) == lwr_expected[lane],
              "LWR merges the correct little-endian byte lane");
    }

    const std::uint32_t swl_expected[] = {
        0x443322AAU, 0x4433AABBU, 0x44AABBCCU, 0xAABBCCDDU
    };
    const std::uint32_t swr_expected[] = {
        0xAABBCCDDU, 0xBBCCDD11U, 0xCCDD2211U, 0xDD332211U
    };
    for (std::uint32_t lane = 0; lane < 4; ++lane) {
        check(ee_memory.write(0x200, 4, 0x44332211U, &error),
              "seed EE RAM for SWL/SWR lane test");
        cpu.reset();
        cpu.set_reg(1, 0x200);
        cpu.set_reg(2, 0xAABBCCDDU);
        check(run_ee_instruction(itype(0x2A, 1, 2, static_cast<std::int16_t>(lane))) &&
                  ee_memory.read(0x200, 4, memory_value, &error) &&
                  memory_value == swl_expected[lane],
              "SWL writes the correct little-endian byte lane");
        check(ee_memory.write(0x200, 4, 0x44332211U, &error),
              "restore EE RAM for SWR lane test");
        cpu.reset();
        cpu.set_reg(1, 0x200);
        cpu.set_reg(2, 0xAABBCCDDU);
        check(run_ee_instruction(itype(0x2E, 1, 2, static_cast<std::int16_t>(lane))) &&
                  ee_memory.read(0x200, 4, memory_value, &error) &&
                  memory_value == swr_expected[lane],
              "SWR writes the correct little-endian byte lane");
    }


    // LQ/SQ transfer both 64-bit halves of the EE's 128-bit GPR at 16-byte alignment.
    cpu.reset();
    check(ee_memory.write128(0x220, 0x0123456789ABCDEFULL, 0xFEDCBA9876543210ULL, &error),
          "seed aligned 128-bit EE RAM");
    cpu.set_reg(1, 0x220);
    check(run_ee_instruction(itype(0x1E, 1, 2, 0)), "LQ executes for aligned EE RAM");
    check(cpu.reg(2) == 0x0123456789ABCDEFULL &&
              cpu.reg_high(2) == 0xFEDCBA9876543210ULL,
          "LQ loads low and high 64-bit GPR halves");
    cpu.set_reg(3, 0xAABBCCDDEEFF0011ULL);
    cpu.set_reg_high(3, 0x1122334455667788ULL);
    check(run_ee_instruction(itype(0x1F, 1, 3, 0)), "SQ executes for aligned EE RAM");
    std::uint64_t lq_low = 0, lq_high = 0;
    check(ee_memory.read128(0x220, lq_low, lq_high, &error) &&
              lq_low == 0xAABBCCDDEEFF0011ULL &&
              lq_high == 0x1122334455667788ULL,
          "SQ stores both 64-bit GPR halves in little-endian order");
    cpu.reset();
    cpu.set_reg(1, 0x220);
    error.clear();
    check(!run_ee_instruction(itype(0x1E, 1, 2, 8)) && !error.empty(),
          "misaligned LQ raises an address exception");
    check(((cpu.cop0_cause() >> 2U) & 0x1FU) == retroshelf::Cpu::ExcAddressErrorLoad,
          "misaligned LQ records AdEL");
    cpu.reset();
    cpu.set_reg(1, 0x220);
    error.clear();
    check(!run_ee_instruction(itype(0x1F, 1, 2, 8)) && !error.empty(),
          "misaligned SQ raises an address exception");
    check(((cpu.cop0_cause() >> 2U) & 0x1FU) == retroshelf::Cpu::ExcAddressErrorStore,
          "misaligned SQ records AdES");

    // LQ/SQ offsets are signed 16-bit immediates. Verify negative offsets
    // address the preceding quadword instead of becoming a large positive offset.
    cpu.reset();
    check(ee_memory.write128(0x220, 0x0123456789ABCDEFULL, 0xFEDCBA9876543210ULL, &error),
          "seed quadword for signed-offset LQ regression");
    cpu.set_reg(1, 0x230);
    check(run_ee_instruction(itype(0x1E, 1, 4, -16)),
          "LQ accepts a negative signed immediate");
    check(cpu.reg(4) == 0x0123456789ABCDEFULL &&
              cpu.reg_high(4) == 0xFEDCBA9876543210ULL,
          "LQ negative offset loads the preceding quadword");
    cpu.set_reg(5, 0xAABBCCDDEEFF0011ULL);
    cpu.set_reg_high(5, 0x1122334455667788ULL);
    check(run_ee_instruction(itype(0x1F, 1, 5, -16)),
          "SQ accepts a negative signed immediate");
    std::uint64_t signed_offset_low = 0, signed_offset_high = 0;
    check(ee_memory.read128(0x220, signed_offset_low, signed_offset_high, &error) &&
              signed_offset_low == 0xAABBCCDDEEFF0011ULL &&
              signed_offset_high == 0x1122334455667788ULL,
          "SQ negative offset writes the preceding quadword");
    cpu.set_reg(1, 0x230);
    check(run_ee_instruction(itype(0x1E, 1, 0, -16)) &&
              cpu.reg(0) == 0 && cpu.reg_high(0) == 0,
          "LQ targeting register zero preserves both zero-register halves");

    // Cause is mostly read-only: MTC0 may change only software interrupt-pending bits IP0/IP1.
    retroshelf::Cpu cause_cpu;
    check(!cause_cpu.step(0x0000000CU), "SYSCALL seeds read-only Cause exception-code bits");
    const std::uint32_t cause_before_write = cause_cpu.cop0_cause();
    cause_cpu.set_reg(5, 0xFFFFFFFFFFFFFFFFULL);
    const std::uint32_t mtc0_cause = (0x10U << 26U) | (0x04U << 21U) | (5U << 16U) | (13U << 11U);
    check(cause_cpu.step(mtc0_cause), "MTC0 Cause executes");
    check(cause_cpu.cop0_cause() == (cause_before_write | 0x00000300U),
          "MTC0 Cause changes only writable software interrupt-pending bits");
    cause_cpu.set_reg(5, 0);
    check(cause_cpu.step(mtc0_cause) && cause_cpu.cop0_cause() == (cause_before_write & ~0x00000300U),
          "MTC0 Cause clears software interrupt-pending bits without changing exception code");

    // Common COP0 setup registers can round-trip through MTC0/MFC0.
    retroshelf::Cpu cop0_bank_cpu;
    cop0_bank_cpu.set_reg(8, 0xC1234000U);
    check(cop0_bank_cpu.step((0x10U << 26U) | (4U << 21U) | (8U << 16U) | (10U << 11U)),
          "MTC0 writes EntryHi");
    check(cop0_bank_cpu.step((0x10U << 26U) | (9U << 16U) | (10U << 11U)) &&
              cop0_bank_cpu.reg(9) == 0xFFFFFFFFC1234000ULL,
          "MFC0 reads EntryHi with sign extension");
    cop0_bank_cpu.set_reg(8, 0x12345678U);
    check(cop0_bank_cpu.step((0x10U << 26U) | (4U << 21U) | (8U << 16U) | (9U << 11U)),
          "MTC0 writes the placeholder Count register");
    check(cop0_bank_cpu.step((0x10U << 26U) | (10U << 16U) | (9U << 11U)) &&
              cop0_bank_cpu.reg(10) == 0x12345678U,
          "MFC0 reads Count register storage");

    // Reset must clear the newly modelled COP0 bank as well as legacy state.
    cop0_bank_cpu.reset();
    check(cop0_bank_cpu.step((0x10U << 26U) | (9U << 16U) | (10U << 11U)) &&
              cop0_bank_cpu.reg(9) == 0,
          "CPU reset clears COP0 EntryHi storage");

    // BLEZ/BGTZ and their likely forms require the rt field to be zero.
    // Malformed encodings must raise RI rather than silently branch.
    for (const std::uint32_t opcode : {0x06U, 0x07U, 0x16U, 0x17U}) {
        retroshelf::Cpu reserved_branch_cpu;
        const std::uint32_t malformed = (opcode << 26U) | (1U << 16U);
        check(!reserved_branch_cpu.step(malformed),
              "BLEZ/BGTZ family rejects nonzero reserved rt field");
        check(((reserved_branch_cpu.cop0_cause() >> 2U) & 0x1FU) ==
                  retroshelf::Cpu::ExcReservedInstruction,
              "malformed BLEZ/BGTZ encoding raises Reserved Instruction");
    }

    // Malformed BLEZ-family words are reserved instructions even in a
    // delay slot; they must not be rejected as a valid nested branch.
    retroshelf::Cpu malformed_delay_cpu;
    constexpr std::uint32_t beq_always = (0x04U << 26U) | (1U << 21U) |
                                          (1U << 16U) | 1U;
    check(malformed_delay_cpu.step(beq_always), "branch setup for malformed delay-slot test");
    const std::uint32_t malformed_blez_delay = (0x06U << 26U) | (1U << 16U);
    check(!malformed_delay_cpu.step(malformed_blez_delay),
          "malformed BLEZ in delay slot is rejected");
    check(((malformed_delay_cpu.cop0_cause() >> 2U) & 0x1FU) ==
              retroshelf::Cpu::ExcReservedInstruction,
          "malformed BLEZ in delay slot raises Reserved Instruction");


    // Variable 64-bit shift amounts use only the low six bits.
    cpu.reset(); cpu.set_reg(1, 64); cpu.set_reg(2, 0x123456789ABCDEF0ULL);
    check(cpu.step(rtype(1, 2, 3, 0x14)) &&
              cpu.reg(3) == 0x123456789ABCDEF0ULL,
          "DSLLV masks a shift count of 64 to zero");
    cpu.reset(); cpu.set_reg(1, 65); cpu.set_reg(2, 3);
    check(cpu.step(rtype(1, 2, 3, 0x14)) && cpu.reg(3) == 6,
          "DSLLV masks a shift count of 65 to one");
    cpu.reset(); cpu.set_reg(1, 65); cpu.set_reg(2, 0x8000000000000000ULL);
    check(cpu.step(rtype(1, 2, 3, 0x17)) &&
              cpu.reg(3) == 0xC000000000000000ULL,
          "DSRAV masks a shift count of 65 to one and sign-fills");
    cpu.reset(); cpu.set_reg(2, 1);
    check(cpu.step((2U << 16U) | (3U << 11U) | (31U << 6U) | 0x3CU) &&
              cpu.reg(3) == 0x8000000000000000ULL,
          "DSLL32 supports the maximum shift amount of 63");
    cpu.reset(); cpu.set_reg(2, 0x8000000000000000ULL);
    check(cpu.step((2U << 16U) | (3U << 11U) | (31U << 6U) | 0x3EU) &&
              cpu.reg(3) == 1,
          "DSRL32 supports the maximum shift amount of 63");


    // Sign-extension and comparison edge cases used by ordinary EE integer code.
    cpu.reset();
    check(cpu.step(itype(0x09, 0, 6, -1)) &&
              cpu.reg(6) == 0xFFFFFFFFFFFFFFFFULL,
          "ADDIU sign-extends a negative immediate");
    cpu.reset();
    cpu.set_reg(1, 0x0000000080000000ULL);
    check(cpu.step(rtype(0, 1, 2, 0x00)) &&
              cpu.reg(2) == 0xFFFFFFFF80000000ULL,
          "SLL sign-extends a 32-bit result with bit 31 set");
    cpu.reset();
    cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL);
    cpu.set_reg(2, 1);
    check(cpu.step(rtype(1, 2, 3, 0x2A)) && cpu.reg(3) == 1,
          "SLT compares signed 64-bit register values");
    check(cpu.step(rtype(1, 2, 4, 0x2B)) && cpu.reg(4) == 0,
          "SLTU compares register values as unsigned 64-bit integers");

    // Immediate logical operations zero-extend their 16-bit operand, while LUI
    // constructs a 32-bit value and sign-extends it to the EE register width.
    // SLTI compares against a sign-extended immediate as a signed EE value.
    cpu.reset();
    cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL);
    check(cpu.step(itype(0x0A, 1, 2, 1)) && cpu.reg(2) == 1,
          "SLTI treats all-ones GPR value as signed negative one");
    cpu.reset();
    cpu.set_reg(1, 1);
    check(cpu.step(itype(0x0A, 1, 2, -1)) && cpu.reg(2) == 0,
          "SLTI sign-extends a negative immediate before signed comparison");

    // SLTIU sign-extends its immediate first, then performs an unsigned comparison.
    cpu.reset();
    cpu.set_reg(1, 0x7FFFFFFFFFFFFFFFULL);
    check(cpu.step(itype(0x0B, 1, 2, -1)) && cpu.reg(2) == 1,
          "SLTIU compares against the sign-extended all-ones immediate as unsigned");
    cpu.reset();
    cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL);
    check(cpu.step(itype(0x0B, 1, 2, -1)) && cpu.reg(2) == 0,
          "SLTIU equality with sign-extended negative immediate is false");
    cpu.reset();
    cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL);
    check(cpu.step(itype(0x0B, 1, 2, 1)) && cpu.reg(2) == 0,
          "SLTIU treats positive immediate as unsigned positive value");

    cpu.reset();
    cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL);
    check(cpu.step(itype(0x0C, 1, 2, static_cast<std::int16_t>(0x8001))) &&
              cpu.reg(2) == 0x8001ULL,
          "ANDI zero-extends its immediate and clears upper register bits");
    cpu.reset();
    cpu.set_reg(1, 0x1234000000000000ULL);
    check(cpu.step(itype(0x0D, 1, 2, static_cast<std::int16_t>(0xABCD))) &&
              cpu.reg(2) == 0x123400000000ABCDULL,
          "ORI zero-extends its immediate without changing upper bits");
    cpu.reset();
    cpu.set_reg(1, 0xFFFFFFFFFFFF0000ULL);
    check(cpu.step(itype(0x0E, 1, 2, static_cast<std::int16_t>(0x00FF))) &&
              cpu.reg(2) == 0xFFFFFFFFFFFF00FFULL,
          "XORI zero-extends its immediate");
    cpu.reset();
    check(cpu.step(itype(0x0F, 0, 2, static_cast<std::int16_t>(0x8000))) &&
              cpu.reg(2) == 0xFFFFFFFF80000000ULL,
          "LUI sign-extends the constructed 32-bit value");

    // Arithmetic-right-shift regression boundaries: sign propagation must
    // be explicit for 32-bit and 64-bit forms, including the +32 encoding.
    cpu.reset();
    cpu.set_reg(1, 0x80000001ULL);
    check(cpu.step((1U << 16U) | (2U << 11U) | (1U << 6U) | 0x03U) &&
              cpu.reg(2) == 0xFFFFFFFFC0000000ULL,
          "SRA sign-fills a negative 32-bit operand and sign-extends its result");
    cpu.reset();
    cpu.set_reg(1, 0xFFFFFFFF80000000ULL);
    cpu.set_reg(3, 1);
    check(cpu.step((3U << 21U) | (1U << 16U) | (2U << 11U) | 0x07U) &&
              cpu.reg(2) == 0xFFFFFFFFC0000000ULL,
          "SRAV uses the low five shift-count bits and preserves the sign");
    cpu.reset();
    cpu.set_reg(1, 0x8000000000000000ULL);
    check(cpu.step((1U << 16U) | (2U << 11U) | (4U << 6U) | 0x3BU) &&
              cpu.reg(2) == 0xF800000000000000ULL,
          "DSRA arithmetic-shifts a negative 64-bit operand");
    cpu.reset();
    cpu.set_reg(1, 0x8000000000000000ULL);
    check(cpu.step((1U << 16U) | (2U << 11U) | 0x3FU) &&
              cpu.reg(2) == 0xFFFFFFFF80000000ULL,
          "DSRA32 applies the encoded 32-bit shift and sign-fills");

    // MMI0 packed word arithmetic operates independently on all four
    // 32-bit lanes across the low/high halves of each 128-bit EE GPR.
    cpu.reset();
    cpu.set_reg(1, 0xFFFFFFFF00000001ULL);
    cpu.set_reg_high(1, 0x7FFFFFFF80000000ULL);
    cpu.set_reg(2, 0x0000000100000002ULL);
    cpu.set_reg_high(2, 0x0000000180000001ULL);
    const std::uint32_t paddw = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) | (3U << 11U) | 0x08U;
    check(cpu.step(paddw) &&
              cpu.reg(3) == 0x0000000000000003ULL &&
              cpu.reg_high(3) == 0x8000000000000001ULL,
          "PADDW wraps independently across all four 32-bit lanes");
    cpu.reset();
    cpu.set_reg(1, 0x0000000000000001ULL);
    cpu.set_reg_high(1, 0x8000000000000000ULL);
    cpu.set_reg(2, 0x0000000000000002ULL);
    cpu.set_reg_high(2, 0x0000000100000001ULL);
    const std::uint32_t psubw = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) | (3U << 11U) | (1U << 6U) | 0x08U;
    check(cpu.step(psubw) &&
              cpu.reg(3) == 0x00000000FFFFFFFFULL &&
              cpu.reg_high(3) == 0x7FFFFFFFFFFFFFFFULL,
          "PSUBW wraps independently across all four 32-bit lanes");
    // Signed packed greater-than compares each lane independently.
    cpu.reset();
    cpu.set_reg(1, 0x00000007FFFFFFFFULL);
    cpu.set_reg_high(1, 0x0000000080000000ULL);
    cpu.set_reg(2, 0x0000000500000000ULL);
    cpu.set_reg_high(2, 0x0000000000000001ULL);
    const std::uint32_t pcgtw = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) |
                                (3U << 11U) | (2U << 6U) | 0x08U;
    check(cpu.step(pcgtw) &&
              cpu.reg(3) == 0xFFFFFFFF00000000ULL &&
              cpu.reg_high(3) == 0x0000000000000000ULL,
          "PCGTW emits all-ones only for lanes with a greater signed operand");
    cpu.reset();
    cpu.set_reg(1, 0x0000000000018000ULL);
    cpu.set_reg(2, 0x0000000000000001ULL);
    const std::uint32_t pcgth = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) |
                                (3U << 11U) | (6U << 6U) | 0x08U;
    check(cpu.step(pcgth) && cpu.reg(3) == 0x00000000FFFF0000ULL,
          "PCGTH compares signed halfwords independently and recognizes a positive lane");
    cpu.reset();
    cpu.set_reg(1, 0x000000000000007FULL);
    cpu.set_reg(2, 0x0000000000000001ULL);
    const std::uint32_t pcgtb = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) |
                                (3U << 11U) | (10U << 6U) | 0x08U;
    check(cpu.step(pcgtb) && cpu.reg(3) == 0x00000000000000FFULL,
          "PCGTB emits an all-ones byte for a positive signed comparison");

    // Packed equality emits an all-ones lane for equality and zero otherwise.
    cpu.reset();
    cpu.set_reg(1, 0x0000000500000005ULL);
    cpu.set_reg_high(1, 0x0000000500000005ULL);
    cpu.set_reg(2, 0x0000000500000004ULL);
    cpu.set_reg_high(2, 0x0000000400000005ULL);
    const std::uint32_t pceqw = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) |
                                (3U << 11U) | (12U << 6U) | 0x08U;
    check(cpu.step(pceqw) &&
              cpu.reg(3) == 0x00000000FFFFFFFFULL &&
              cpu.reg_high(3) == 0x00000000FFFFFFFFULL,
          "PCEQW compares four independent 32-bit lanes across both GPR halves");
    cpu.reset();
    cpu.set_reg(1, 0x0002000100020001ULL);
    cpu.set_reg(2, 0x0002000000020001ULL);
    const std::uint32_t pceqh = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) |
                                (3U << 11U) | (14U << 6U) | 0x08U;
    check(cpu.step(pceqh) && cpu.reg(3) == 0xFFFF0000FFFFFFFFULL &&
              cpu.reg_high(3) == 0xFFFFFFFFFFFFFFFFULL,
          "PCEQH compares eight independent 16-bit lanes");
    cpu.reset();
    cpu.set_reg(1, 0x0000000004030201ULL);
    cpu.set_reg(2, 0x000000000403FF01ULL);
    const std::uint32_t pceqb = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) |
                                (3U << 11U) | (16U << 6U) | 0x08U;
    check(cpu.step(pceqb) && cpu.reg(3) == 0xFFFFFFFFFFFF00FFULL &&
              cpu.reg_high(3) == 0xFFFFFFFFFFFFFFFFULL,
          "PCEQB compares sixteen independent 8-bit lanes");

    // Byte packed operations use sixteen independent 8-bit lanes.
    cpu.reset();
    cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL);
    cpu.set_reg_high(1, 0x7F7F7F7F7F7F7F7FULL);
    cpu.set_reg(2, 0x0101010101010101ULL);
    cpu.set_reg_high(2, 0x0101010101010101ULL);
    const std::uint32_t paddb = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) |
                                (3U << 11U) | (8U << 6U) | 0x08U;
    check(cpu.step(paddb) &&
              cpu.reg(3) == 0x0000000000000000ULL &&
              cpu.reg_high(3) == 0x8080808080808080ULL,
          "PADDB wraps independently across sixteen 8-bit lanes");
    cpu.reset();
    cpu.set_reg(1, 0x0000000000000000ULL);
    cpu.set_reg_high(1, 0x8080808080808080ULL);
    cpu.set_reg(2, 0x0101010101010101ULL);
    cpu.set_reg_high(2, 0x0101010101010101ULL);
    const std::uint32_t psubb = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) |
                                (3U << 11U) | (9U << 6U) | 0x08U;
    check(cpu.step(psubb) &&
              cpu.reg(3) == 0xFFFFFFFFFFFFFFFFULL &&
              cpu.reg_high(3) == 0x7F7F7F7F7F7F7F7FULL,
          "PSUBB wraps independently across sixteen 8-bit lanes");

    // Halfword packed operations use eight independent 16-bit lanes.
    cpu.reset();
    cpu.set_reg(1, 0x7FFF8000FFFF0001ULL);
    cpu.set_reg_high(1, 0x0001000200030004ULL);
    cpu.set_reg(2, 0x0001FFFF00010002ULL);
    cpu.set_reg_high(2, 0x0001000200030004ULL);
    const std::uint32_t paddh = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) |
                                (3U << 11U) | (4U << 6U) | 0x08U;
    check(cpu.step(paddh) &&
              cpu.reg(3) == 0x80007FFF00000003ULL &&
              cpu.reg_high(3) == 0x0002000400060008ULL,
          "PADDH wraps independently across eight 16-bit lanes");
    cpu.reset();
    cpu.set_reg(1, 0x0001000000000000ULL);
    cpu.set_reg_high(1, 0x8000000000000000ULL);
    cpu.set_reg(2, 0x0002000100010001ULL);
    cpu.set_reg_high(2, 0x0001000100010001ULL);
    const std::uint32_t psubh = (0x1CU << 26U) | (1U << 21U) | (2U << 16U) |
                                (3U << 11U) | (5U << 6U) | 0x08U;
    check(cpu.step(psubh) &&
              cpu.reg(3) == 0xFFFFFFFFFFFFFFFFULL &&
              cpu.reg_high(3) == 0x7FFFFFFFFFFFFFFFULL,
          "PSUBH wraps independently across eight 16-bit lanes");

    cpu.reset();
    cpu.set_reg(1, 0xFFFFFFFFFFFFFFFFULL);
    cpu.set_reg_high(1, 0xFFFFFFFFFFFFFFFFULL);
    cpu.set_reg(2, 1U);
    cpu.set_reg_high(2, 1U);
    const std::uint32_t paddw_zero = (1U << 21U) | (2U << 16U) | 0x08U;
    check(cpu.step(paddw_zero) && cpu.reg(0) == 0U && cpu.reg_high(0) == 0U,
          "PADDW targeting GPR zero preserves both zero-register halves");

    // MMI primary opcode rejects unsupported sub-operations rather than
    // accidentally treating unrelated encodings as packed arithmetic.
    cpu.reset();
    check(!cpu.step((0x1CU << 26U) | (3U << 6U) | 0x08U),
          "unsupported MMI function encoding raises Reserved Instruction");

    return failures == 0 ? 0 : 1;
}

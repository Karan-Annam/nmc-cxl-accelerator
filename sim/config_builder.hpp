// Builds the 56-bit PE operation configuration word.
#pragma once
#include <cstdint>

// PE op codes (mirror nmc_pkg.sv)
enum PeOp : uint8_t {
    PE_ADD = 0x0, PE_SUB = 0x1, PE_MUL = 0x2, PE_MAX = 0x3,
    PE_MIN = 0x4, PE_AND = 0x5, PE_OR  = 0x6, PE_XOR = 0x7,
    PE_MACC = 0x8, PE_SACC = 0x9, PE_PASS_A = 0xA, PE_PASS_B = 0xB,
    PE_NEG = 0xC, PE_ABS = 0xD, PE_SHR = 0xE, PE_ZERO = 0xF
};

// operand-A source select
enum PeSrc : uint8_t {
    SRC_ACC = 0x0, SRC_SRAM_A = 0x1, SRC_SRAM_B = 0x2, SRC_ZERO = 0x3
};

class ConfigBuilder {
  public:
    ConfigBuilder() : word_(0) {}

    ConfigBuilder& set_pe(int pe, PeOp op, PeSrc src, bool mask_en = false) {
        uint64_t f = (uint64_t(op) & 0xF) | ((uint64_t(src) & 0x3) << 4) |
                     ((mask_en ? 1ull : 0ull) << 6);
        word_ &= ~(0x7Full << (pe * 7));
        word_ |= f << (pe * 7);
        return *this;
    }

    // common case: all 8 PEs identical
    ConfigBuilder& set_all(PeOp op, PeSrc src, bool mask_en = false) {
        for (int p = 0; p < 8; p++) set_pe(p, op, src, mask_en);
        return *this;
    }

    uint64_t build() const { return word_ & 0x00FFFFFFFFFFFFFFull; }

  private:
    uint64_t word_;
};

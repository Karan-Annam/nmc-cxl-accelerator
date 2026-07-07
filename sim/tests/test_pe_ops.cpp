// Each of the 16 PE ops produces the correct result through
// a full dense command; accumulator resets cleanly between commands.
#include <cstdint>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

namespace {
constexpr uint16_t SRC_A = 0;
constexpr uint16_t SRC_B = 16;
constexpr uint16_t DST   = 32;

int32_t ref_op(PeOp op, int32_t a, int32_t b) {
    switch (op) {
        case PE_ADD: return a + b;
        case PE_SUB: return a - b;
        case PE_MUL: return int32_t(int64_t(a) * int64_t(b));
        case PE_MAX: return a > b ? a : b;
        case PE_MIN: return a < b ? a : b;
        case PE_AND: return a & b;
        case PE_OR:  return a | b;
        case PE_XOR: return a ^ b;
        case PE_PASS_A: return a;
        case PE_PASS_B: return b;
        case PE_NEG: return -a;
        case PE_ABS: return a < 0 ? -a : a;
        case PE_SHR: return int32_t(uint32_t(a) >> (b & 31));
        case PE_ZERO: return 0;
        default: return 0;
    }
}
}  // namespace

TEST(test_pe_ops) {
    const std::vector<int32_t> va = {12, -7, 300, -4096, 55, 0, -1, 99};
    const std::vector<int32_t> vb = {3, 5, -2, 12, -55, 7, 1, 4};
    for (int i = 0; i < 8; i++) {
        host.mem_write(uint16_t(SRC_A + i), uint32_t(va[i]));
        host.mem_write(uint16_t(SRC_B + i), uint32_t(vb[i]));
    }

    // elementwise ops
    const PeOp elementwise[] = {PE_ADD, PE_SUB, PE_MUL, PE_MAX, PE_MIN, PE_AND,
                                PE_OR,  PE_XOR, PE_PASS_A, PE_PASS_B, PE_NEG,
                                PE_ABS, PE_SHR, PE_ZERO};
    for (PeOp op : elementwise) {
        host.download_config(ConfigBuilder().set_all(op, SRC_SRAM_A).build());
        host.submit_dense(OPC_DENSE, SRC_A, SRC_B, DST, 8);
        CHECK(host.wait_for_done());
        for (int i = 0; i < 8; i++) {
            int32_t got = int32_t(host.mem_read(uint16_t(DST + i)));
            int32_t exp = ref_op(op, va[i], vb[i]);
            if (got != exp)
                std::printf("    op=%d lane=%d\n", int(op), i);
            CHECK_EQ(got, exp);
        }
    }

    // MACC: dense dot product → scalar at DST
    int64_t dot = 0;
    for (int i = 0; i < 8; i++) dot += int64_t(va[i]) * vb[i];
    host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());
    host.submit_dense(OPC_DENSE, SRC_A, SRC_B, DST, 8);
    CHECK(host.wait_for_done());
    CHECK_EQ(int32_t(host.mem_read(DST)), int32_t(dot));

    // accumulator reset: run the same MACC again — identical result, no bleed
    host.submit_dense(OPC_DENSE, SRC_A, SRC_B, DST, 8);
    CHECK(host.wait_for_done());
    CHECK_EQ(int32_t(host.mem_read(DST)), int32_t(dot));

    // SACC: dense sum of A
    int64_t sum = 0;
    for (int i = 0; i < 8; i++) sum += va[i];
    host.download_config(ConfigBuilder().set_all(PE_SACC, SRC_SRAM_A).build());
    host.submit_dense(OPC_DENSE, SRC_A, SRC_B, DST, 8);
    CHECK(host.wait_for_done());
    CHECK_EQ(int32_t(host.mem_read(DST)), int32_t(sum));

    // MAX-accumulate (GNN max): fold max over A via src=ACC (b = A stream when
    // src_b points at the same array)
    int32_t mx = va[0];
    for (int i = 1; i < 8; i++) mx = va[i] > mx ? va[i] : mx;
    host.download_config(ConfigBuilder().set_all(PE_MAX, SRC_ACC).build());
    host.submit_dense(OPC_DENSE, SRC_A, SRC_A, DST, 8);
    CHECK(host.wait_for_done());
    CHECK_EQ(int32_t(host.mem_read(DST)), mx);
}

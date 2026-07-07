// Per-index sparse row dot products (the Q·K[j] pattern)
// against a C++ golden reference; reduction tree correctness; index masking.
#include <cstdint>
#include <random>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_sparse_dot) {
    std::mt19937 rng(7);
    std::uniform_int_distribution<int32_t> dist(-50, 50);

    const int rows = 32, dk = 16;
    const uint16_t k_base = 0;            // 32 rows × 16 = 512 words
    const uint16_t q_base = 1024;
    const uint16_t idx_base = 1100;
    const uint16_t dst = 1200;

    std::vector<int32_t> K(rows * dk), Q(dk);
    for (auto& v : K) v = dist(rng);
    for (auto& v : Q) v = dist(rng);
    for (int i = 0; i < rows * dk; i++)
        host.mem_write(uint16_t(k_base + i), uint32_t(K[i]));
    for (int d = 0; d < dk; d++) host.mem_write(uint16_t(q_base + d), uint32_t(Q[d]));

    const std::vector<uint32_t> idx = {3, 17, 29, 0, 31};
    for (size_t m = 0; m < idx.size(); m++)
        host.mem_write(uint16_t(idx_base + m), idx[m]);

    host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());
    host.submit_sparse(OPC_SPARSE, k_base, q_base, dst, idx_base,
                       uint16_t(idx.size()), dk);
    CHECK(host.wait_for_done());

    for (size_t m = 0; m < idx.size(); m++) {
        int64_t ref = 0;
        for (int d = 0; d < dk; d++) ref += int64_t(K[idx[m] * dk + d]) * Q[d];
        CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + m))), int32_t(ref));
    }

    // masked entries: with mask_en set, only MSB-tagged indices contribute;
    // a masked-out row produces score 0
    const std::vector<uint32_t> midx = {
        0x80000000u | 3,   // valid
        17,                // masked out (no MSB)
        0x80000000u | 29,  // valid
    };
    for (size_t m = 0; m < midx.size(); m++)
        host.mem_write(uint16_t(idx_base + m), midx[m]);
    host.download_config(
        ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A, /*mask_en=*/true).build());
    host.submit_sparse(OPC_SPARSE, k_base, q_base, dst, idx_base,
                       uint16_t(midx.size()), dk);
    CHECK(host.wait_for_done());

    for (size_t m = 0; m < midx.size(); m++) {
        int64_t ref = 0;
        if (midx[m] & 0x80000000u) {
            uint32_t j = midx[m] & 0x7FFFFFFFu;
            for (int d = 0; d < dk; d++) ref += int64_t(K[j * dk + d]) * Q[d];
        }
        CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + m))), int32_t(ref));
    }
}

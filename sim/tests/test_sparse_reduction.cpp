// Sparse sum (SACC) and sparse max (MAX) folds
// over an index list, through the reduction tree.
#include <random>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_sparse_reduction) {
    std::mt19937 rng(11);
    std::uniform_int_distribution<int32_t> dist(-10000, 10000);

    const uint16_t data_base = 0, idx_base = 512, dst = 600;
    std::vector<int32_t> data(64);
    for (auto& v : data) v = dist(rng);
    for (int i = 0; i < 64; i++)
        host.mem_write(uint16_t(data_base + i), uint32_t(data[i]));

    const std::vector<uint32_t> idx = {5, 63, 12, 0, 40, 33, 21, 8, 9, 10, 11};
    for (size_t m = 0; m < idx.size(); m++)
        host.mem_write(uint16_t(idx_base + m), idx[m]);

    // sparse sum
    int64_t sum = 0;
    for (uint32_t j : idx) sum += data[j];
    host.download_config(ConfigBuilder().set_all(PE_SACC, SRC_SRAM_A).build());
    host.submit_sparse(OPC_REDUCTION, data_base, 0, dst, idx_base,
                       uint16_t(idx.size()), 1);
    CHECK(host.wait_for_done());
    CHECK_EQ(int32_t(host.mem_read(dst)), int32_t(sum));

    // sparse max (MAX with operand_a = own accumulator)
    int32_t mx = data[idx[0]];
    for (uint32_t j : idx) mx = data[j] > mx ? data[j] : mx;
    host.download_config(ConfigBuilder().set_all(PE_MAX, SRC_ACC).build());
    host.submit_sparse(OPC_REDUCTION, data_base, 0, dst, idx_base,
                       uint16_t(idx.size()), 1);
    CHECK(host.wait_for_done());
    CHECK_EQ(int32_t(host.mem_read(dst)), mx);

    // all-negative max (exercises the INT_MIN accumulator identity)
    for (int i = 0; i < 8; i++)
        host.mem_write(uint16_t(data_base + 100 + i), uint32_t(-100 - i * 7));
    const std::vector<uint32_t> nidx = {100, 103, 107};
    for (size_t m = 0; m < nidx.size(); m++)
        host.mem_write(uint16_t(idx_base + m), nidx[m]);
    host.submit_sparse(OPC_REDUCTION, 0, 0, dst, idx_base, uint16_t(nidx.size()), 1);
    CHECK(host.wait_for_done());
    CHECK_EQ(int32_t(host.mem_read(dst)), -100);
}

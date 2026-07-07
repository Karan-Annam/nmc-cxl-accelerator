// PASS_A row gather from an embedding table;
// masked ids produce zero rows.
#include <random>
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_embedding_lookup) {
    std::mt19937 rng(23);
    std::uniform_int_distribution<int32_t> dist(-30000, 30000);

    const int rows = 64, dim = 8;
    const uint16_t tab = 0;               // 64×8 = 512 words
    const uint16_t idx_base = 1024, dst = 1100;

    std::vector<int32_t> table(rows * dim);
    for (auto& v : table) v = dist(rng);
    std::vector<uint32_t> raw(table.begin(), table.end());
    host.mem_write_burst(tab, raw);

    const std::vector<uint32_t> ids = {5, 12, 33, 60};
    for (size_t m = 0; m < ids.size(); m++)
        host.mem_write(uint16_t(idx_base + m), ids[m]);

    host.download_config(ConfigBuilder().set_all(PE_PASS_A, SRC_SRAM_A).build());
    host.submit_sparse(OPC_EMBEDDING, tab, 0, dst, idx_base, uint16_t(ids.size()),
                       dim);
    CHECK(host.wait_for_done());

    for (size_t m = 0; m < ids.size(); m++)
        for (int d = 0; d < dim; d++)
            CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + m * dim + d))),
                     table[ids[m] * dim + d]);

    // masked id → zero row
    const std::vector<uint32_t> mids = {0x80000000u | 7, 22 /* masked out */};
    for (size_t m = 0; m < mids.size(); m++)
        host.mem_write(uint16_t(idx_base + m), mids[m]);
    host.download_config(
        ConfigBuilder().set_all(PE_PASS_A, SRC_SRAM_A, /*mask_en=*/true).build());
    host.submit_sparse(OPC_EMBEDDING, tab, 0, dst, idx_base, uint16_t(mids.size()),
                       dim);
    CHECK(host.wait_for_done());
    for (int d = 0; d < dim; d++) {
        CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + d))), table[7 * dim + d]);
        CHECK_EQ(int32_t(host.mem_read(uint16_t(dst + dim + d))), 0);
    }
}

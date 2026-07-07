// Irregular index-driven gathers return exactly the right elements in list
// order, for any index pattern.
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_scatter_gather) {
    const uint16_t data_base = 0, idx_base = 512, dst = 1024;

    for (uint16_t i = 0; i < 32; i++) host.mem_write(uint16_t(data_base + i), i * 100u);

    // deliberately shuffled index pattern
    const std::vector<uint32_t> idx = {7, 2, 15, 0, 3, 11, 5, 9};
    for (size_t m = 0; m < idx.size(); m++)
        host.mem_write(uint16_t(idx_base + m), idx[m]);

    host.download_config(ConfigBuilder().set_all(PE_PASS_A, SRC_SRAM_A).build());
    host.submit_sparse(OPC_EMBEDDING, data_base, 0, dst, idx_base,
                       uint16_t(idx.size()), 1);
    CHECK(host.wait_for_done());

    const std::vector<uint32_t> expect = {700, 200, 1500, 0, 300, 1100, 500, 900};
    for (size_t m = 0; m < expect.size(); m++)
        CHECK_EQ(host.mem_read(uint16_t(dst + m)), expect[m]);

    // adversarial pattern: repeated indices, same-bank runs, descending
    const std::vector<uint32_t> idx2 = {8, 8, 16, 24, 31, 30, 29, 1, 1, 9};
    for (size_t m = 0; m < idx2.size(); m++)
        host.mem_write(uint16_t(idx_base + m), idx2[m]);
    host.submit_sparse(OPC_EMBEDDING, data_base, 0, dst, idx_base,
                       uint16_t(idx2.size()), 1);
    CHECK(host.wait_for_done());
    for (size_t m = 0; m < idx2.size(); m++)
        CHECK_EQ(host.mem_read(uint16_t(dst + m)), idx2[m] * 100u);
}

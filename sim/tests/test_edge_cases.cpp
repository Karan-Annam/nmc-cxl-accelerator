// test_edge_cases — len=1, unaligned lengths, empty index list, all-masked list,
// back-to-back commands with config change, invalid op, WGATHER stride cap,
// device ID sanity.
#include <vector>
#include "../config_builder.hpp"
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_edge_cases) {
    CHECK_EQ(host.mmio_read(REG_DEVICE_ID), 0xCA550001u);

    // len = 1 dense
    host.mem_write(0, 41);
    host.mem_write(8, 1);
    host.download_config(ConfigBuilder().set_all(PE_ADD, SRC_SRAM_A).build());
    host.submit_dense(OPC_DENSE, 0, 8, 16, 1);
    CHECK(host.wait_for_done());
    CHECK_EQ(host.mem_read(16), 42u);

    // len = 0 dense → immediate DONE, no writes
    host.mem_write(24, 0x77);
    host.submit_dense(OPC_DENSE, 0, 8, 24, 0);
    CHECK(host.wait_for_done());
    CHECK_EQ(host.mem_read(24), 0x77u);      // untouched

    // empty index list → immediate DONE
    host.submit_sparse(OPC_REDUCTION, 0, 0, 24, 100, 0, 1);
    CHECK(host.wait_for_done());
    CHECK_EQ(host.mem_read(24), 0x77u);

    // all-masked index list (SPARSE) → zeros written to every output slot
    for (int i = 0; i < 4; i++) host.mem_write(uint16_t(100 + i), uint32_t(i));
    host.mem_write(200, 5);
    host.mem_write(201, 5);
    host.download_config(
        ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A, /*mask_en=*/true).build());
    host.mem_write(300, 0xAAAA);
    host.mem_write(301, 0xBBBB);
    host.submit_sparse(OPC_SPARSE, 100, 200, 300, 100, 2, 1);
    CHECK(host.wait_for_done());
    CHECK_EQ(host.mem_read(300), 0u);
    CHECK_EQ(host.mem_read(301), 0u);

    // back-to-back commands with config change between them
    host.mem_write(0, 10);
    host.mem_write(8, 3);
    host.download_config(ConfigBuilder().set_all(PE_SUB, SRC_SRAM_A).build());
    host.submit_dense(OPC_DENSE, 0, 8, 16, 1);
    CHECK(host.wait_for_done());
    host.download_config(ConfigBuilder().set_all(PE_MUL, SRC_SRAM_A).build());
    host.submit_dense(OPC_DENSE, 0, 8, 17, 1);
    CHECK(host.wait_for_done());
    CHECK_EQ(int32_t(host.mem_read(16)), 7);
    CHECK_EQ(int32_t(host.mem_read(17)), 30);

    // invalid opcode → ERROR status + ERROR_CODE 1, then recovers
    host.mmio_write(REG_CMD_OP, 0xF);
    host.mmio_write(REG_CMD_SUBMIT, 1);
    CHECK(!host.wait_for_done());            // returns false on ST_ERROR
    CHECK_EQ(host.mmio_read(REG_ERROR_CODE), 1u);

    // WGATHER stride over the accumulator-vector cap → ERROR_CODE 2
    host.download_config(ConfigBuilder().set_all(PE_MACC, SRC_SRAM_A).build());
    host.mem_write(100, 0);
    host.submit_sparse(OPC_WGATHER, 0, 200, 400, 100, 1, 65);
    CHECK(!host.wait_for_done());
    CHECK_EQ(host.mmio_read(REG_ERROR_CODE), 2u);

    // device fully functional after both errors
    host.download_config(ConfigBuilder().set_all(PE_ADD, SRC_SRAM_A).build());
    host.submit_dense(OPC_DENSE, 0, 8, 18, 1);
    CHECK(host.wait_for_done());
    CHECK_EQ(int32_t(host.mem_read(18)), 13);

    // softmax len=0 → immediate done
    host.submit_softmax(0, 500, 0);
    CHECK(host.wait_for_done());
}

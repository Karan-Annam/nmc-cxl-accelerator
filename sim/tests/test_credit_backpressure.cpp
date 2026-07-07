// Credit flow control: (a) an honest host exhausts its credit
// pool and stalls until credit returns arrive; (b) a dishonest host that ignores
// credits hits the RTL rx-ready gate — stall, never data loss; recovery is
// lossless once the device drains.
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"
#include "../config_builder.hpp"

TEST(test_credit_backpressure) {
    // ---- (a) host-side credit exhaustion ----
    // block device→host flits so credit returns cannot arrive
    host.set_tx_ready(false);
    CHECK_EQ(host.mem_credits, INIT_CREDITS);
    for (int i = 0; i < INIT_CREDITS; i++)
        host.mem_write(uint16_t(1000 + i), uint32_t(0xC0DE0000 + i));
    CHECK_EQ(host.mem_credits, 0);          // pool fully consumed

    // re-open the return path: credits must flow back to full
    host.set_tx_ready(true);
    for (int guard = 0; guard < 500 && host.mem_credits < INIT_CREDITS; guard++)
        host.pump(1);
    CHECK_EQ(host.mem_credits, INIT_CREDITS);

    // nothing was lost
    for (int i = 0; i < INIT_CREDITS; i++)
        CHECK_EQ(host.mem_read(uint16_t(1000 + i)), uint32_t(0xC0DE0000 + i));

    // ---- (b) RTL-side ready gate under a credit-ignoring host ----
    // Occupy the engine so the MEM dispatcher cannot drain, then blast write
    // slots far beyond the rx queue capacity, ignoring credits on purpose.
    std::vector<uint32_t> arr(1024);
    for (size_t i = 0; i < arr.size(); i++) arr[i] = uint32_t(i);
    host.mem_write_burst(2000, arr);
    host.download_config(ConfigBuilder().set_all(PE_ADD, SRC_SRAM_A).build());
    host.submit_dense(OPC_DENSE, 2000, 2000, 4000, 1024);   // engine busy for a while

    bool saw_stall = false;
    for (int k = 0; k < 6; k++) {                            // 6 flits × 3 slots = 18
        CxlFlit f;
        for (int s = 0; s < 3; s++) {
            f.set_slot_type(1 + s, SLOT_MEM);
            f.set_slot_word(1 + s, 0, 0x80000000u | uint16_t(3000 + k * 3 + s));
            f.set_slot_word(1 + s, 1, uint32_t(0xBB000000 + k * 3 + s));
        }
        host.send_flit(f, false);            // dishonest: no credit accounting
        if (!host.rx_ready()) saw_stall = true;
    }
    std::printf("    rx_ready deasserted during blast: %s\n",
                saw_stall ? "yes" : "no");
    CHECK(saw_stall);                        // the gate actually engaged

    CHECK(host.wait_for_done());             // engine finishes on its own
    host.pump(200);                          // queues drain

    // stall, not drop: every blasted write must have landed, in order
    for (int i = 0; i < 18; i++)
        CHECK_EQ(host.mem_read(uint16_t(3000 + i)), uint32_t(0xBB000000 + i));
    // and the dense command results are intact too (spot check)
    CHECK_EQ(host.mem_read(4000), 0u);
    CHECK_EQ(host.mem_read(4100), 200u);
}

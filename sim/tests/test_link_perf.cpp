// Link-layer perf counters (cxl_link_perf.sv): error/retry events and
// flit/slot utilization are MMIO-visible, so link efficiency is a measured
// number instead of an inference. Also covers the commands_issued counter.
// Counters share the posted PERF_RESET with the engine set, so every window
// opens with the standard quiesce-then-poll-zero dance.
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

static void open_window(CxlHostModel& host) {
    host.pump(100);
    host.reset_perf();
    while (host.get_cxl_writes() != 0 || host.get_cxl_reads() != 0)
        host.pump(10);
}

TEST(test_link_perf) {
    // ---- clean traffic: no error events; flits + slots accumulate ----
    open_window(host);
    for (int i = 0; i < 16; i++) host.mem_write(uint16_t(900 + i), uint32_t(i * 5));
    for (int i = 0; i < 16; i++)
        CHECK_EQ(host.mem_read(uint16_t(900 + i)), uint32_t(i * 5));
    host.pump(50);
    CHECK_EQ(host.get_lnk(REG_LNK_CRC_ERRS), 0u);
    CHECK_EQ(host.get_lnk(REG_LNK_NAKS), 0u);
    CHECK_EQ(host.get_lnk(REG_LNK_RETRIES), 0u);
    uint32_t flits = host.get_lnk(REG_LNK_TX_FLITS);
    uint32_t slots = host.get_lnk(REG_LNK_TX_SLOTS);
    CHECK(flits > 0);
    CHECK(slots > 0);
    CHECK(slots <= 4 * flits);
    metric("link.slot_util_pct", 100.0 * double(slots) / (4.0 * double(flits)));

    // ---- pipelined read dispatch: burst reads sustain well under the old
    //      stop-and-wait cost (~1 full round trip per word) ----
    {
        std::vector<uint32_t> ref(60);
        for (size_t i = 0; i < ref.size(); i++) ref[i] = uint32_t(i * 7 + 1);
        host.mem_write_burst(1000, ref);
        std::vector<uint32_t> got;
        uint64_t c0 = host.stat_cycles;
        host.mem_read_burst(1000, got, ref.size());
        double cyc_per_word = double(host.stat_cycles - c0) / double(ref.size());
        for (size_t i = 0; i < ref.size(); i++) CHECK_EQ(got[i], ref[i]);
        metric("link.read_cyc_per_word", cyc_per_word);
        CHECK(cyc_per_word < 10.0);
    }

    // ---- a corrupted host flit: CRC error counted, NAK-sent counted ----
    host.inject_crc_error();
    host.mem_write(950, 0xDEADBEEF);
    host.pump(200);                                  // NAK + retransmit loop
    CHECK(host.get_lnk(REG_LNK_CRC_ERRS) >= 1);
    CHECK(host.get_lnk(REG_LNK_NAKS) >= 1);
    CHECK_EQ(host.mem_read(950), 0xDEADBEEFu);       // clean copy landed

    // ---- a host NAK: device retry-buffer replay counted ----
    CHECK_EQ(host.get_lnk(REG_LNK_RETRIES), 0u);
    host.mem_write(951, 7);
    CHECK_EQ(host.mem_read(951), 7u);
    host.inject_nak(host.last_rx.seq());
    CxlFlit rf;
    CHECK(host.recv_flit(rf, 5000));
    host.pump(100);                                  // drain the go-back-N window
    CHECK(host.get_lnk(REG_LNK_RETRIES) >= 1);

    // ---- commands_issued counts accepted CMD_SUBMITs ----
    open_window(host);
    CHECK_EQ(host.get_perf_cmds(), 0u);
    host.submit_dense(OPC_DENSE, 0, 0, 64, 8);
    CHECK(host.wait_for_done());
    host.submit_dense(OPC_DENSE, 0, 0, 64, 8);
    CHECK(host.wait_for_done());
    CHECK_EQ(host.get_perf_cmds(), 2u);

    // ---- PERF_RESET clears the link counters too ----
    open_window(host);
    CHECK_EQ(host.get_lnk(REG_LNK_CRC_ERRS), 0u);
    CHECK_EQ(host.get_lnk(REG_LNK_RETRIES), 0u);
}

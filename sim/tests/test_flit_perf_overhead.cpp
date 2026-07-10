// Secondary metric only: flit framing overhead and slot packing efficiency.
// Explicitly NOT the primary metric (that's CXL transaction count) — this just
// quantifies the framing tax.
#include <vector>
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_flit_perf_overhead) {
    // fixed framing overhead: header (2B) + CRC (2B) per 68B flit
    double framing_pct = 100.0 * 4.0 / 68.0;
    metric("flit.framing_overhead_pct", framing_pct);

    // packing efficiency, single writes vs burst (3 data slots per flit)
    uint32_t f0 = host.stat_flits_sent;
    for (int i = 0; i < 30; i++) host.mem_write(uint16_t(5000 + i), uint32_t(i));
    uint32_t singles = host.stat_flits_sent - f0;

    std::vector<uint32_t> blk(30);
    for (size_t i = 0; i < blk.size(); i++) blk[i] = uint32_t(i);
    f0 = host.stat_flits_sent;
    host.mem_write_burst(5100, blk);
    uint32_t bursts = host.stat_flits_sent - f0;

    std::printf("    30 writes: %u flits singly, %u flits bursted (%.0f%% framing"
                " tax per flit)\n", singles, bursts, framing_pct);
    metric("flit.singles_30wr", singles);
    metric("flit.burst_30wr", bursts);
    metric("flit.words_per_flit_burst", 30.0 / double(bursts));
    CHECK(bursts < singles);
    CHECK(bursts <= 4 + 2);       // ceil(30/9) plus possible standalone acks

    // all 30 landed either way
    for (int i = 0; i < 30; i++) {
        CHECK_EQ(host.mem_read(uint16_t(5000 + i)), uint32_t(i));
        CHECK_EQ(host.mem_read(uint16_t(5100 + i)), uint32_t(i));
    }
}

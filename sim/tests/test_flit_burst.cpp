// Burst transaction slots (word0 bit29 + count, see nmc_pkg.sv): up to 3
// sequential data words per 16-byte slot, tripling payload efficiency. The
// encoding is wire-compatible — is_burst=0 parses exactly as the original
// single-word format, and the legacy path is exercised in the same flits.
#include <vector>
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_flit_burst) {
    // ---- burst writes land: full slots, a partial tail, non-multiple base ----
    std::vector<uint32_t> data(7);
    for (size_t i = 0; i < data.size(); i++) data[i] = 0xB0000000u + uint32_t(i);
    host.mem_write_burst(6001, data);            // 3+3+1 words in one flit
    for (size_t i = 0; i < data.size(); i++)
        CHECK_EQ(host.mem_read(uint16_t(6001 + i)), data[i]);

    // ---- burst reads return exactly the requested words, in order ----
    std::vector<uint32_t> got;
    host.mem_read_burst(6001, got, data.size());
    CHECK_EQ(got.size(), data.size());
    for (size_t i = 0; i < data.size(); i++) CHECK_EQ(got[i], data[i]);

    // ---- 1- and 2-word bursts (tail counts) ----
    std::vector<uint32_t> two = {0xAA55AA55u, 0x55AA55AAu};
    host.mem_write_burst(6100, two);
    host.mem_read_burst(6100, got, 2);
    CHECK_EQ(got[0], two[0]);
    CHECK_EQ(got[1], two[1]);
    std::vector<uint32_t> one = {0x12345678u};
    host.mem_write_burst(6200, one);
    CHECK_EQ(host.mem_read(6200), one[0]);

    // ---- mixed flit: legacy single-word slot + burst slot side by side ----
    {
        CxlFlit f;
        f.set_slot_type(1, SLOT_MEM);            // legacy single write
        f.set_slot_word(1, 0, 0x80000000u | 6300u);
        f.set_slot_word(1, 1, 0xC0FFEE00u);
        uint32_t burst[3] = {1u, 2u, 3u};
        f.set_burst_write(2, 6310, burst, 3);    // burst write beside it
        host.send_flit(f, true, 0, 2);           // send_flit waits for credits
        host.pump(50);
        CHECK_EQ(host.mem_read(6300), 0xC0FFEE00u);
        CHECK_EQ(host.mem_read(6310), 1u);
        CHECK_EQ(host.mem_read(6311), 2u);
        CHECK_EQ(host.mem_read(6312), 3u);
    }

    // ---- legacy single reads still work interleaved with burst traffic ----
    host.mem_read_burst(6310, got, 3);
    CHECK_EQ(got[0], 1u);
    CHECK_EQ(host.mem_read(6311), 2u);   // legacy encoding of the same data
    CHECK_EQ(got[2], 3u);

    // ---- larger block round-trip through both burst paths ----
    std::vector<uint32_t> blk(50), rd;
    for (size_t i = 0; i < blk.size(); i++) blk[i] = uint32_t(i * 977 + 13);
    host.mem_write_burst(6400, blk);
    host.mem_read_burst(6400, rd, blk.size());
    for (size_t i = 0; i < blk.size(); i++) CHECK_EQ(rd[i], blk[i]);
}

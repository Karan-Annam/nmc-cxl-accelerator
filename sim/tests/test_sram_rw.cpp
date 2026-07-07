// HDM word writes/reads through the full flit path,
// bank isolation, no aliasing across banks.
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_sram_rw) {
    // pattern write/read across the first 8 addresses (one per bank)
    for (uint16_t i = 0; i < 8; i++) host.mem_write(i, 0xDEAD0000u + i);
    for (uint16_t i = 0; i < 8; i++) CHECK_EQ(host.mem_read(i), 0xDEAD0000u + i);

    // same bank, different offsets (bank 0: addresses 0, 8, 16)
    host.mem_write(0, 111);
    host.mem_write(8, 222);
    host.mem_write(16, 333);
    CHECK_EQ(host.mem_read(0), 111u);
    CHECK_EQ(host.mem_read(8), 222u);
    CHECK_EQ(host.mem_read(16), 333u);

    // no aliasing across a wider range (burst path)
    std::vector<uint32_t> blk(64);
    for (size_t i = 0; i < blk.size(); i++) blk[i] = 0xA5000000u + uint32_t(i * 7);
    host.mem_write_burst(100, blk);
    std::vector<uint32_t> rd;
    host.mem_read_burst(100, rd, blk.size());
    for (size_t i = 0; i < blk.size(); i++) CHECK_EQ(rd[i], blk[i]);

    // high address range
    host.mem_write(65535, 0xCAFEBABE);
    CHECK_EQ(host.mem_read(65535), 0xCAFEBABEu);
}

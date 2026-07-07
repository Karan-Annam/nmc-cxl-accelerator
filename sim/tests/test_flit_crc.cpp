// A corrupted host flit is detected by the device's CRC
// checker and NAKed (not applied); the host retransmits and the data lands. Clean
// flits never false-positive.
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_flit_crc) {
    host.mem_write(700, 0x11111111);           // known-good baseline
    uint32_t naks0 = host.stat_naks_from_dev;
    uint32_t res0  = host.stat_resends;

    // corrupt exactly one flit carrying a write of new data
    host.inject_crc_error();
    host.mem_write(700, 0x22222222);
    // give the NAK + retransmit loop time to complete
    host.pump(200);

    CHECK_EQ(host.stat_naks_from_dev, naks0 + 1);   // device saw the corruption
    CHECK_EQ(host.stat_resends, res0 + 1);          // host retransmitted a clean copy
    CHECK_EQ(host.mem_read(700), 0x22222222u);      // the retransmit was applied

    // clean traffic afterwards: no spurious NAKs
    uint32_t naks1 = host.stat_naks_from_dev;
    for (int i = 0; i < 10; i++) host.mem_write(uint16_t(710 + i), uint32_t(i * 3));
    for (int i = 0; i < 10; i++)
        CHECK_EQ(host.mem_read(uint16_t(710 + i)), uint32_t(i * 3));
    host.pump(100);
    CHECK_EQ(host.stat_naks_from_dev, naks1);
}

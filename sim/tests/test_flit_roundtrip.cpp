// Transactions survive flit pack → wire → unpack
// in both directions; multi-slot flits dispatch correctly; response slots carry
// the is_response bit, echoed tag, and correct data; device flits CRC-verify
// against the host's independent golden CRC.
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_flit_roundtrip) {
    // simple transaction-level round trip (each call is one flit each way)
    CHECK_EQ(host.mmio_read(REG_DEVICE_ID), 0xCA550001u);
    host.mem_write(500, 0x0BADF00D);
    CHECK_EQ(host.mem_read(500), 0x0BADF00Du);

    // hand-crafted multi-slot flit: 1 ctrl-ack + 2 MEM writes + 1 MEM read
    uint32_t sent_before = host.stat_flits_sent;
    CxlFlit f;
    f.set_slot_type(1, SLOT_MEM);
    f.set_slot_word(1, 0, 0x80000000u | 600);
    f.set_slot_word(1, 1, 111111);
    f.set_slot_type(2, SLOT_MEM);
    f.set_slot_word(2, 0, 0x80000000u | 601);
    f.set_slot_word(2, 1, 222222);
    f.set_slot_type(3, SLOT_MEM);
    f.set_slot_word(3, 0, 600u | (77u << 16));    // read addr 600, tag 77
    host.send_flit(f, true, 0, 3);
    CHECK_EQ(host.stat_flits_sent, sent_before + 1);   // one flit, three transactions

    // the read response must come back with tag 77 and the just-written data
    CxlFlit rf;
    bool got = false;
    for (int tries = 0; tries < 50 && !got; tries++) {
        if (!host.recv_flit(rf, 2000)) break;
        for (int s = 0; s < SLOTS_PER_FLIT; s++) {
            if (rf.slot_type(s) == SLOT_MEM && (rf.slot_word(s, 0) & 0x40000000u)) {
                CHECK_EQ((rf.slot_word(s, 0) >> 16) & 0xFF, 77u);
                CHECK_EQ(rf.slot_word(s, 1), 111111u);
                got = true;
            }
        }
    }
    CHECK(got);

    // device flit passed the host's independent CRC check (recv_flit drops bad CRC)
    CHECK(rf.crc_ok());

    // both writes landed (order preserved within the flit)
    CHECK_EQ(host.mem_read(600), 111111u);
    CHECK_EQ(host.mem_read(601), 222222u);

    CHECK(host.stat_flits_rcvd > 0);
}

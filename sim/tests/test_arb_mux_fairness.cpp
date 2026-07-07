// Concurrent CXL.io and CXL.mem traffic both
// make progress; the ARB/MUX interleaves response slots without starving either
// protocol and without cross-protocol data corruption.
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_arb_mux_fairness) {
    for (int i = 0; i < 8; i++) host.mem_write(uint16_t(900 + i), 0x9000u + i);

    // queue a backlog of MEM reads and one IO read in back-to-back flits,
    // then watch the response order.
    CxlFlit fm;
    fm.set_slot_type(1, SLOT_MEM);
    fm.set_slot_word(1, 0, 900u | (210u << 16));
    fm.set_slot_type(2, SLOT_MEM);
    fm.set_slot_word(2, 0, 901u | (211u << 16));
    fm.set_slot_type(3, SLOT_MEM);
    fm.set_slot_word(3, 0, 902u | (212u << 16));
    host.send_flit(fm, true, 0, 3);

    CxlFlit fi;
    fi.set_slot_type(1, SLOT_IO);
    fi.set_slot_word(1, 0, uint32_t(REG_DEVICE_ID) | (220u << 16));
    host.send_flit(fi, true, 1, 0);

    // harvest until all four responses are seen; record arrival positions
    int pos = 0;
    int io_pos = -1, last_mem_pos = -1;
    uint32_t vals[4] = {0, 0, 0, 0};
    bool seen[4] = {false, false, false, false};
    for (int guard = 0; guard < 200 && !(seen[0] && seen[1] && seen[2] && seen[3]);
         guard++) {
        CxlFlit rf;
        if (!host.recv_flit(rf, 2000)) break;
        for (int s = 0; s < SLOTS_PER_FLIT; s++) {
            uint8_t t = rf.slot_type(s);
            if ((t == SLOT_MEM || t == SLOT_IO) &&
                (rf.slot_word(s, 0) & 0x40000000u)) {
                uint32_t tag = (rf.slot_word(s, 0) >> 16) & 0xFF;
                pos++;
                if (tag >= 210 && tag <= 212) {
                    seen[tag - 210] = true;
                    vals[tag - 210] = rf.slot_word(s, 1);
                    last_mem_pos = pos;
                    CHECK_EQ(int(t), int(SLOT_MEM));   // no cross-protocol mixup
                } else if (tag == 220) {
                    seen[3] = true;
                    vals[3] = rf.slot_word(s, 1);
                    io_pos = pos;
                    CHECK_EQ(int(t), int(SLOT_IO));
                }
            }
        }
    }
    CHECK(seen[0] && seen[1] && seen[2] && seen[3]);

    // data integrity per protocol
    CHECK_EQ(vals[0], 0x9000u);
    CHECK_EQ(vals[1], 0x9001u);
    CHECK_EQ(vals[2], 0x9002u);
    CHECK_EQ(vals[3], 0xCA550001u);

    // fairness: the IO response is not starved behind the whole MEM backlog
    std::printf("    io response position %d, last mem position %d\n", io_pos,
                last_mem_pos);
    CHECK(io_pos > 0);
    CHECK(io_pos <= last_mem_pos);

    // sustained mixed traffic: interleave many of both, all must complete
    for (int i = 0; i < 16; i++) {
        CHECK_EQ(host.mem_read(uint16_t(900 + (i % 8))), 0x9000u + (i % 8));
        CHECK_EQ(host.mmio_read(REG_DEVICE_ID), 0xCA550001u);
    }
}

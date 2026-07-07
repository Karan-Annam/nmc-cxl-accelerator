// A host NAK forces the device retry buffer to replay
// from the requested sequence number (go-back-N); the replayed flit is bit-usable
// (correct data re-delivered) and the link keeps working afterwards.
#include "../cxl_host_model.hpp"
#include "../test_runner.hpp"

TEST(test_flit_retry) {
    // generate a device response flit whose payload we know
    host.mem_write(800, 0x5EC0FFEE);
    CHECK_EQ(host.mem_read(800), 0x5EC0FFEEu);
    uint8_t seq_of_resp = host.last_rx.seq();

    // pretend that response flit was lost/corrupt on our side: demand replay
    uint32_t rcvd0 = host.stat_flits_rcvd;
    host.inject_nak(seq_of_resp);

    // the device must retransmit the flit with the SAME sequence number
    CxlFlit rf;
    CHECK(host.recv_flit(rf, 5000));
    CHECK_EQ(rf.seq(), seq_of_resp);

    // and it must carry the same response payload
    bool found = false;
    for (int s = 0; s < SLOTS_PER_FLIT; s++) {
        if (rf.slot_type(s) == SLOT_MEM && (rf.slot_word(s, 0) & 0x40000000u)) {
            CHECK_EQ(rf.slot_word(s, 1), 0x5EC0FFEEu);
            found = true;
        }
    }
    CHECK(found);
    CHECK(host.stat_flits_rcvd > rcvd0);

    // link is healthy after the replay window drains
    host.pump(100);
    host.mem_write(801, 42);
    CHECK_EQ(host.mem_read(801), 42u);
    CHECK_EQ(host.mem_read(800), 0x5EC0FFEEu);
}

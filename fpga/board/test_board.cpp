// Board-chain smoke test, run under Verilator BEFORE hardware exists:
// bit-banged UART bytes -> uart_flit_bridge -> nmc_top (full CXL link layer)
// -> response flits back over UART. Proves the exact bitstream datapath
// (minus the MMCM, which is bypassed under `VERILATOR`) so bring-up on a
// real Arty A7 starts from a known-good chain.
//
// Checks: DEVICE_ID MMIO read, HDM write/readback, device CRC validity,
// and device sequence-number continuity across the frames received.
#include <cstdio>
#include <cstdint>
#include <deque>
#include <vector>
#include "Vfpga_top.h"
#include "verilated.h"
#include "../../sim/cxl_host_model.hpp"   // CxlFlit (header-only helper)

double sc_time_stamp() { return 0; }   // legacy Verilator runtime hook

static Vfpga_top* top;
static int fails = 0;
#define CHECKB(c, msg) do { if (!(c)) { std::printf("    CHECK failed: %s\n", msg); fails++; } } while (0)

// must match the -GUART_DIVISOR override in the build script
static const int DIV = 16;

// software UART endpoint watching/driving the serial pins each core cycle
struct SwUart {
    // rx sampler (device -> host)
    int  rx_state = 0, rx_cnt = 0, rx_bit = 0;
    uint8_t rx_sh = 0;
    std::deque<uint8_t> rx_bytes;
    void sample(int txd) {
        switch (rx_state) {
            case 0: if (!txd) { rx_state = 1; rx_cnt = DIV / 2; } break;
            case 1: if (--rx_cnt == 0) { rx_state = 2; rx_cnt = DIV; rx_bit = 0; rx_sh = 0; } break;
            case 2: if (--rx_cnt == 0) {
                        rx_sh = uint8_t((rx_sh >> 1) | (txd ? 0x80 : 0));
                        rx_cnt = DIV;
                        if (++rx_bit == 8) rx_state = 3;
                    } break;
            case 3: if (--rx_cnt == 0) { if (txd) rx_bytes.push_back(rx_sh); rx_state = 0; } break;
        }
    }
    // tx driver (host -> device): queue of bit periods
    std::deque<int> tx_bits;
    int tx_cnt = 0, tx_cur = 1;
    void queue_byte(uint8_t b) {
        tx_bits.push_back(0);                                   // start
        for (int i = 0; i < 8; i++) tx_bits.push_back((b >> i) & 1);
        tx_bits.push_back(1);                                   // stop
        tx_bits.push_back(1);                                   // idle gap
    }
    int drive() {
        if (tx_cnt == 0) {
            if (!tx_bits.empty()) { tx_cur = tx_bits.front(); tx_bits.pop_front(); tx_cnt = DIV; }
            else tx_cur = 1;
        }
        if (tx_cnt > 0) tx_cnt--;
        return tx_cur;
    }
};

static SwUart uart;

static void tick() {
    top->uart_rx = uart.drive() ? 1 : 0;
    top->clk100 = 0; top->eval();
    top->clk100 = 1; top->eval();
    uart.sample(top->uart_tx);
}

static void send_flit(CxlFlit& f, uint8_t seq) {
    f.set_seq(seq);
    f.seal();
    uart.queue_byte(0xA5);
    uart.queue_byte(0x5A);
    for (int k = 0; k < FLIT_BYTES; k++) uart.queue_byte(f.bytes[k]);
}

// gather device frames (preamble + 68 bytes) out of the rx byte stream
static bool next_frame(CxlFlit& f, int max_cycles) {
    std::vector<uint8_t> buf;
    int hunting = 0;   // 0: want A5, 1: want 5A, 2: body
    for (int c = 0; c < max_cycles; c++) {
        tick();
        while (!uart.rx_bytes.empty()) {
            uint8_t b = uart.rx_bytes.front(); uart.rx_bytes.pop_front();
            if (hunting == 0)      { if (b == 0xA5) hunting = 1; }
            else if (hunting == 1) { hunting = (b == 0x5A) ? 2 : (b == 0xA5 ? 1 : 0); }
            else {
                buf.push_back(b);
                if (int(buf.size()) == FLIT_BYTES) {
                    std::memcpy(f.bytes, buf.data(), FLIT_BYTES);
                    return true;
                }
            }
        }
    }
    return false;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vfpga_top;

    // reset
    top->ck_rstn = 0;
    top->uart_rx = 1;
    for (int i = 0; i < 20; i++) tick();
    top->ck_rstn = 1;
    for (int i = 0; i < 20; i++) tick();

    uint8_t seq = 0;
    int expect_dev_seq = 0;
    bool seq_ok = true, crc_ok_all = true;

    // harvest device frames until the tagged response shows up (ctrl-only
    // ack/credit frames in between are counted but carry no response)
    auto harvest = [&](uint32_t want_tag, uint32_t* got) {
        for (int i = 0; i < 8; i++) {
            CxlFlit rf;
            if (!next_frame(rf, 400000)) return;
            crc_ok_all = crc_ok_all && rf.crc_ok();
            seq_ok = seq_ok && (rf.seq() == (expect_dev_seq & 0xF));
            expect_dev_seq++;
            bool found = false;
            for (int s = 0; s < SLOTS_PER_FLIT; s++) {
                uint8_t t = rf.slot_type(s);
                if ((t == SLOT_IO || t == SLOT_MEM) && (rf.slot_word(s, 0) & 0x40000000u) &&
                    ((rf.slot_word(s, 0) >> 16) & 0xFF) == want_tag) {
                    *got = rf.slot_word(s, 1);
                    found = true;
                }
            }
            if (found) return;
        }
    };

    // 1. MMIO read of DEVICE_ID (tag 0x42) — expect ack ctrl flit + response
    std::printf("[ RUN  ] board chain: DEVICE_ID over UART\n");
    CxlFlit f1;
    f1.set_slot_type(1, SLOT_IO);
    f1.set_slot_word(1, 0, uint32_t(REG_DEVICE_ID) | (0x42u << 16));
    send_flit(f1, seq++);
    uint32_t id = 0;
    harvest(0x42, &id);
    CHECKB(id == 0xCA550001u, "DEVICE_ID != 0xCA550001");

    // 2. HDM write then readback (tag 0x43)
    std::printf("[ RUN  ] board chain: HDM write/readback over UART\n");
    CxlFlit f2;
    f2.set_slot_type(1, SLOT_MEM);
    f2.set_slot_word(1, 0, 0x80000000u | 100u);
    f2.set_slot_word(1, 1, 0xDEADBEEFu);
    send_flit(f2, seq++);
    CxlFlit f3;
    f3.set_slot_type(1, SLOT_MEM);
    f3.set_slot_word(1, 0, 100u | (0x43u << 16));
    send_flit(f3, seq++);
    uint32_t rd = 0;
    harvest(0x43, &rd);
    CHECKB(rd == 0xDEADBEEFu, "HDM readback != 0xDEADBEEF");

    CHECKB(crc_ok_all, "a device flit failed the host CRC check");
    CHECKB(seq_ok, "device sequence numbers not contiguous");

    if (fails == 0) std::printf("[ PASS ] board chain (%d frames)\n", expect_dev_seq);
    else            std::printf("[ FAIL ] board chain (%d checks)\n", fails);
    delete top;
    return fails ? 1 : 0;
}

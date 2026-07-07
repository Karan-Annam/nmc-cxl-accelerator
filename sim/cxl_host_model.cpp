// cxl_host_model.cpp — implementation. See header for the design notes.
#include "cxl_host_model.hpp"
#include <algorithm>
#include <cstdio>
#include <stdexcept>

CxlHostModel::CxlHostModel(DutPins pins) : p_(std::move(pins)) {
    p_.drive_rx_valid(0);
    p_.drive_tx_ready(1);
}

// ------------------------------------------------------------------
// rx processing
// ------------------------------------------------------------------
void CxlHostModel::process_rx(const CxlFlit& f) {
    if (!f.crc_ok()) {
        // Should not happen: the sim wire is ideal for device→host. Drop.
        return;
    }
    if (f.seq() != expected_rx_seq_) {
        stat_dup_rcvd++;      // duplicate from a replay window; drop
        return;
    }
    expected_rx_seq_ = uint8_t((expected_rx_seq_ + 1) & 0xF);
    last_dev_seq     = f.seq();
    last_rx          = f;
    stat_flits_rcvd++;
    rcvd_since_ack_++;

    for (int s = 0; s < SLOTS_PER_FLIT; s++) {
        uint8_t t = f.slot_type(s);
        if (t == SLOT_CTRL) {
            uint32_t w0 = f.slot_word(s, 0);
            uint8_t flags = uint8_t(w0 & 0xFF);
            uint8_t seq   = uint8_t((w0 >> 8) & 0xF);
            // cap at pool size: replayed ctrl slots must not double-count returns
            io_credits  = std::min(io_credits + int((w0 >> 16) & 0xFF), INIT_CREDITS);
            mem_credits = std::min(mem_credits + int((w0 >> 24) & 0xFF), INIT_CREDITS);
            if (flags & 0x2) {          // device NAK: our flit was corrupt
                stat_naks_from_dev++;
                (void)seq;
                if (have_last_sent_) {  // retransmit a clean copy
                    CxlFlit copy = last_sent_;
                    copy.seal();
                    stat_resends++;
                    // raw resend, no new credits consumed (same transactions)
                    send_flit(copy, false);
                }
            }
            // flags & 0x1 (device acks our flits): informational only — the host
            // is software; it does not need a hardware retry window.
        } else if (t == SLOT_IO || t == SLOT_MEM) {
            uint32_t w0 = f.slot_word(s, 0);
            if (w0 & 0x40000000u) {     // response
                uint8_t tag = uint8_t((w0 >> 16) & 0xFF);
                responses_[tag] = f.slot_word(s, 1);
                if (t == SLOT_IO) stat_io_slots_rx++;
                else              stat_mem_slots_rx++;
            }
        }
    }
}

void CxlHostModel::pump(int cycles) {
    for (int c = 0; c < cycles; c++) {
        bool have = p_.sample_tx_valid() && tx_ready_;
        CxlFlit f;
        if (have)
            p_.sample_tx_data([&](uint8_t* raw) { std::memcpy(f.bytes, raw, FLIT_BYTES); });
        p_.tick();
        if (have) process_rx(f);
    }
    // standalone ack: keep the device retry window from filling while we idle
    if (rcvd_since_ack_ >= 4) {
        CxlFlit ack;
        auto_ack_slot(ack, 0);
        if (ack.slot_type(0) == SLOT_CTRL) send_flit(ack, false);
    }
}

// ------------------------------------------------------------------
// tx
// ------------------------------------------------------------------
void CxlHostModel::auto_ack_slot(CxlFlit& f, int s) {
    if (last_dev_seq == 0xFF || rcvd_since_ack_ == 0) return;
    f.set_slot_type(s, SLOT_CTRL);
    f.set_slot_word(s, 0, 0x1u | (uint32_t(last_dev_seq & 0xF) << 8));
    rcvd_since_ack_ = 0;
}

void CxlHostModel::send_flit(CxlFlit& f, bool consume_credits, int io_slots,
                             int mem_slots) {
    if (consume_credits) wait_credits(io_slots, mem_slots);
    f.set_seq(tx_seq_);
    tx_seq_ = uint8_t((tx_seq_ + 1) & 0xF);
    f.seal();
    last_sent_ = f;                    // keep a clean copy for retransmit
    have_last_sent_ = true;
    if (corrupt_next_) {
        f.bytes[20] ^= 0xA5;           // corrupt a payload byte, CRC now wrong
        corrupt_next_ = false;
    }
    p_.drive_rx_data(f.bytes);
    p_.drive_rx_valid(1);
    for (uint32_t guard = 0; guard < 100000; guard++) {
        bool ready = p_.sample_rx_ready() != 0;
        // harvest any tx flit in the same cycle
        bool have = p_.sample_tx_valid() && tx_ready_;
        CxlFlit rf;
        if (have)
            p_.sample_tx_data([&](uint8_t* raw) { std::memcpy(rf.bytes, raw, FLIT_BYTES); });
        p_.tick();
        if (have) process_rx(rf);
        if (ready) break;
    }
    p_.drive_rx_valid(0);
    stat_flits_sent++;
    io_credits  -= io_slots;
    mem_credits -= mem_slots;
}

bool CxlHostModel::recv_flit(CxlFlit& f, uint32_t max_cycles) {
    uint32_t before = stat_flits_rcvd;
    for (uint32_t c = 0; c < max_cycles; c++) {
        pump(1);
        if (stat_flits_rcvd != before) {
            f = last_rx;
            return true;
        }
    }
    return false;
}

void CxlHostModel::wait_credits(int io_need, int mem_need, uint32_t timeout) {
    for (uint32_t c = 0; c < timeout; c++) {
        if (io_credits >= io_need && mem_credits >= mem_need) return;
        pump(1);
    }
    throw std::runtime_error("wait_credits: timeout (credit starvation)");
}

void CxlHostModel::inject_crc_error() { corrupt_next_ = true; }

void CxlHostModel::inject_nak(uint8_t seq) {
    expected_rx_seq_ = uint8_t(seq & 0xF);
    CxlFlit f;
    f.set_slot_type(0, SLOT_CTRL);
    f.set_slot_word(0, 0, 0x2u | (uint32_t(seq & 0xF) << 8));
    rcvd_since_ack_ = 0;               // this ctrl flit carries no ack
    send_flit(f, false);
}

void CxlHostModel::set_tx_ready(bool r) {
    tx_ready_ = r;
    p_.drive_tx_ready(r ? 1 : 0);
}

// ------------------------------------------------------------------
// transactions
// ------------------------------------------------------------------
uint32_t CxlHostModel::wait_response(uint8_t tag, uint32_t timeout) {
    for (uint32_t c = 0; c < timeout; c++) {
        auto it = responses_.find(tag);
        if (it != responses_.end()) {
            uint32_t v = it->second;
            responses_.erase(it);
            return v;
        }
        pump(1);
    }
    throw std::runtime_error("wait_response: timeout");
}

void CxlHostModel::mmio_write(uint8_t offset, uint32_t value) {
    CxlFlit f;
    auto_ack_slot(f, 0);
    f.set_slot_type(1, SLOT_IO);
    f.set_slot_word(1, 0, 0x80000000u | offset);
    f.set_slot_word(1, 1, value);
    send_flit(f, true, 1, 0);
}

uint32_t CxlHostModel::mmio_read(uint8_t offset) {
    uint8_t tag = next_tag();
    CxlFlit f;
    auto_ack_slot(f, 0);
    f.set_slot_type(1, SLOT_IO);
    f.set_slot_word(1, 0, uint32_t(offset) | (uint32_t(tag) << 16));
    send_flit(f, true, 1, 0);
    return wait_response(tag, 100000);
}

void CxlHostModel::mem_write(uint16_t addr, uint32_t data) {
    CxlFlit f;
    auto_ack_slot(f, 0);
    f.set_slot_type(1, SLOT_MEM);
    f.set_slot_word(1, 0, 0x80000000u | addr);
    f.set_slot_word(1, 1, data);
    send_flit(f, true, 0, 1);
}

uint32_t CxlHostModel::mem_read(uint16_t addr) {
    uint8_t tag = next_tag();
    CxlFlit f;
    auto_ack_slot(f, 0);
    f.set_slot_type(1, SLOT_MEM);
    f.set_slot_word(1, 0, uint32_t(addr) | (uint32_t(tag) << 16));
    send_flit(f, true, 0, 1);
    return wait_response(tag, 100000);
}

void CxlHostModel::mem_write_burst(uint16_t base, const std::vector<uint32_t>& data) {
    // pack 3 MEM write slots per flit (slot 0 reserved for the ack ctrl slot)
    size_t i = 0;
    while (i < data.size()) {
        int n = int(std::min<size_t>(3, data.size() - i));
        wait_credits(0, n);
        CxlFlit f;
        auto_ack_slot(f, 0);
        for (int s = 0; s < n; s++) {
            f.set_slot_type(1 + s, SLOT_MEM);
            f.set_slot_word(1 + s, 0, 0x80000000u | uint16_t(base + i + s));
            f.set_slot_word(1 + s, 1, data[i + s]);
        }
        send_flit(f, true, 0, n);
        i += n;
    }
}

void CxlHostModel::mem_read_burst(uint16_t base, std::vector<uint32_t>& result,
                                  size_t n) {
    result.resize(n);
    for (size_t i = 0; i < n; i++) result[i] = mem_read(uint16_t(base + i));
}

// ------------------------------------------------------------------
// high level
// ------------------------------------------------------------------
void CxlHostModel::download_config(uint64_t cfg_word) {
    mmio_write(REG_CFG_WORD_LO, uint32_t(cfg_word & 0xFFFFFFFFu));
    mmio_write(REG_CFG_WORD_HI, uint32_t(cfg_word >> 32));
    mmio_write(REG_CFG_SUBMIT, 1);
}

void CxlHostModel::submit_dense(uint8_t op, uint16_t src_a, uint16_t src_b,
                                uint16_t dst, uint16_t len, uint16_t stride) {
    mmio_write(REG_CMD_OP, op);
    mmio_write(REG_CMD_SRC_A, src_a);
    mmio_write(REG_CMD_SRC_B, src_b);
    mmio_write(REG_CMD_DST, dst);
    mmio_write(REG_CMD_LEN, len);
    mmio_write(REG_CMD_STRIDE, stride);
    mmio_write(REG_CMD_SUBMIT, 1);
}

void CxlHostModel::submit_sparse(uint8_t op, uint16_t src_a, uint16_t src_b,
                                 uint16_t dst, uint16_t idx_base,
                                 uint16_t idx_len, uint16_t stride) {
    mmio_write(REG_CMD_OP, op);
    mmio_write(REG_CMD_SRC_A, src_a);
    mmio_write(REG_CMD_SRC_B, src_b);
    mmio_write(REG_CMD_DST, dst);
    mmio_write(REG_IDX_BASE, idx_base);
    mmio_write(REG_IDX_LEN, idx_len);
    mmio_write(REG_CMD_STRIDE, stride);
    mmio_write(REG_CMD_SUBMIT, 1);
}

void CxlHostModel::submit_softmax(uint16_t src, uint16_t dst, uint16_t len) {
    mmio_write(REG_CMD_OP, OPC_SOFTMAX);
    mmio_write(REG_CMD_SRC_A, src);
    mmio_write(REG_CMD_DST, dst);
    mmio_write(REG_CMD_LEN, len);
    mmio_write(REG_CMD_SUBMIT, 1);
}

bool CxlHostModel::wait_for_done(uint32_t timeout_cycles) {
    for (uint32_t c = 0; c < timeout_cycles; c += 32) {
        uint32_t st = mmio_read(REG_CMD_STATUS);
        if (st == 2) return true;
        if (st == 3) return false;
        pump(8);
    }
    throw std::runtime_error("wait_for_done: timeout");
}

uint64_t CxlHostModel::get_perf_cycles() {
    uint64_t lo = mmio_read(REG_PERF_CYCLES_LO);
    uint64_t hi = mmio_read(REG_PERF_CYCLES_HI);
    return (hi << 32) | lo;
}
uint32_t CxlHostModel::get_perf_ops()   { return mmio_read(REG_PERF_OPS); }
uint32_t CxlHostModel::get_cxl_reads()  { return mmio_read(REG_PERF_CXL_RD); }
uint32_t CxlHostModel::get_cxl_writes() { return mmio_read(REG_PERF_CXL_WR); }
void     CxlHostModel::reset_perf()     { mmio_write(REG_PERF_RESET, 1); }

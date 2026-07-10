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
            if (w0 & 0x40000000u) {     // response (possibly a multi-word burst)
                uint8_t tag = uint8_t((w0 >> 16) & 0xFF);
                int n = (w0 & 0x20000000u) ? int(((w0 >> 24) & 3) + 1) : 1;
                std::vector<uint32_t> v(static_cast<size_t>(n));
                for (int w = 0; w < n; w++) v[size_t(w)] = f.slot_word(s, 1 + w);
                responses_[tag] = std::move(v);
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
        tick_();
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
        tick_();
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
    return wait_response_vec(tag, timeout)[0];
}

std::vector<uint32_t> CxlHostModel::wait_response_vec(uint8_t tag, uint32_t timeout) {
    for (uint32_t c = 0; c < timeout; c++) {
        auto it = responses_.find(tag);
        if (it != responses_.end()) {
            std::vector<uint32_t> v = std::move(it->second);
            responses_.erase(it);
            return v;
        }
        pump(1);
    }
    throw std::runtime_error("wait_response: timeout");
}

void CxlHostModel::mmio_write(uint8_t offset, uint32_t value) {
    stat_io_req_slots++;
    CxlFlit f;
    auto_ack_slot(f, 0);
    f.set_slot_type(1, SLOT_IO);
    f.set_slot_word(1, 0, 0x80000000u | offset);
    f.set_slot_word(1, 1, value);
    send_flit(f, true, 1, 0);
}

uint32_t CxlHostModel::mmio_read(uint8_t offset) {
    stat_io_req_slots++;
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
    // 3 burst-write slots per flit x 3 sequential words per slot = 9 words
    // per flit (slot 0 reserved for the ack ctrl slot); credits are per slot
    size_t i = 0;
    while (i < data.size()) {
        int slots = 0;
        CxlFlit f;
        size_t j = i;
        for (int s = 1; s < SLOTS_PER_FLIT && j < data.size(); s++) {
            int n = int(std::min<size_t>(3, data.size() - j));
            f.set_burst_write(s, uint16_t(base + j), &data[j], n);
            j += size_t(n);
            slots++;
        }
        wait_credits(0, slots);
        auto_ack_slot(f, 0);
        send_flit(f, true, 0, slots);
        i = j;
    }
    // Fence: CXL.io and CXL.mem dispatch from independent queues, so a
    // subsequent MMIO command submit could overtake still-queued burst writes
    // (a 3-word slot takes 3 dispatch cycles). All mem credits returned ⇔
    // every write slot has retired — an ordering guarantee that costs idle
    // cycles only, no extra link traffic.
    for (uint32_t c = 0; c < 200000 && mem_credits < INIT_CREDITS; c++) pump(1);
}

void CxlHostModel::mem_read_burst(uint16_t base, std::vector<uint32_t>& result,
                                  size_t n) {
    // 3 burst-read slots per flit x 3 words per slot; distinct tag per slot,
    // responses collected as multi-word slots (the device pipelines dispatch)
    result.resize(n);
    size_t i = 0;
    while (i < n) {
        int slots = 0;
        uint8_t tags[3] = {0, 0, 0};
        int     lens[3] = {0, 0, 0};
        CxlFlit f;
        size_t j = i;
        for (int s = 1; s < SLOTS_PER_FLIT && j < n; s++) {
            int k = int(std::min<size_t>(3, n - j));
            tags[slots] = next_tag();
            lens[slots] = k;
            f.set_burst_read(s, uint16_t(base + j), tags[slots], k);
            j += size_t(k);
            slots++;
        }
        wait_credits(0, slots);
        auto_ack_slot(f, 0);
        send_flit(f, true, 0, slots);
        size_t pos = i;
        for (int s = 0; s < slots; s++) {
            std::vector<uint32_t> v = wait_response_vec(tags[s], 100000);
            for (int w = 0; w < lens[s]; w++) result[pos + size_t(w)] = v[size_t(w)];
            pos += size_t(lens[s]);
        }
        i = j;
    }
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

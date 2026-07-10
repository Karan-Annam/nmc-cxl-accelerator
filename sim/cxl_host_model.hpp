// The host side of the CXL link. Speaks real 68-byte flits:
// packs transactions into slots, computes CRC-16 independently of the RTL (golden
// model), tracks credits, acks/naks, and re-sends on device NAK. The public
// transaction API (mem_write / mmio_read / ...) is a thin wrapper over flits, so
// tests are written at transaction level while every byte crosses the boundary
// framed, CRC-protected and credit-gated.
#pragma once
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>

// ---- flit format constants (mirror nmc_pkg.sv) ----
constexpr int FLIT_BYTES      = 68;
constexpr int SLOT_BYTES      = 16;
constexpr int SLOTS_PER_FLIT  = 4;
constexpr int INIT_CREDITS    = 8;
constexpr uint8_t SLOT_EMPTY  = 0;
constexpr uint8_t SLOT_IO     = 1;
constexpr uint8_t SLOT_MEM    = 2;
constexpr uint8_t SLOT_CTRL   = 3;

// MMIO register byte offsets (mirror nmc_pkg.sv)
enum MmioReg : uint8_t {
    REG_DEVICE_ID = 0x00, REG_DEVICE_STATUS = 0x04, REG_CMD_OP = 0x08,
    REG_CMD_SRC_A = 0x0C, REG_CMD_SRC_B = 0x10, REG_CMD_DST = 0x14,
    REG_CMD_LEN = 0x18, REG_CMD_STRIDE = 0x1C, REG_IDX_BASE = 0x20,
    REG_IDX_LEN = 0x24, REG_CMD_SUBMIT = 0x28, REG_CMD_STATUS = 0x2C,
    REG_CFG_WORD_LO = 0x30, REG_CFG_WORD_HI = 0x34, REG_CFG_SUBMIT = 0x38,
    REG_PERF_CYCLES_LO = 0x3C, REG_PERF_CYCLES_HI = 0x40, REG_PERF_OPS = 0x44,
    REG_PERF_CXL_RD = 0x48, REG_PERF_CXL_WR = 0x4C, REG_PERF_RESET = 0x50,
    REG_ERROR_CODE = 0x54,
    REG_LNK_CRC_ERRS = 0x58, REG_LNK_NAKS = 0x5C, REG_LNK_RETRIES = 0x60,
    REG_LNK_TXSTALL = 0x64, REG_LNK_RXNRDY = 0x68, REG_LNK_TX_FLITS = 0x6C,
    REG_LNK_TX_SLOTS = 0x70, REG_PERF_CMDS = 0x74
};

// command opcodes (mirror nmc_pkg.sv)
enum CmdOp : uint8_t {
    OPC_DENSE = 0, OPC_SPARSE = 1, OPC_SOFTMAX = 2,
    OPC_WGATHER = 3, OPC_REDUCTION = 4, OPC_EMBEDDING = 5
};

struct CxlFlit {
    uint8_t bytes[FLIT_BYTES];
    CxlFlit() { std::memset(bytes, 0, sizeof(bytes)); }

    uint8_t  slot_type(int s) const { return (bytes[0] >> (2 * s)) & 3; }
    void     set_slot_type(int s, uint8_t t) {
        bytes[0] = uint8_t((bytes[0] & ~(3 << (2 * s))) | ((t & 3) << (2 * s)));
    }
    uint8_t  seq() const { return bytes[1] & 0xF; }
    void     set_seq(uint8_t q) { bytes[1] = uint8_t((bytes[1] & 0xF0) | (q & 0xF)); }

    uint32_t slot_word(int s, int w) const {
        uint32_t v;
        std::memcpy(&v, &bytes[2 + SLOT_BYTES * s + 4 * w], 4);
        return v;
    }
    void set_slot_word(int s, int w, uint32_t v) {
        std::memcpy(&bytes[2 + SLOT_BYTES * s + 4 * w], &v, 4);
    }

    // burst slots (word0 bit29 + count-1 in [25:24]; see nmc_pkg.sv): n
    // sequential words at addr, addr+1, addr+2. n=1..3.
    void set_burst_write(int s, uint16_t addr, const uint32_t* d, int n) {
        set_slot_type(s, SLOT_MEM);
        set_slot_word(s, 0, 0x80000000u | 0x20000000u |
                            (uint32_t(n - 1) << 24) | addr);
        for (int w = 0; w < n; w++) set_slot_word(s, 1 + w, d[w]);
    }
    void set_burst_read(int s, uint16_t addr, uint8_t tag, int n) {
        set_slot_type(s, SLOT_MEM);
        set_slot_word(s, 0, 0x20000000u | (uint32_t(n - 1) << 24) |
                            (uint32_t(tag) << 16) | addr);
    }

    static uint16_t crc16(const uint8_t* data, int n) {
        uint16_t c = 0xFFFF;
        for (int k = 0; k < n; k++) {
            c = uint16_t(c ^ (uint16_t(data[k]) << 8));
            for (int i = 0; i < 8; i++)
                c = (c & 0x8000) ? uint16_t((c << 1) ^ 0x1021) : uint16_t(c << 1);
        }
        return c;
    }
    void seal() {   // compute + append CRC
        uint16_t c = crc16(bytes, FLIT_BYTES - 2);
        bytes[66] = uint8_t(c & 0xFF);
        bytes[67] = uint8_t(c >> 8);
    }
    bool crc_ok() const {
        uint16_t c = crc16(bytes, FLIT_BYTES - 2);
        return bytes[66] == uint8_t(c & 0xFF) && bytes[67] == uint8_t(c >> 8);
    }
};

// The testbench provides raw pin access + clocking through this interface, so the
// host model does not depend on the Verilated header directly.
struct DutPins {
    std::function<void(const uint8_t*)> drive_rx_data;  // 68 bytes → flit_rx_data
    std::function<void(int)>            drive_rx_valid;
    std::function<int()>                sample_rx_ready;
    std::function<int()>                sample_tx_valid;
    std::function<void(const std::function<void(uint8_t*)>&)> sample_tx_data;
    std::function<void(int)>            drive_tx_ready;
    std::function<void()>               tick;           // one full clock cycle
};

class CxlHostModel {
  public:
    explicit CxlHostModel(DutPins pins);

    // ---- CXL.mem ----
    void     mem_write(uint16_t addr, uint32_t data);
    uint32_t mem_read(uint16_t addr);
    void     mem_write_burst(uint16_t base, const std::vector<uint32_t>& data);
    void     mem_read_burst(uint16_t base, std::vector<uint32_t>& result, size_t n);

    // ---- CXL.io ----
    void     mmio_write(uint8_t offset, uint32_t value);
    uint32_t mmio_read(uint8_t offset);

    // ---- high level ----
    void download_config(uint64_t cfg_word);
    void submit_dense(uint8_t op, uint16_t src_a, uint16_t src_b, uint16_t dst,
                      uint16_t len, uint16_t stride = 1);
    void submit_sparse(uint8_t op, uint16_t src_a, uint16_t src_b, uint16_t dst,
                       uint16_t idx_base, uint16_t idx_len, uint16_t stride = 1);
    void submit_softmax(uint16_t src, uint16_t dst, uint16_t len);
    bool wait_for_done(uint32_t timeout_cycles = 2000000);   // false on ERROR status

    // ---- perf ----
    uint64_t get_perf_cycles();
    uint32_t get_perf_ops();
    uint32_t get_cxl_reads();
    uint32_t get_cxl_writes();
    uint32_t get_perf_cmds()  { return mmio_read(REG_PERF_CMDS); }
    // link-layer counters (device-side; see cxl_link_perf.sv)
    uint32_t get_lnk(uint8_t reg) { return mmio_read(reg); }
    void     reset_perf();

    // ---- flit layer primitives / test hooks ----
    void send_flit(CxlFlit& f, bool consume_credits = true, int io_slots = 0,
                   int mem_slots = 0);
    bool recv_flit(CxlFlit& f, uint32_t max_cycles = 10000);
    void inject_crc_error();          // corrupt the next tx flit (device must NAK)
    void inject_nak(uint8_t seq);     // demand replay of device flits from seq
    void pump(int cycles = 1);        // advance time, harvesting device flits
    void set_tx_ready(bool r);        // deassert to backpressure the device
    bool rx_ready() { return p_.sample_rx_ready() != 0; }  // device accepting flits?

    // observability for the flit tests
    uint32_t stat_flits_sent    = 0;
    uint32_t stat_flits_rcvd    = 0;   // good, in-order device flits processed
    uint32_t stat_dup_rcvd      = 0;   // out-of-order/duplicate device flits dropped
    uint32_t stat_naks_from_dev = 0;   // device detected a corrupt host flit
    uint32_t stat_resends       = 0;   // host retransmissions after device NAK
    uint32_t stat_io_slots_rx   = 0;   // response slots received per protocol
    uint32_t stat_mem_slots_rx  = 0;
    uint32_t stat_io_req_slots  = 0;   // MMIO request slots sent (ctrl-traffic accounting)
    uint64_t stat_cycles        = 0;   // clock cycles this model has driven
    int      io_credits  = INIT_CREDITS;
    int      mem_credits = INIT_CREDITS;
    CxlFlit  last_rx;                  // last good device flit (raw)
    uint8_t  last_dev_seq = 0xFF;      // latest good device seq (0xFF = none yet)

  private:
    void tick_() { p_.tick(); ++stat_cycles; }
    void process_rx(const CxlFlit& f);
    void wait_credits(int io_need, int mem_need, uint32_t timeout = 200000);
    void auto_ack_slot(CxlFlit& f, int s);
    uint32_t wait_response(uint8_t tag, uint32_t timeout);
    std::vector<uint32_t> wait_response_vec(uint8_t tag, uint32_t timeout);
    uint8_t next_tag() { return tag_ctr_++; }

    DutPins  p_;
    uint8_t  tx_seq_ = 0;
    uint8_t  expected_rx_seq_ = 0;
    uint8_t  tag_ctr_ = 1;
    bool     corrupt_next_ = false;
    bool     tx_ready_ = true;
    int      rcvd_since_ack_ = 0;
    CxlFlit  last_sent_;               // for retransmit after device NAK
    bool     have_last_sent_ = false;
    std::map<uint8_t, std::vector<uint32_t>> responses_;   // tag → data word(s)
};

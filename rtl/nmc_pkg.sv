// Shared types, opcodes, flit format, parameters. The register map and
// command semantics are documented in docs/ARCHITECTURE.md.
package nmc_pkg;

  // ---------------- Core parameters ----------------
  parameter int PE_COUNT   = 8;      // fixed: 3-bit bank fields, 56-bit config word
  parameter int DATA_WIDTH = 32;
  parameter int ADDR_WIDTH = 16;     // HDM word address (65536 words = 256 KB)
  parameter int SRAM_DEPTH = 65536;  // total HDM words across all banks
  parameter int BANK_DEPTH = SRAM_DEPTH / PE_COUNT; // 8192 words per bank
  parameter int BANK_AW    = $clog2(BANK_DEPTH);    // 13

  // Bank striping: word address A lives in bank A%8 at offset A/8.
  function automatic logic [2:0] bank_of(input logic [ADDR_WIDTH-1:0] a);
    return a[2:0];
  endfunction
  function automatic logic [BANK_AW-1:0] off_of(input logic [ADDR_WIDTH-1:0] a);
    return a[ADDR_WIDTH-1:3];
  endfunction

  // ---------------- PE operations ----------------
  typedef enum logic [3:0] {
    OP_ADD    = 4'h0,
    OP_SUB    = 4'h1,
    OP_MUL    = 4'h2,
    OP_MAX    = 4'h3,
    OP_MIN    = 4'h4,
    OP_AND    = 4'h5,
    OP_OR     = 4'h6,
    OP_XOR    = 4'h7,
    OP_MACC   = 4'h8,
    OP_SACC   = 4'h9,
    OP_PASS_A = 4'hA,
    OP_PASS_B = 4'hB,
    OP_NEG    = 4'hC,
    OP_ABS    = 4'hD,
    OP_SHR    = 4'hE,
    OP_ZERO   = 4'hF
  } pe_op_e;

  // pe_cfg: [3:0] op, [5:4] src_sel (00 acc, 01 bankA, 10 bankB, 11 zero), [6] mask_en
  parameter int PE_CFG_W  = 7;
  parameter int CFG_WORD_W = PE_COUNT * PE_CFG_W; // 56

  // ---------------- Commands ----------------
  typedef enum logic [3:0] {
    CMD_DENSE     = 4'h0,
    CMD_SPARSE    = 4'h1,  // per index m: dst[m] = tree(sum_d A[idx[m]*stride+d] * B[d])
    CMD_SOFTMAX   = 4'h2,
    CMD_WGATHER   = 4'h3,  // acc_vec[d] += B[m] * A[idx[m]*stride+d]; dst[d] = acc_vec[d]
    CMD_REDUCTION = 4'h4,  // dst[0] = fold over A[idx[m]*stride] (sum via SACC / max via MAX)
    CMD_EMBEDDING = 4'h5   // dst[m*stride+d] = A[idx[m]*stride+d]
  } cmd_op_e;

  typedef struct packed {
    logic [3:0]            cmd_op;
    logic [ADDR_WIDTH-1:0] src_a;
    logic [ADDR_WIDTH-1:0] src_b;
    logic [ADDR_WIDTH-1:0] dst;
    logic [15:0]           len;      // output element count
    logic [ADDR_WIDTH-1:0] idx_base;
    logic [15:0]           idx_len;
    logic [ADDR_WIDTH-1:0] stride;   // element/row stride (default 1)
  } nmc_cmd_t;

  // CMD_STATUS values
  parameter logic [1:0] ST_IDLE    = 2'd0;
  parameter logic [1:0] ST_RUNNING = 2'd1;
  parameter logic [1:0] ST_DONE    = 2'd2;
  parameter logic [1:0] ST_ERROR   = 2'd3;

  // Error codes
  parameter logic [7:0] ERR_NONE       = 8'd0;
  parameter logic [7:0] ERR_BAD_OP     = 8'd1;
  parameter logic [7:0] ERR_STRIDE_CAP = 8'd2;

  // WGATHER accumulator vector capacity (d_k <= 64)
  parameter int ACCVEC_DEPTH = 64;

  // ---------------- MMIO register offsets (byte offsets, 32-bit regs) ----------------
  parameter logic [7:0] R_DEVICE_ID      = 8'h00;
  parameter logic [7:0] R_DEVICE_STATUS  = 8'h04;
  parameter logic [7:0] R_CMD_OP         = 8'h08;
  parameter logic [7:0] R_CMD_SRC_A      = 8'h0C;
  parameter logic [7:0] R_CMD_SRC_B      = 8'h10;
  parameter logic [7:0] R_CMD_DST        = 8'h14;
  parameter logic [7:0] R_CMD_LEN        = 8'h18;
  parameter logic [7:0] R_CMD_STRIDE     = 8'h1C;
  parameter logic [7:0] R_IDX_BASE       = 8'h20;
  parameter logic [7:0] R_IDX_LEN        = 8'h24;
  parameter logic [7:0] R_CMD_SUBMIT     = 8'h28;
  parameter logic [7:0] R_CMD_STATUS     = 8'h2C;
  parameter logic [7:0] R_CFG_WORD_LO    = 8'h30;
  parameter logic [7:0] R_CFG_WORD_HI    = 8'h34;
  parameter logic [7:0] R_CFG_SUBMIT     = 8'h38;
  parameter logic [7:0] R_PERF_CYCLES_LO = 8'h3C;
  parameter logic [7:0] R_PERF_CYCLES_HI = 8'h40;
  parameter logic [7:0] R_PERF_OPS       = 8'h44;
  parameter logic [7:0] R_PERF_CXL_RD    = 8'h48;
  parameter logic [7:0] R_PERF_CXL_WR    = 8'h4C;
  parameter logic [7:0] R_PERF_RESET     = 8'h50;
  parameter logic [7:0] R_ERROR_CODE     = 8'h54;

  parameter logic [31:0] DEVICE_ID_VAL = 32'hCA55_0001;

  // ---------------- CXL flit/link layer ----------------
  // 68-byte flit: 2B header + 4 x 16B slots + 2B CRC-16-CCITT.
  // Byte k of the flit occupies bits [8k+7 : 8k] of the 544-bit vector.
  parameter int CXL_FLIT_BYTES     = 68;
  parameter int CXL_FLIT_W         = CXL_FLIT_BYTES * 8;  // 544
  parameter int CXL_SLOT_BYTES     = 16;
  parameter int CXL_SLOTS_PER_FLIT = 4;
  parameter int CXL_SEQ_W          = 4;
  parameter int RETRY_DEPTH        = 8;   // must be <= 2**(CXL_SEQ_W-1)
  parameter int INIT_CREDITS       = 8;   // per protocol, initialized at reset (no LTSSM)
  parameter int RXQ_DEPTH          = 16;  // per-protocol rx slot queue depth

  // Slot type field (2 bits per slot, header byte 0)
  parameter logic [1:0] SLOT_EMPTY = 2'b00;
  parameter logic [1:0] SLOT_IO    = 2'b01;
  parameter logic [1:0] SLOT_MEM   = 2'b10;
  parameter logic [1:0] SLOT_CTRL  = 2'b11;

  // Transaction slot layout (16B = 4 x 32-bit little-endian words):
  //  word0: [31] is_write  [30] is_response  [23:16] tag  [15:0] address
  //  word1: data
  //  word2, word3: reserved (zero)
  // LINK_CTRL slot layout:
  //  byte0: bit0 ack_valid, bit1 nak_valid
  //  byte1: [3:0] ack/nak sequence number
  //  byte2: io credit return count
  //  byte3: mem credit return count

  // CRC-16-CCITT (poly 0x1021, init 0xFFFF), bytewise MSB-first, over bytes 0..65.
  function automatic logic [15:0] crc16_step(input logic [15:0] crc, input logic [7:0] b);
    logic [15:0] c;
    c = crc ^ {b, 8'h00};
    for (int i = 0; i < 8; i++)
      c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
    return c;
  endfunction

  function automatic logic [15:0] flit_crc(input logic [CXL_FLIT_W-1:0] f);
    logic [15:0] c;
    c = 16'hFFFF;
    for (int k = 0; k < CXL_FLIT_BYTES - 2; k++)
      c = crc16_step(c, f[8*k +: 8]);
    return c;
  endfunction

endpackage

// CXL protocol-semantics layer: MMIO register file (CXL.io) and
// HDM port arbiter (CXL.mem). Fires commands/config into the NMC engine. This module
// is identical in role whether it sits behind the flit link layer (this project) or a
// PCIe BAR controller (FPGA port) — it cannot tell the difference.
// Note the HDM arbiter and the link-layer credit system are orthogonal: the arbiter
// decides WHO may issue HDM requests (host vs engine), credits decide HOW MANY host
// requests may be in flight.
module cxl_controller
  import nmc_pkg::*;
(
  input  logic                  clk,
  input  logic                  rst_n,

  // MMIO (CXL.io) — reads answered combinationally
  input  logic                  mmio_valid,
  input  logic                  mmio_write,
  input  logic [7:0]            mmio_offset,
  input  logic [DATA_WIDTH-1:0] mmio_wdata,
  output logic [DATA_WIDTH-1:0] mmio_rdata,

  // HDM (CXL.mem) host port
  input  logic                  hdm_valid,
  input  logic                  hdm_write,
  input  logic [ADDR_WIDTH-1:0] hdm_addr,
  input  logic [DATA_WIDTH-1:0] hdm_wdata,
  output logic                  hdm_ready,
  output logic                  hdm_rvalid,
  output logic [DATA_WIDTH-1:0] hdm_rdata,

  // host→SRAM access path (muxed with engine in nmc_top)
  output logic                  host_we,
  output logic [2:0]            host_wbank,
  output logic [BANK_AW-1:0]    host_woff,
  output logic [DATA_WIDTH-1:0] host_wdata,
  output logic                  host_re,
  output logic [2:0]            host_rbank,
  output logic [BANK_AW-1:0]    host_roff,
  input  logic [DATA_WIDTH-1:0] host_rdata,   // registered bank output, muxed in top

  // engine interface
  output logic                  cmd_valid,
  output nmc_cmd_t              cmd,
  output logic                  cfg_valid,
  output logic [CFG_WORD_W-1:0] cfg_word,
  input  logic                  cmd_done,
  input  logic                  engine_busy,
  input  logic                  engine_error,
  input  logic [7:0]            engine_error_code,

  // perf
  output logic                  perf_reset,
  output logic                  cmd_issued_pulse,
  output logic                  cxl_rd_inc,
  output logic                  cxl_wr_inc,
  input  logic [63:0]           perf_cycles,
  input  logic [31:0]           perf_ops,
  input  logic [31:0]           perf_cxl_reads,
  input  logic [31:0]           perf_cxl_writes,
  input  logic [15:0]           perf_cmds,

  // link perf counters (read-only mirrors from cxl_link_layer)
  input  logic [31:0]           lnk_crc_errs,
  input  logic [31:0]           lnk_naks_sent,
  input  logic [31:0]           lnk_retries,
  input  logic [31:0]           lnk_tx_stall_cyc,
  input  logic [31:0]           lnk_rx_nrdy_cyc,
  input  logic [31:0]           lnk_tx_flits,
  input  logic [31:0]           lnk_tx_slots
);

  // ---------------- command/config staging registers ----------------
  logic [3:0]            r_cmd_op;
  logic [ADDR_WIDTH-1:0] r_src_a, r_src_b, r_dst, r_idx_base, r_stride;
  logic [15:0]           r_len, r_idx_len;
  logic [31:0]           r_cfg_lo, r_cfg_hi;
  logic [1:0]            r_status;
  logic [7:0]            r_error;

  // ---------------- HDM arbiter ----------------
  // Host has HDM access only when the engine is idle.
  assign hdm_ready = !engine_busy;

  logic hdm_rd_fire, hdm_wr_fire;
  assign hdm_rd_fire = hdm_valid && hdm_ready && !hdm_write;
  assign hdm_wr_fire = hdm_valid && hdm_ready && hdm_write;

  assign host_we    = hdm_wr_fire;
  assign host_wbank = bank_of(hdm_addr);
  assign host_woff  = off_of(hdm_addr);
  assign host_wdata = hdm_wdata;
  assign host_re    = hdm_rd_fire;
  assign host_rbank = bank_of(hdm_addr);
  assign host_roff  = off_of(hdm_addr);

  logic hdm_rv1;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      hdm_rv1    <= 1'b0;
      hdm_rvalid <= 1'b0;
    end else begin
      hdm_rv1    <= hdm_rd_fire;
      hdm_rvalid <= hdm_rv1;
    end
  end
  assign hdm_rdata = host_rdata;   // valid TWO cycles after the read fires
                                   // (bank outpost register + BRAM register)

  assign cxl_rd_inc = hdm_rd_fire;
  assign cxl_wr_inc = hdm_wr_fire;

  // ---------------- MMIO register file ----------------
  logic mmio_wr_fire;
  assign mmio_wr_fire = mmio_valid && mmio_write;

  assign cfg_word = {r_cfg_hi[CFG_WORD_W-33:0], r_cfg_lo};

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      r_cmd_op <= '0; r_src_a <= '0; r_src_b <= '0; r_dst <= '0;
      r_idx_base <= '0; r_stride <= ADDR_WIDTH'(1);
      r_len <= '0; r_idx_len <= '0;
      r_cfg_lo <= '0; r_cfg_hi <= '0;
      r_status <= ST_IDLE; r_error <= ERR_NONE;
      cmd_valid <= 1'b0; cfg_valid <= 1'b0;
      perf_reset <= 1'b0; cmd_issued_pulse <= 1'b0;
      cmd <= '0;
    end else begin
      cmd_valid <= 1'b0;
      cfg_valid <= 1'b0;
      perf_reset <= 1'b0;
      cmd_issued_pulse <= 1'b0;

      if (mmio_wr_fire) begin
        unique case (mmio_offset)
          R_CMD_OP:      r_cmd_op   <= mmio_wdata[3:0];
          R_CMD_SRC_A:   r_src_a    <= mmio_wdata[ADDR_WIDTH-1:0];
          R_CMD_SRC_B:   r_src_b    <= mmio_wdata[ADDR_WIDTH-1:0];
          R_CMD_DST:     r_dst      <= mmio_wdata[ADDR_WIDTH-1:0];
          R_CMD_LEN:     r_len      <= mmio_wdata[15:0];
          R_CMD_STRIDE:  r_stride   <= mmio_wdata[ADDR_WIDTH-1:0];
          R_IDX_BASE:    r_idx_base <= mmio_wdata[ADDR_WIDTH-1:0];
          R_IDX_LEN:     r_idx_len  <= mmio_wdata[15:0];
          R_CFG_WORD_LO: r_cfg_lo   <= mmio_wdata;
          R_CFG_WORD_HI: r_cfg_hi   <= mmio_wdata;
          R_CFG_SUBMIT:  if (mmio_wdata[0]) cfg_valid <= 1'b1;
          R_PERF_RESET:  if (mmio_wdata[0]) perf_reset <= 1'b1;
          R_CMD_SUBMIT: begin
            if (mmio_wdata[0] && !engine_busy) begin
              cmd.cmd_op   <= r_cmd_op;
              cmd.src_a    <= r_src_a;
              cmd.src_b    <= r_src_b;
              cmd.dst      <= r_dst;
              cmd.len      <= r_len;
              cmd.idx_base <= r_idx_base;
              cmd.idx_len  <= r_idx_len;
              cmd.stride   <= r_stride;
              cmd_valid    <= 1'b1;
              cmd_issued_pulse <= 1'b1;
              r_status     <= ST_RUNNING;
              r_error      <= ERR_NONE;
            end
          end
          default: ;
        endcase
      end

      if (cmd_done) begin
        r_status <= engine_error ? ST_ERROR : ST_DONE;
        if (engine_error) r_error <= engine_error_code;
      end
    end
  end

  always_comb begin
    mmio_rdata = '0;
    unique case (mmio_offset)
      R_DEVICE_ID:      mmio_rdata = DEVICE_ID_VAL;
      R_DEVICE_STATUS:  mmio_rdata = {29'd0, (r_status == ST_ERROR),
                                      engine_busy, !engine_busy};
      R_CMD_STATUS:     mmio_rdata = {30'd0, r_status};
      R_ERROR_CODE:     mmio_rdata = {24'd0, r_error};
      R_PERF_CYCLES_LO: mmio_rdata = perf_cycles[31:0];
      R_PERF_CYCLES_HI: mmio_rdata = perf_cycles[63:32];
      R_PERF_OPS:       mmio_rdata = perf_ops;
      R_PERF_CXL_RD:    mmio_rdata = perf_cxl_reads;
      R_PERF_CXL_WR:    mmio_rdata = perf_cxl_writes;
      R_PERF_CMDS:      mmio_rdata = {16'd0, perf_cmds};
      R_LNK_CRC_ERRS:   mmio_rdata = lnk_crc_errs;
      R_LNK_NAKS:       mmio_rdata = lnk_naks_sent;
      R_LNK_RETRIES:    mmio_rdata = lnk_retries;
      R_LNK_TXSTALL:    mmio_rdata = lnk_tx_stall_cyc;
      R_LNK_RXNRDY:     mmio_rdata = lnk_rx_nrdy_cyc;
      R_LNK_TX_FLITS:   mmio_rdata = lnk_tx_flits;
      R_LNK_TX_SLOTS:   mmio_rdata = lnk_tx_slots;
      default:          mmio_rdata = '0;
    endcase
  end

endmodule

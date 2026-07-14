// Chip top. External boundary is the CXL flit interface (544-bit
// flits); inside: cxl_link_layer → cxl_controller → nmc_engine (+ softmax_unit,
// perf_counters) over 8 dual-read-port SRAM banks. This file owns the SRAM port
// muxing between the three owners: host (engine idle), engine, softmax unit.
module nmc_top
  import nmc_pkg::*;
(
  input  logic                  clk,
  input  logic                  rst_n,

  input  logic                  flit_rx_valid,
  input  logic [CXL_FLIT_W-1:0] flit_rx_data,
  output logic                  flit_rx_ready,
  output logic                  flit_tx_valid,
  output logic [CXL_FLIT_W-1:0] flit_tx_data,
  input  logic                  flit_tx_ready
);

  // ------------------------------------------------------------------
  // link layer ⇄ controller wires
  // ------------------------------------------------------------------
  logic                  mmio_valid, mmio_write;
  logic [7:0]            mmio_offset;
  logic [DATA_WIDTH-1:0] mmio_wdata, mmio_rdata;
  logic                  hdm_valid, hdm_write, hdm_ready, hdm_rvalid;
  logic [ADDR_WIDTH-1:0] hdm_addr;
  logic [DATA_WIDTH-1:0] hdm_wdata, hdm_rdata;

  logic        perf_reset;   // driven by cxl_controller (declared before use here)
  logic [31:0] lnk_crc_errs, lnk_naks_sent, lnk_retries;
  logic [31:0] lnk_tx_stall_cyc, lnk_rx_nrdy_cyc, lnk_tx_flits, lnk_tx_slots;

  cxl_link_layer u_link (
    .clk(clk), .rst_n(rst_n),
    .flit_rx_valid(flit_rx_valid), .flit_rx_data(flit_rx_data),
    .flit_rx_ready(flit_rx_ready),
    .flit_tx_valid(flit_tx_valid), .flit_tx_data(flit_tx_data),
    .flit_tx_ready(flit_tx_ready),
    .mmio_valid(mmio_valid), .mmio_write(mmio_write),
    .mmio_offset(mmio_offset), .mmio_wdata(mmio_wdata), .mmio_rdata(mmio_rdata),
    .hdm_valid(hdm_valid), .hdm_write(hdm_write),
    .hdm_addr(hdm_addr), .hdm_wdata(hdm_wdata),
    .hdm_ready(hdm_ready), .hdm_rvalid(hdm_rvalid), .hdm_rdata(hdm_rdata),
    .perf_reset(perf_reset),
    .lnk_crc_errs(lnk_crc_errs), .lnk_naks_sent(lnk_naks_sent),
    .lnk_retries(lnk_retries), .lnk_tx_stall_cyc(lnk_tx_stall_cyc),
    .lnk_rx_nrdy_cyc(lnk_rx_nrdy_cyc), .lnk_tx_flits(lnk_tx_flits),
    .lnk_tx_slots(lnk_tx_slots)
  );

  // ------------------------------------------------------------------
  // controller ⇄ engine wires
  // ------------------------------------------------------------------
  logic                  host_we, host_re;
  logic [2:0]            host_wbank, host_rbank;
  logic [BANK_AW-1:0]    host_woff, host_roff;
  logic [DATA_WIDTH-1:0] host_wdata, host_rdata;

  logic                  cmd_valid, cfg_valid, cmd_done;
  nmc_cmd_t              cmd;
  logic [CFG_WORD_W-1:0] cfg_word;
  logic                  engine_busy, engine_error;
  logic [7:0]            engine_error_code;

  logic                  cmd_issued_pulse, cxl_rd_inc, cxl_wr_inc;
  logic [63:0]           perf_cycles;
  logic [31:0]           perf_ops, perf_cxl_reads, perf_cxl_writes;
  logic [15:0]           perf_cmds;
  logic [3:0]            ops_inc;

  cxl_controller u_ctrl (
    .clk(clk), .rst_n(rst_n),
    .mmio_valid(mmio_valid), .mmio_write(mmio_write),
    .mmio_offset(mmio_offset), .mmio_wdata(mmio_wdata), .mmio_rdata(mmio_rdata),
    .hdm_valid(hdm_valid), .hdm_write(hdm_write),
    .hdm_addr(hdm_addr), .hdm_wdata(hdm_wdata),
    .hdm_ready(hdm_ready), .hdm_rvalid(hdm_rvalid), .hdm_rdata(hdm_rdata),
    .host_we(host_we), .host_wbank(host_wbank), .host_woff(host_woff),
    .host_wdata(host_wdata),
    .host_re(host_re), .host_rbank(host_rbank), .host_roff(host_roff),
    .host_rdata(host_rdata),
    .cmd_valid(cmd_valid), .cmd(cmd),
    .cfg_valid(cfg_valid), .cfg_word(cfg_word),
    .cmd_done(cmd_done), .engine_busy(engine_busy),
    .engine_error(engine_error), .engine_error_code(engine_error_code),
    .perf_reset(perf_reset), .cmd_issued_pulse(cmd_issued_pulse),
    .cxl_rd_inc(cxl_rd_inc), .cxl_wr_inc(cxl_wr_inc),
    .perf_cycles(perf_cycles), .perf_ops(perf_ops),
    .perf_cxl_reads(perf_cxl_reads), .perf_cxl_writes(perf_cxl_writes),
    .perf_cmds(perf_cmds),
    .lnk_crc_errs(lnk_crc_errs), .lnk_naks_sent(lnk_naks_sent),
    .lnk_retries(lnk_retries), .lnk_tx_stall_cyc(lnk_tx_stall_cyc),
    .lnk_rx_nrdy_cyc(lnk_rx_nrdy_cyc), .lnk_tx_flits(lnk_tx_flits),
    .lnk_tx_slots(lnk_tx_slots)
  );

  // ------------------------------------------------------------------
  // engine + softmax
  // ------------------------------------------------------------------
  logic [BANK_AW-1:0]    eng_raddr_a [PE_COUNT];
  logic [BANK_AW-1:0]    eng_raddr_b [PE_COUNT];
  logic [DATA_WIDTH-1:0] rdata_a     [PE_COUNT];
  logic [DATA_WIDTH-1:0] rdata_b     [PE_COUNT];
  logic                  eng_we      [PE_COUNT];
  logic [BANK_AW-1:0]    eng_woff    [PE_COUNT];
  logic [DATA_WIDTH-1:0] eng_wdata   [PE_COUNT];

  logic                  sm_start, sm_busy, sm_done;
  logic [ADDR_WIDTH-1:0] sm_src, sm_dst;
  logic [15:0]           sm_len;
  logic                  sm_rd_en, sm_wr_en;
  logic [2:0]            sm_rd_bank, sm_wr_bank;
  logic [BANK_AW-1:0]    sm_rd_off, sm_wr_off;
  logic [DATA_WIDTH-1:0] sm_wr_data, sm_rd_data;

  nmc_engine u_engine (
    .clk(clk), .rst_n(rst_n),
    .cmd_valid(cmd_valid), .cmd(cmd),
    .cfg_valid(cfg_valid), .cfg_word(cfg_word),
    .cmd_done(cmd_done), .engine_busy(engine_busy),
    .engine_error(engine_error), .engine_error_code(engine_error_code),
    .eng_raddr_a(eng_raddr_a), .eng_raddr_b(eng_raddr_b),
    .rdata_a(rdata_a), .rdata_b(rdata_b),
    .eng_we(eng_we), .eng_woff(eng_woff), .eng_wdata(eng_wdata),
    .sm_start(sm_start), .sm_src(sm_src), .sm_dst(sm_dst), .sm_len(sm_len),
    .sm_busy(sm_busy), .sm_done(sm_done),
    .ops_inc(ops_inc)
  );

  softmax_unit u_softmax (
    .clk(clk), .rst_n(rst_n),
    .sm_start(sm_start), .sm_src(sm_src), .sm_dst(sm_dst), .sm_len(sm_len),
    .sm_busy(sm_busy), .sm_done(sm_done),
    .rd_en(sm_rd_en), .rd_bank(sm_rd_bank), .rd_off(sm_rd_off),
    .rd_data(sm_rd_data),
    .wr_en(sm_wr_en), .wr_bank(sm_wr_bank), .wr_off(sm_wr_off),
    .wr_data(sm_wr_data)
  );

  perf_counters u_perf (
    .clk(clk), .rst_n(rst_n),
    .perf_reset(perf_reset),
    .engine_active(engine_busy),
    .ops_inc(ops_inc),
    .cmd_issued(cmd_issued_pulse),
    .cxl_rd_inc(cxl_rd_inc), .cxl_wr_inc(cxl_wr_inc),
    .cycles_active(perf_cycles), .ops_completed(perf_ops),
    .commands_issued(perf_cmds),
    .cxl_reads(perf_cxl_reads), .cxl_writes(perf_cxl_writes)
  );

  // ------------------------------------------------------------------
  // SRAM banks + port muxing
  // owners: softmax (sm_busy) > engine (engine_busy) > host (idle)
  // ------------------------------------------------------------------
  logic                  bank_we    [PE_COUNT];
  logic [BANK_AW-1:0]    bank_waddr [PE_COUNT];
  logic [DATA_WIDTH-1:0] bank_wdata [PE_COUNT];
  logic [BANK_AW-1:0]    bank_ra    [PE_COUNT];
  logic [BANK_AW-1:0]    bank_rb    [PE_COUNT];

  always_comb begin
    for (int k = 0; k < PE_COUNT; k++) begin
      if (sm_busy) begin
        bank_we[k]    = sm_wr_en && (sm_wr_bank == 3'(k));
        bank_waddr[k] = sm_wr_off;
        bank_wdata[k] = sm_wr_data;
        bank_ra[k]    = (sm_rd_en && (sm_rd_bank == 3'(k))) ? sm_rd_off : '0;
        bank_rb[k]    = '0;
      end else if (engine_busy) begin
        bank_we[k]    = eng_we[k];
        bank_waddr[k] = eng_woff[k];
        bank_wdata[k] = eng_wdata[k];
        bank_ra[k]    = eng_raddr_a[k];
        bank_rb[k]    = eng_raddr_b[k];
      end else begin
        bank_we[k]    = host_we && (host_wbank == 3'(k));
        bank_waddr[k] = host_woff;
        bank_wdata[k] = host_wdata;
        bank_ra[k]    = (host_re && (host_rbank == 3'(k))) ? host_roff : '0;
        bank_rb[k]    = '0;
      end
    end
  end

  for (genvar k = 0; k < PE_COUNT; k++) begin : g_banks
    sram_bank u_bank (
      .clk(clk),
      .we(bank_we[k]), .waddr(bank_waddr[k]), .wdata(bank_wdata[k]),
      .raddr_a(bank_ra[k]), .rdata_a(rdata_a[k]),
      .raddr_b(bank_rb[k]), .rdata_b(rdata_b[k])
    );
  end

  // registered read-bank selects for the 1-cycle-latency data muxes
  logic [2:0] host_rbank_q, sm_rd_bank_q;
  always_ff @(posedge clk) begin
    host_rbank_q <= host_rbank;
    sm_rd_bank_q <= sm_rd_bank;
  end
  assign host_rdata = rdata_a[host_rbank_q];
  assign sm_rd_data = rdata_a[sm_rd_bank_q];

endmodule

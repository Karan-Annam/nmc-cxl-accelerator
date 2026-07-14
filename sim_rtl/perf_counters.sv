// Cycle / op / command / CXL-traffic counters. cxl_reads and cxl_writes
// (host-initiated HDM word transactions) are the primary metric — they don't
// depend on the 1-cycle memory model. cycles_active is bookkeeping only.
module perf_counters
  import nmc_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        perf_reset,      // PERF_RESET write
  input  logic        engine_active,   // engine outside IDLE
  input  logic [3:0]  ops_inc,         // elements processed this cycle (0..8)
  input  logic        cmd_issued,      // CMD_SUBMIT accepted
  input  logic        cxl_rd_inc,      // host HDM read accepted
  input  logic        cxl_wr_inc,      // host HDM write accepted
  output logic [63:0] cycles_active,
  output logic [31:0] ops_completed,
  output logic [15:0] commands_issued,
  output logic [31:0] cxl_reads,
  output logic [31:0] cxl_writes
);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cycles_active   <= '0;
      ops_completed   <= '0;
      commands_issued <= '0;
      cxl_reads       <= '0;
      cxl_writes      <= '0;
    end else if (perf_reset) begin
      cycles_active   <= '0;
      ops_completed   <= '0;
      commands_issued <= '0;
      cxl_reads       <= '0;
      cxl_writes      <= '0;
    end else begin
      if (engine_active) cycles_active <= cycles_active + 64'd1;
      ops_completed <= ops_completed + {28'd0, ops_inc};
      if (cmd_issued) commands_issued <= commands_issued + 16'd1;
      if (cxl_rd_inc) cxl_reads  <= cxl_reads  + 32'd1;
      if (cxl_wr_inc) cxl_writes <= cxl_writes + 32'd1;
    end
  end

endmodule

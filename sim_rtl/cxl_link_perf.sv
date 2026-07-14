// Link-layer performance counters. The engine-side perf_counters module counts
// what the accelerator computes; this one counts what the link itself does —
// error/retry events, backpressure, and flit/slot utilization — so link
// efficiency is a measured number instead of an inference. Cleared by the same
// posted R_PERF_RESET as the engine counters (host must quiesce the link
// first, same rule as every perf window).
module cxl_link_perf
  import nmc_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        perf_reset,

  input  logic        crc_err,      // rx flit failed CRC
  input  logic        nak_sent,     // ctrl slot carrying a NAK left the device
  input  logic        retry_replay, // one retry-buffer flit replayed
  input  logic        tx_stall,     // new tx blocked: retry window full
  input  logic        rx_nrdy,      // rx queues cannot absorb a flit
  input  logic        tx_flit,      // any tx flit fired
  input  logic [2:0]  tx_slots,     // non-empty slots in a NEW tx flit (0 on replay)

  output logic [31:0] lnk_crc_errs,
  output logic [31:0] lnk_naks_sent,
  output logic [31:0] lnk_retries,
  output logic [31:0] lnk_tx_stall_cyc,
  output logic [31:0] lnk_rx_nrdy_cyc,
  output logic [31:0] lnk_tx_flits,
  output logic [31:0] lnk_tx_slots
);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || perf_reset) begin
      lnk_crc_errs     <= '0;
      lnk_naks_sent    <= '0;
      lnk_retries      <= '0;
      lnk_tx_stall_cyc <= '0;
      lnk_rx_nrdy_cyc  <= '0;
      lnk_tx_flits     <= '0;
      lnk_tx_slots     <= '0;
    end else begin
      if (crc_err)      lnk_crc_errs     <= lnk_crc_errs + 32'd1;
      if (nak_sent)     lnk_naks_sent    <= lnk_naks_sent + 32'd1;
      if (retry_replay) lnk_retries      <= lnk_retries + 32'd1;
      if (tx_stall)     lnk_tx_stall_cyc <= lnk_tx_stall_cyc + 32'd1;
      if (rx_nrdy)      lnk_rx_nrdy_cyc  <= lnk_rx_nrdy_cyc + 32'd1;
      if (tx_flit)      lnk_tx_flits     <= lnk_tx_flits + 32'd1;
      lnk_tx_slots <= lnk_tx_slots + 32'(tx_slots);
    end
  end

endmodule

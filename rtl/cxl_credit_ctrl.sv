// Device-side bookkeeping for the request-direction credit pools.
// The HOST consumes a credit per request slot it sends; the DEVICE returns credits as
// requests retire. This module accumulates pending returns and hands them to the
// arb/mux for piggybacking into the next LINK_CTRL slot. The device also derives the
// rx flit gate: a flit is only accepted when both protocol queues have room for a
// worst-case full flit (4 slots) — that gate is the RTL backpressure the credit test
// observes when the host deliberately over-runs its credit allowance.
module cxl_credit_ctrl
  import nmc_pkg::*;
(
  input  logic       clk,
  input  logic       rst_n,
  input  logic       io_retired,      // an IO request slot finished dispatch
  input  logic       mem_retired,     // a MEM request slot finished dispatch
  input  logic       returns_taken,   // arb/mux consumed the pending return counts
  output logic [7:0] io_credit_ret,   // pending credit returns (to LINK_CTRL slot)
  output logic [7:0] mem_credit_ret,
  output logic       returns_pending
);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      io_credit_ret  <= '0;
      mem_credit_ret <= '0;
    end else begin
      // retire and hand-off can coincide; the retired credit joins the *next* batch
      if (returns_taken) begin
        io_credit_ret  <= io_retired  ? 8'd1 : 8'd0;
        mem_credit_ret <= mem_retired ? 8'd1 : 8'd0;
      end else begin
        if (io_retired)  io_credit_ret  <= io_credit_ret  + 8'd1;
        if (mem_retired) mem_credit_ret <= mem_credit_ret + 8'd1;
      end
    end
  end

  assign returns_pending = (io_credit_ret != 0) || (mem_credit_ret != 0);

endmodule

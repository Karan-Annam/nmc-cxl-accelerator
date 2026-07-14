// Device-transmit-side retry storage, RETRY_DEPTH deep, indexed by the low
// bits of the 4-bit tx sequence number. Go-back-N: on a NAK for sequence N the
// buffer replays N, N+1, ... in original order. Entries are freed by
// piggybacked acks from the host (LINK_CTRL slots). The host->device direction
// has no RTL retry buffer on purpose: each side owns replay for what *it*
// transmits, so the host's replay buffer is the C++ model re-sending flits it
// still holds. Same shape as a real link, half the RTL.
module cxl_retry_buffer
  import nmc_pkg::*;
(
  input  logic                  clk,
  input  logic                  rst_n,
  // store path: every tx flit is written here as it is sent
  input  logic                  store_en,
  input  logic [CXL_SEQ_W-1:0]  store_seq,
  input  logic [CXL_FLIT_W-1:0] store_flit,
  // ack path: host acked all sequences up to and including ack_seq
  input  logic                  ack_en,
  input  logic [CXL_SEQ_W-1:0]  ack_seq,
  // nak path: host requests replay starting at nak_seq
  input  logic                  nak_en,
  input  logic [CXL_SEQ_W-1:0]  nak_seq,
  // replay interface: when replaying, the link layer transmits replay_flit and
  // pulses replay_advance after each successful handoff
  input  logic                  replay_advance,
  output logic                  replaying,
  output logic [CXL_SEQ_W-1:0]  replay_seq,
  output logic [CXL_FLIT_W-1:0] replay_flit,
  // window state: full when RETRY_DEPTH un-acked flits are outstanding
  output logic                  window_full
);

  localparam int IDXW = $clog2(RETRY_DEPTH);

  logic [CXL_FLIT_W-1:0] buf_q [RETRY_DEPTH];
  // window: [base_seq, next_seq) are outstanding (un-acked)
  logic [CXL_SEQ_W-1:0] base_seq, next_seq;
  logic [CXL_SEQ_W:0]   outstanding;   // 0..RETRY_DEPTH

  logic [CXL_SEQ_W-1:0] replay_ptr;
  logic                 replay_q;

  assign replaying   = replay_q;
  assign replay_seq  = replay_ptr;
  assign replay_flit = buf_q[replay_ptr[IDXW-1:0]];
  assign window_full = (outstanding >= RETRY_DEPTH[CXL_SEQ_W:0]);

  // distance helper on 4-bit modular sequence space
  function automatic logic [CXL_SEQ_W-1:0] seq_dist(
      input logic [CXL_SEQ_W-1:0] from_s, input logic [CXL_SEQ_W-1:0] to_s);
    return to_s - from_s;  // modular
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      base_seq    <= '0;
      next_seq    <= '0;
      outstanding <= '0;
      replay_q    <= 1'b0;
      replay_ptr  <= '0;
    end else begin : upd
      logic [CXL_SEQ_W:0] out_next;
      out_next = outstanding;

      if (store_en) begin
        buf_q[store_seq[IDXW-1:0]] <= store_flit;
        // store of a *new* flit (not a replay retransmit) grows the window
        if (store_seq == next_seq) begin
          next_seq <= next_seq + CXL_SEQ_W'(1);
          out_next = out_next + 1'b1;
        end
      end

      if (ack_en) begin
        // free everything up to and including ack_seq, if it is inside the window
        logic [CXL_SEQ_W-1:0] d;
        d = seq_dist(base_seq, ack_seq + CXL_SEQ_W'(1));
        if ({1'b0, d} <= out_next) begin
          base_seq <= ack_seq + CXL_SEQ_W'(1);
          out_next = out_next - {1'b0, d};
        end
      end

      outstanding <= out_next;

      if (nak_en) begin
        replay_q   <= 1'b1;
        replay_ptr <= nak_seq;
      end else if (replay_q && replay_advance) begin
        if (replay_ptr + CXL_SEQ_W'(1) == next_seq)
          replay_q <= 1'b0;              // replayed everything outstanding
        else
          replay_ptr <= replay_ptr + CXL_SEQ_W'(1);
      end
    end
  end

endmodule

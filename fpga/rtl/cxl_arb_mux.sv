// Device transmit-side ARB/MUX. Owns the two per-protocol response
// FIFOs (CXL.io and CXL.mem) and assembles up to 4 slots per tx flit: a LINK_CTRL
// slot first when acks/naks/credit-returns are owed, then round-robin between the
// io and mem response queues. Exactly two protocol lanes exist because CXL.cache is
// out of scope for a Type 3 device — this arbiter is complete, not a stub.
module cxl_arb_mux
  import nmc_pkg::*;
(
  input  logic                         clk,
  input  logic                         rst_n,
  // response pushes from the dispatchers
  input  logic                         io_push,
  input  logic [8*CXL_SLOT_BYTES-1:0]  io_push_slot,
  output logic                         io_full,
  input  logic                         mem_push,
  input  logic [8*CXL_SLOT_BYTES-1:0]  mem_push_slot,
  output logic                         mem_full,
  output logic [$clog2(RXQ_DEPTH):0]   mem_free,   // free mem-queue slots (for
                                                   // read-response reservation)
  // link-control slot (ack/nak/credit returns), built by the link layer
  input  logic                         ctrl_valid,
  input  logic [8*CXL_SLOT_BYTES-1:0]  ctrl_slot,
  // flit assembly
  input  logic                         flit_take,   // this cycle's assembly is sent
  output logic                         any_pending, // something to send
  output logic                         resp_pending,// io/mem response queued (shallow
                                                    // decode for the ctrl-only hold)
  output logic                         ctrl_taken,  // ctrl slot included in this flit
  output logic [1:0]                   slot_type [CXL_SLOTS_PER_FLIT],
  output logic [8*CXL_SLOT_BYTES-1:0]  slot_data [CXL_SLOTS_PER_FLIT]
);

  localparam int QD = RXQ_DEPTH;   // response queue depth
  localparam int PW = $clog2(QD);

  // --- two small FIFOs with 4-entry head lookahead ---
  logic [8*CXL_SLOT_BYTES-1:0] ioq [QD], memq [QD];
  logic [PW-1:0] io_rd, io_wr, mem_rd, mem_wr;
  logic [PW:0]   io_cnt, mem_cnt;

  assign io_full  = (io_cnt  == QD[PW:0]);
  assign mem_full = (mem_cnt == QD[PW:0]);
  assign mem_free = QD[PW:0] - mem_cnt;
  assign resp_pending = (io_cnt != '0) || (mem_cnt != '0);

  // --- slot selection (combinational) ---
  logic rr_q;   // 0: io first, 1: mem first
  logic [PW:0] io_take, mem_take;  // how many entries this flit consumes (0..4)

  // Closed-form take counts (the single source of truth for how many entries
  // each queue contributes): the original serial avail-decrement loop missed
  // 6.67 ns. With n data slots and saturated supplies a,b — if a+b <= n both
  // drain fully; else the round-robin share ai/bi applies until one side
  // exhausts and the remainder flows to the other. Fairness is preserved
  // (rr_q alternates ai between the protocols); only the placement WITHIN a
  // flit changed — io entries then mem entries, instead of interleaved —
  // which no receiver observes (slots are scanned, responses keyed by tag).
  logic [2:0] a4, b4, nslots, ai, bi, io_take_cf, mem_take_cf;
  assign a4     = (io_cnt  >= (PW+1)'(4)) ? 3'd4 : io_cnt[2:0];
  assign b4     = (mem_cnt >= (PW+1)'(4)) ? 3'd4 : mem_cnt[2:0];
  assign nslots = ctrl_valid ? 3'd3 : 3'd4;
  assign ai     = (!rr_q) ? ((nslots + 3'd1) >> 1) : (nslots >> 1);
  assign bi     = nslots - ai;
  always_comb begin
    if ({1'b0, a4} + {1'b0, b4} <= {1'b0, nslots}) begin
      io_take_cf  = a4;
      mem_take_cf = b4;
    end else if (a4 < ai) begin
      io_take_cf  = a4;
      mem_take_cf = nslots - a4;
    end else if (b4 < bi) begin
      io_take_cf  = nslots - b4;
      mem_take_cf = b4;
    end else begin
      io_take_cf  = ai;
      mem_take_cf = bi;
    end
  end

  // Parallel head windows: the next 4 entries of each queue, read at
  // addresses that do NOT depend on the selection — so the 16:1 FF-array
  // muxes run in parallel with the take-count math instead of behind it.
  logic [8*CXL_SLOT_BYTES-1:0] io_win [CXL_SLOTS_PER_FLIT];
  logic [8*CXL_SLOT_BYTES-1:0] mem_win [CXL_SLOTS_PER_FLIT];
  always_comb begin
    for (int k = 0; k < CXL_SLOTS_PER_FLIT; k++) begin
      io_win[k]  = ioq [(io_rd  + PW'(k)) % QD];
      mem_win[k] = memq[(mem_rd + PW'(k)) % QD];
    end
  end

  always_comb begin : sel
    logic [2:0] di;
    io_take    = (PW+1)'(io_take_cf);    // legacy names kept for the updater
    mem_take   = (PW+1)'(mem_take_cf);
    ctrl_taken = ctrl_valid;

    for (int s = 0; s < CXL_SLOTS_PER_FLIT; s++) begin
      slot_type[s] = SLOT_EMPTY;
      slot_data[s] = '0;
      if (s == 0 && ctrl_valid) begin
        slot_type[0] = SLOT_CTRL;
        slot_data[0] = ctrl_slot;
      end else begin
        di = 3'(s) - (ctrl_valid ? 3'd1 : 3'd0);   // data-slot position
        if (di < io_take_cf) begin
          slot_type[s] = SLOT_IO;
          slot_data[s] = io_win[2'(di)];
        end else if (di < io_take_cf + mem_take_cf) begin
          slot_type[s] = SLOT_MEM;
          slot_data[s] = mem_win[2'(di - io_take_cf)];
        end
      end
    end

    any_pending = ctrl_valid || (io_cnt != 0) || (mem_cnt != 0);
  end

  // --- state update ---
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      io_rd <= '0; io_wr <= '0; io_cnt <= '0;
      mem_rd <= '0; mem_wr <= '0; mem_cnt <= '0;
      rr_q <= 1'b0;
    end else begin : upd
      logic [PW:0] ioc, memc;
      ioc  = io_cnt;
      memc = mem_cnt;
      if (io_push && !io_full) begin
        ioq[io_wr] <= io_push_slot;
        io_wr <= (io_wr + PW'(1)) % QD;
        ioc = ioc + 1'b1;
      end
      if (mem_push && !mem_full) begin
        memq[mem_wr] <= mem_push_slot;
        mem_wr <= (mem_wr + PW'(1)) % QD;
        memc = memc + 1'b1;
      end
      if (flit_take) begin
        io_rd  <= (io_rd  + PW'(io_take_cf))  % QD;
        mem_rd <= (mem_rd + PW'(mem_take_cf)) % QD;
        ioc  = ioc  - (PW+1)'(io_take_cf);
        memc = memc - (PW+1)'(mem_take_cf);
        rr_q <= !rr_q;
      end
      io_cnt  <= ioc;
      mem_cnt <= memc;
    end
  end

endmodule

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
  // link-control slot (ack/nak/credit returns), built by the link layer
  input  logic                         ctrl_valid,
  input  logic [8*CXL_SLOT_BYTES-1:0]  ctrl_slot,
  // flit assembly
  input  logic                         flit_take,   // this cycle's assembly is sent
  output logic                         any_pending, // something to send
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

  // --- slot selection (combinational) ---
  logic rr_q;   // 0: io first, 1: mem first
  logic [PW:0] io_take, mem_take;  // how many entries this flit consumes (0..4)

  always_comb begin : sel
    logic [PW:0] io_avail, mem_avail;
    logic pick_io;
    int filled;
    io_avail  = io_cnt;
    mem_avail = mem_cnt;
    io_take   = '0;
    mem_take  = '0;
    ctrl_taken = 1'b0;
    filled = 0;

    for (int s = 0; s < CXL_SLOTS_PER_FLIT; s++) begin
      slot_type[s] = SLOT_EMPTY;
      slot_data[s] = '0;
    end

    if (ctrl_valid) begin
      slot_type[0] = SLOT_CTRL;
      slot_data[0] = ctrl_slot;
      ctrl_taken   = 1'b1;
      filled = 1;
    end

    pick_io = !rr_q;
    for (int s = 0; s < CXL_SLOTS_PER_FLIT; s++) begin
      if (s >= filled) begin
        // alternate protocols; fall back to whichever has entries
        if (pick_io ? (io_avail != 0) : (mem_avail == 0 && io_avail != 0)) begin
          slot_type[s] = SLOT_IO;
          slot_data[s] = ioq[(io_rd + io_take[PW-1:0]) % QD];
          io_take  = io_take + 1'b1;
          io_avail = io_avail - 1'b1;
        end else if (mem_avail != 0) begin
          slot_type[s] = SLOT_MEM;
          slot_data[s] = memq[(mem_rd + mem_take[PW-1:0]) % QD];
          mem_take  = mem_take + 1'b1;
          mem_avail = mem_avail - 1'b1;
        end
        pick_io = !pick_io;
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
        io_rd  <= (io_rd  + io_take[PW-1:0])  % QD;
        mem_rd <= (mem_rd + mem_take[PW-1:0]) % QD;
        ioc  = ioc  - io_take;
        memc = memc - mem_take;
        rr_q <= !rr_q;
      end
      io_cnt  <= ioc;
      mem_cnt <= memc;
    end
  end

endmodule

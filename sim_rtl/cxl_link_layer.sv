// The CXL flit/link layer. Sits between nmc_top's external flit ports and the
// (unchanged) cxl_controller mmio/hdm ports. Receives host flits, CRC-checks
// them, demuxes slots into per-protocol rx queues (no head-of-line blocking
// between CXL.io and CXL.mem), dispatches transactions to the controller, and
// transmits CRC-protected, sequence-numbered, retry-buffered response flits
// assembled by the ARB/MUX. Flit format is documented in nmc_pkg.sv.
module cxl_link_layer
  import nmc_pkg::*;
(
  input  logic                  clk,
  input  logic                  rst_n,

  // external flit boundary
  input  logic                  flit_rx_valid,
  input  logic [CXL_FLIT_W-1:0] flit_rx_data,
  output logic                  flit_rx_ready,
  output logic                  flit_tx_valid,
  output logic [CXL_FLIT_W-1:0] flit_tx_data,
  input  logic                  flit_tx_ready,

  // CXL.io side (to cxl_controller MMIO) — read data returns combinationally
  output logic                  mmio_valid,
  output logic                  mmio_write,
  output logic [7:0]            mmio_offset,
  output logic [DATA_WIDTH-1:0] mmio_wdata,
  input  logic [DATA_WIDTH-1:0] mmio_rdata,

  // CXL.mem side (to cxl_controller HDM port)
  output logic                  hdm_valid,
  output logic                  hdm_write,
  output logic [ADDR_WIDTH-1:0] hdm_addr,
  output logic [DATA_WIDTH-1:0] hdm_wdata,
  input  logic                  hdm_ready,
  input  logic                  hdm_rvalid,
  input  logic [DATA_WIDTH-1:0] hdm_rdata,

  // link perf counters (MMIO-visible via cxl_controller)
  input  logic                  perf_reset,
  output logic [31:0]           lnk_crc_errs,
  output logic [31:0]           lnk_naks_sent,
  output logic [31:0]           lnk_retries,
  output logic [31:0]           lnk_tx_stall_cyc,
  output logic [31:0]           lnk_rx_nrdy_cyc,
  output logic [31:0]           lnk_tx_flits,
  output logic [31:0]           lnk_tx_slots
);

  localparam int SLOT_W = 8*CXL_SLOT_BYTES;
  localparam int QD  = RXQ_DEPTH;
  localparam int PW  = $clog2(QD);

  // ------------------------------------------------------------------
  // RX: unpack + CRC check
  // ------------------------------------------------------------------
  logic                 rx_crc_ok;
  logic [CXL_SEQ_W-1:0] rx_seq;
  logic [1:0]           rx_stype [CXL_SLOTS_PER_FLIT];
  logic [SLOT_W-1:0]    rx_sdata [CXL_SLOTS_PER_FLIT];

  cxl_flit_unpack u_unpack (
    .flit(flit_rx_data), .crc_ok(rx_crc_ok), .rx_seq(rx_seq),
    .slot_type(rx_stype), .slot_data(rx_sdata)
  );

  // per-protocol rx slot queues
  logic [SLOT_W-1:0] io_rxq [QD], mem_rxq [QD];
  logic [PW-1:0] iorx_rd, iorx_wr, memrx_rd, memrx_wr;
  logic [PW:0]   iorx_cnt, memrx_cnt;

  // accept a flit only when both queues can absorb a worst-case 4 slots
  assign flit_rx_ready = ((QD[PW:0] - iorx_cnt)  >= CXL_SLOTS_PER_FLIT[PW:0]) &&
                         ((QD[PW:0] - memrx_cnt) >= CXL_SLOTS_PER_FLIT[PW:0]);

  logic rx_fire;
  assign rx_fire = flit_rx_valid && flit_rx_ready;

  // link-control state harvested from rx flits
  logic                 ack_pending_q;     // we owe the host an ack
  logic [CXL_SEQ_W-1:0] last_good_rx_seq;
  logic                 naks_pending_q;    // we owe the host a NAK (bad CRC seen)
  logic [CXL_SEQ_W-1:0] nak_rx_seq;

  // retry-buffer control strobes (parsed from host LINK_CTRL slots)
  logic                 host_ack_en, host_nak_en;
  logic [CXL_SEQ_W-1:0] host_ack_seq, host_nak_seq;

  always_comb begin
    host_ack_en  = 1'b0;
    host_nak_en  = 1'b0;
    host_ack_seq = '0;
    host_nak_seq = '0;
    if (rx_fire && rx_crc_ok) begin
      for (int s = 0; s < CXL_SLOTS_PER_FLIT; s++) begin
        if (rx_stype[s] == SLOT_CTRL) begin
          if (rx_sdata[s][0]) begin       // ack_valid
            host_ack_en  = 1'b1;
            host_ack_seq = rx_sdata[s][8 +: CXL_SEQ_W];
          end
          if (rx_sdata[s][1]) begin       // nak_valid
            host_nak_en  = 1'b1;
            host_nak_seq = rx_sdata[s][8 +: CXL_SEQ_W];
          end
        end
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      iorx_rd <= '0; iorx_wr <= '0; iorx_cnt <= '0;
      memrx_rd <= '0; memrx_wr <= '0; memrx_cnt <= '0;
      ack_pending_q <= 1'b0; last_good_rx_seq <= '0;
      naks_pending_q <= 1'b0; nak_rx_seq <= '0;
    end else begin : rxupd
      logic [PW:0] ioc, memc;
      logic [PW-1:0] iow, memw;
      ioc = iorx_cnt; memc = memrx_cnt;
      iow = iorx_wr;  memw = memrx_wr;

      if (rx_fire) begin
        if (rx_crc_ok) begin
          for (int s = 0; s < CXL_SLOTS_PER_FLIT; s++) begin
            if (rx_stype[s] == SLOT_IO) begin
              io_rxq[iow] <= rx_sdata[s];
              iow = (iow + PW'(1)) % QD;
              ioc = ioc + 1'b1;
            end else if (rx_stype[s] == SLOT_MEM) begin
              mem_rxq[memw] <= rx_sdata[s];
              memw = (memw + PW'(1)) % QD;
              memc = memc + 1'b1;
            end
          end
          ack_pending_q    <= 1'b1;
          last_good_rx_seq <= rx_seq;
        end else begin
          naks_pending_q <= 1'b1;
          nak_rx_seq     <= rx_seq;
        end
      end

      // dispatch pops (see below)
      if (io_pop)  begin iorx_rd <= (iorx_rd + PW'(1)) % QD;  ioc = ioc - 1'b1; end
      if (mem_pop) begin memrx_rd <= (memrx_rd + PW'(1)) % QD; memc = memc - 1'b1; end

      iorx_wr <= iow;  memrx_wr <= memw;
      iorx_cnt <= ioc; memrx_cnt <= memc;

      // clear owed ack/nak once a ctrl slot carrying them is transmitted
      if (ctrl_sent) begin
        if (!naks_pending_q) ack_pending_q <= (rx_fire && rx_crc_ok); // keep fresh ack
        naks_pending_q <= (rx_fire && !rx_crc_ok);
      end
    end
  end

  // ------------------------------------------------------------------
  // Dispatchers: rx queues → controller ports → response slots
  // ------------------------------------------------------------------
  logic io_pop, mem_pop;
  logic io_retired;

  // -- CXL.io (MMIO): controller answers reads combinationally
  logic [SLOT_W-1:0] io_head;
  assign io_head = io_rxq[iorx_rd];

  logic io_resp_push;
  logic [SLOT_W-1:0] io_resp_slot;
  logic io_respq_full;

  always_comb begin
    mmio_valid   = 1'b0;
    mmio_write   = 1'b0;
    mmio_offset  = io_head[7:0];         // word0[15:0] address → offset byte
    mmio_wdata   = io_head[32 +: 32];    // word1
    io_pop       = 1'b0;
    io_resp_push = 1'b0;
    io_resp_slot = '0;
    io_retired   = 1'b0;
    if (iorx_cnt != 0) begin
      if (io_head[31]) begin             // write: posted
        mmio_valid = 1'b1;
        mmio_write = 1'b1;
        io_pop     = 1'b1;
        io_retired = 1'b1;
      end else if (!io_respq_full) begin // read: response slot
        mmio_valid   = 1'b1;
        io_resp_push = 1'b1;
        io_resp_slot[31:0]    = io_head[31:0] | 32'h4000_0000;   // is_response
        io_resp_slot[32 +: 32] = mmio_rdata;
        io_pop       = 1'b1;
        io_retired   = 1'b1;
      end
    end
  end

  // -- CXL.mem (HDM): streaming dispatcher. Writes retire one per cycle;
  //    reads issue back-to-back into a small pending FIFO (the SRAM answers
  //    one cycle later) and responses push to the mem response queue in
  //    order. A read only issues when a response slot is provably reservable
  //    (mem_free > pend_cnt) — the arb/mux silently drops pushes when full,
  //    so the reservation is what makes pipelining safe.
  logic [SLOT_W-1:0] mem_head;
  assign mem_head = mem_rxq[memrx_rd];

  // burst decode (nmc_pkg slot layout): count-1 words at addr, addr+1, addr+2
  logic       head_burst;
  logic [1:0] head_cnt1;
  assign head_burst = mem_head[29];
  assign head_cnt1  = head_burst ? mem_head[25:24] : 2'd0;

  localparam int MPEND = 4;
  logic [31:0] pend_w0   [MPEND];   // word0 of in-flight reads (tag/addr echo)
  logic [1:0]  pend_lane [MPEND];   // which response word this read fills
  logic        pend_last [MPEND];   // final word of its slot → push the response
  logic [1:0]  pend_rd, pend_wr;
  logic [2:0]  pend_cnt;
  logic [PW:0] mem_free;

  logic mem_resp_push;
  logic [SLOT_W-1:0] mem_resp_slot;
  logic mem_respq_full;
  logic [1:0] mem_retired_cnt;
  logic [31:0] stage_d0, stage_d1;   // burst response words awaiting the last

  logic [1:0]  bcnt_q;   // word index within the head slot (issue side)
  logic        slot_done;
  assign slot_done = (bcnt_q == head_cnt1);

  logic rd_issue_ok, rd_fire, wr_fire, resp_last;
  assign rd_issue_ok = ({29'd0, pend_cnt} < 32'(mem_free)) && (pend_cnt < 3'(MPEND));
  assign rd_fire = (memrx_cnt != 0) && !mem_head[31] && rd_issue_ok && hdm_ready;
  assign wr_fire = (memrx_cnt != 0) &&  mem_head[31] && hdm_ready;
  assign resp_last = hdm_rvalid && pend_last[pend_rd];

  always_comb begin
    hdm_valid = (memrx_cnt != 0) && (mem_head[31] || rd_issue_ok);
    hdm_write = mem_head[31];
    hdm_addr  = mem_head[15:0] + {14'd0, bcnt_q};
    hdm_wdata = mem_head[(32 * ({3'd0, bcnt_q} + 5'd1)) +: 32];
    mem_pop   = (wr_fire || rd_fire) && slot_done;

    // response for the oldest in-flight read (data arrives 1 cycle after
    // fire); burst words stage until the slot's last word lands
    mem_resp_push = resp_last;
    mem_resp_slot = '0;
    mem_resp_slot[31:0] = pend_w0[pend_rd] | 32'h4000_0000;
    unique case (pend_lane[pend_rd])
      2'd0: mem_resp_slot[32 +: 32] = hdm_rdata;
      2'd1: begin
        mem_resp_slot[32 +: 32] = stage_d0;
        mem_resp_slot[64 +: 32] = hdm_rdata;
      end
      default: begin
        mem_resp_slot[32 +: 32] = stage_d0;
        mem_resp_slot[64 +: 32] = stage_d1;
        mem_resp_slot[96 +: 32] = hdm_rdata;
      end
    endcase

    // credits are per SLOT: a write slot retires with its last word, a read
    // slot when its response pushes — the two can coincide
    mem_retired_cnt = {1'b0, wr_fire && slot_done} + {1'b0, resp_last};
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pend_rd <= '0; pend_wr <= '0; pend_cnt <= '0;
      bcnt_q <= '0; stage_d0 <= '0; stage_d1 <= '0;
    end else begin : pupd
      logic [2:0] pc;
      pc = pend_cnt;
      if (wr_fire || rd_fire) bcnt_q <= slot_done ? 2'd0 : (bcnt_q + 2'd1);
      if (rd_fire) begin
        pend_w0[pend_wr]   <= mem_head[31:0];
        pend_lane[pend_wr] <= bcnt_q;
        pend_last[pend_wr] <= slot_done;
        pend_wr <= pend_wr + 2'd1;
        pc = pc + 3'd1;
      end
      if (hdm_rvalid) begin
        if (!pend_last[pend_rd]) begin
          if (pend_lane[pend_rd] == 2'd0) stage_d0 <= hdm_rdata;
          else                            stage_d1 <= hdm_rdata;
        end
        pend_rd <= pend_rd + 2'd1;
        pc = pc - 3'd1;
      end
      pend_cnt <= pc;
    end
  end

  // ------------------------------------------------------------------
  // Credits (request-direction pools; device side accumulates returns)
  // ------------------------------------------------------------------
  logic [7:0] io_cr_ret, mem_cr_ret;
  logic       cr_pending, ctrl_taken, ctrl_sent;

  cxl_credit_ctrl u_credit (
    .clk(clk), .rst_n(rst_n),
    .io_retired(io_retired), .mem_retired_cnt(mem_retired_cnt),
    .returns_taken(ctrl_sent),
    .io_credit_ret(io_cr_ret), .mem_credit_ret(mem_cr_ret),
    .returns_pending(cr_pending)
  );

  // ------------------------------------------------------------------
  // TX: ARB/MUX → pack → retry buffer → flit_tx
  // ------------------------------------------------------------------
  logic ctrl_valid;
  logic [SLOT_W-1:0] ctrl_slot;

  always_comb begin
    ctrl_valid = ack_pending_q || naks_pending_q || cr_pending;
    ctrl_slot  = '0;
    // one seq field: NAK takes precedence (host must replay before acks matter)
    if (naks_pending_q) begin
      ctrl_slot[1]        = 1'b1;
      ctrl_slot[8 +: CXL_SEQ_W] = nak_rx_seq;
    end else if (ack_pending_q) begin
      ctrl_slot[0]        = 1'b1;
      ctrl_slot[8 +: CXL_SEQ_W] = last_good_rx_seq;
    end
    ctrl_slot[16 +: 8] = io_cr_ret;
    ctrl_slot[24 +: 8] = mem_cr_ret;
  end

  logic any_pending;
  logic [1:0]        tx_stype [CXL_SLOTS_PER_FLIT];
  logic [SLOT_W-1:0] tx_sdata [CXL_SLOTS_PER_FLIT];
  logic flit_take;

  cxl_arb_mux u_arb (
    .clk(clk), .rst_n(rst_n),
    .io_push(io_resp_push), .io_push_slot(io_resp_slot), .io_full(io_respq_full),
    .mem_push(mem_resp_push), .mem_push_slot(mem_resp_slot), .mem_full(mem_respq_full),
    .mem_free(mem_free),
    .ctrl_valid(ctrl_valid), .ctrl_slot(ctrl_slot),
    .flit_take(flit_take), .any_pending(any_pending), .ctrl_taken(ctrl_taken),
    .slot_type(tx_stype), .slot_data(tx_sdata)
  );

  logic [CXL_SEQ_W-1:0] tx_seq_q;
  logic [CXL_FLIT_W-1:0] tx_flit_new;

  cxl_flit_pack u_pack (
    .slot_type(tx_stype), .slot_data(tx_sdata), .tx_seq(tx_seq_q), .flit(tx_flit_new)
  );

  // retry buffer
  logic ret_replaying, ret_window_full, ret_advance;
  logic [CXL_SEQ_W-1:0]  ret_replay_seq;
  logic [CXL_FLIT_W-1:0] ret_replay_flit;
  logic tx_store;

  cxl_retry_buffer u_retry (
    .clk(clk), .rst_n(rst_n),
    .store_en(tx_store), .store_seq(tx_seq_q), .store_flit(tx_flit_new),
    .ack_en(host_ack_en), .ack_seq(host_ack_seq),
    .nak_en(host_nak_en), .nak_seq(host_nak_seq),
    .replay_advance(ret_advance),
    .replaying(ret_replaying), .replay_seq(ret_replay_seq),
    .replay_flit(ret_replay_flit),
    .window_full(ret_window_full)
  );

  // tx selection: replay has priority; new flits stall while the window is full
  logic send_new;
  assign send_new      = !ret_replaying && any_pending && !ret_window_full;
  assign flit_tx_valid = ret_replaying || send_new;
  assign flit_tx_data  = ret_replaying ? ret_replay_flit : tx_flit_new;

  logic tx_fire;
  assign tx_fire     = flit_tx_valid && flit_tx_ready;
  assign ret_advance = tx_fire && ret_replaying;
  assign flit_take   = tx_fire && send_new;
  assign tx_store    = tx_fire && send_new;
  assign ctrl_sent   = flit_take && ctrl_taken;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) tx_seq_q <= '0;
    else if (tx_store) tx_seq_q <= tx_seq_q + CXL_SEQ_W'(1);
  end

  // ------------------------------------------------------------------
  // link perf counters
  // ------------------------------------------------------------------
  // Slots are only counted for NEW flits: a replayed flit's slots were
  // already counted when it was first transmitted.
  logic [2:0] tx_slots_used;
  always_comb begin
    tx_slots_used = '0;
    if (tx_fire && send_new)
      for (int s = 0; s < CXL_SLOTS_PER_FLIT; s++)
        if (tx_stype[s] != SLOT_EMPTY) tx_slots_used = tx_slots_used + 3'd1;
  end

  cxl_link_perf u_lnkperf (
    .clk(clk), .rst_n(rst_n), .perf_reset(perf_reset),
    .crc_err     (rx_fire && !rx_crc_ok),
    .nak_sent    (ctrl_sent && naks_pending_q),
    .retry_replay(ret_advance),
    .tx_stall    (any_pending && !ret_replaying && ret_window_full),
    .rx_nrdy     (!flit_rx_ready),
    .tx_flit     (tx_fire),
    .tx_slots    (tx_slots_used),
    .lnk_crc_errs(lnk_crc_errs), .lnk_naks_sent(lnk_naks_sent),
    .lnk_retries(lnk_retries), .lnk_tx_stall_cyc(lnk_tx_stall_cyc),
    .lnk_rx_nrdy_cyc(lnk_rx_nrdy_cyc), .lnk_tx_flits(lnk_tx_flits),
    .lnk_tx_slots(lnk_tx_slots)
  );

endmodule

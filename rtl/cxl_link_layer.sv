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
  input  logic [DATA_WIDTH-1:0] hdm_rdata
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
  logic io_retired, mem_retired;

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

  // -- CXL.mem (HDM): 1-cycle read latency, single outstanding
  logic [SLOT_W-1:0] mem_head;
  assign mem_head = mem_rxq[memrx_rd];

  typedef enum logic [1:0] { M_IDLE, M_WAITR, M_PUSHR } mstate_e;
  mstate_e mstate_q;
  logic [31:0] mem_req_w0_q;   // word0 of in-flight read (for response tag/addr)
  logic [31:0] mem_rdata_q;

  logic mem_resp_push;
  logic [SLOT_W-1:0] mem_resp_slot;
  logic mem_respq_full;

  always_comb begin
    hdm_valid     = 1'b0;
    hdm_write     = 1'b0;
    hdm_addr      = mem_head[15:0];
    hdm_wdata     = mem_head[32 +: 32];
    mem_pop       = 1'b0;
    mem_retired   = 1'b0;
    mem_resp_push = 1'b0;
    mem_resp_slot = '0;
    unique case (mstate_q)
      M_IDLE: begin
        if (memrx_cnt != 0) begin
          hdm_valid = 1'b1;
          hdm_write = mem_head[31];
          if (hdm_ready) begin
            mem_pop = 1'b1;
            if (mem_head[31]) mem_retired = 1'b1;   // posted write retires now
          end
        end
      end
      M_WAITR: ;   // waiting for hdm_rvalid (handled in ff)
      M_PUSHR: begin
        if (!mem_respq_full) begin
          mem_resp_push = 1'b1;
          mem_resp_slot[31:0]     = mem_req_w0_q | 32'h4000_0000;
          mem_resp_slot[32 +: 32] = mem_rdata_q;
          mem_retired = 1'b1;
        end
      end
      default: ;
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mstate_q <= M_IDLE;
      mem_req_w0_q <= '0;
      mem_rdata_q  <= '0;
    end else begin
      unique case (mstate_q)
        M_IDLE:  if (memrx_cnt != 0 && hdm_ready && !mem_head[31]) begin
                   mem_req_w0_q <= mem_head[31:0];
                   mstate_q <= M_WAITR;
                 end
        M_WAITR: if (hdm_rvalid) begin
                   mem_rdata_q <= hdm_rdata;
                   mstate_q <= M_PUSHR;
                 end
        M_PUSHR: if (!mem_respq_full) mstate_q <= M_IDLE;
        default: mstate_q <= M_IDLE;
      endcase
    end
  end

  // ------------------------------------------------------------------
  // Credits (request-direction pools; device side accumulates returns)
  // ------------------------------------------------------------------
  logic [7:0] io_cr_ret, mem_cr_ret;
  logic       cr_pending, ctrl_taken, ctrl_sent;

  cxl_credit_ctrl u_credit (
    .clk(clk), .rst_n(rst_n),
    .io_retired(io_retired), .mem_retired(mem_retired),
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

endmodule

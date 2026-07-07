// The scatter/gather engine — the module this project exists for. Resolves
// dense strided and sparse index-list access patterns into per-bank SRAM
// addresses and routing selects, entirely on-device. The NMC engine FSM
// strobes this module; all address arithmetic (bank striping, two-level
// indirection, row/element stepping) lives here.
//
// Dense mode: 8 lanes per group; element i = 8g+p is processed by PE p, reading
//   operand A from bank (src_a+i)%8 and operand B from bank (src_b+i)%8.
// Sparse mode: two-cycle-per-word schedule — the FSM first strobes an index
//   fetch (idx_addr), latches the arriving index word (step_idx), then strobes
//   data fetches for each inner element d of the row (data_addr). Fetches sit
//   on alternating cycles/ports so no bank port can double-book, whatever the
//   index pattern.
module scatter_gather_engine
  import nmc_pkg::*;
(
  input  logic                  clk,
  input  logic                  rst_n,

  // command latch
  input  logic                  start,       // latch bases/lengths, clear counters
  input  logic [ADDR_WIDTH-1:0] src_a,
  input  logic [ADDR_WIDTH-1:0] src_b,
  input  logic [ADDR_WIDTH-1:0] dst,
  input  logic [ADDR_WIDTH-1:0] idx_base,
  input  logic [ADDR_WIDTH-1:0] stride,
  input  logic [15:0]           len,
  input  logic [15:0]           idx_len,

  // FSM strobes
  input  logic                  step_group,  // dense: advance to next 8-element group
  input  logic                  step_idx,    // sparse: latch idx_rdata as j, d <= 0
  input  logic                  step_d,      // sparse: advance inner element d
  input  logic                  step_m,      // sparse: advance index m
  input  logic [DATA_WIDTH-1:0] idx_rdata,   // index word arriving from SRAM

  // ---- dense outputs (all 8 lanes of the current group) ----
  output logic [BANK_AW-1:0]    dn_offA  [PE_COUNT],  // port-A offset for bank k
  output logic [BANK_AW-1:0]    dn_offB  [PE_COUNT],  // port-B offset for bank k
  output logic [2:0]            dn_abank [PE_COUNT],  // PE p's operand-A source bank
  output logic [2:0]            dn_bbank [PE_COUNT],  // PE p's operand-B source bank
  output logic [2:0]            dn_wbank [PE_COUNT],  // PE p's writeback target bank
  output logic [BANK_AW-1:0]    dn_woff  [PE_COUNT],
  output logic [PE_COUNT-1:0]   dn_lane_valid,        // element 8g+p < len
  output logic                  dn_group_last,        // this is the final group

  // ---- sparse outputs ----
  output logic [2:0]            sp_idx_bank,
  output logic [BANK_AW-1:0]    sp_idx_off,           // idx_base + m
  output logic [2:0]            sp_data_bank,         // src_a + j*stride + d
  output logic [BANK_AW-1:0]    sp_data_off,
  output logic [2:0]            sp_b_bank,            // SPARSE: src_b+d, WGATHER: src_b+m
  output logic [BANK_AW-1:0]    sp_b_off,
  input  logic                  b_by_m,               // 1: operand B indexed by m (WGATHER)
  output logic [2:0]            sp_emb_wbank,         // dst + m*stride + d
  output logic [BANK_AW-1:0]    sp_emb_woff,
  output logic [2:0]            sp_out_wbank,         // dst + m  (per-index scalar out)
  output logic [BANK_AW-1:0]    sp_out_woff,
  output logic                  sp_mask_bit,          // latched index word MSB
  output logic [15:0]           sp_m,
  output logic [ADDR_WIDTH-1:0] sp_d,
  output logic                  sp_m_last,
  output logic                  sp_d_last
);

  // latched command
  logic [ADDR_WIDTH-1:0] a_q, b_q, dst_q, idx_q, stride_q;
  logic [15:0]           len_q, ilen_q;

  // dense group counter (group base = 8*grp_q)
  logic [15:0] grp_q;

  // sparse counters + latched index
  logic [15:0]           m_q;
  logic [ADDR_WIDTH-1:0] d_q;
  logic [ADDR_WIDTH-1:0] j_q;
  logic                  mask_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      a_q <= '0; b_q <= '0; dst_q <= '0; idx_q <= '0; stride_q <= ADDR_WIDTH'(1);
      len_q <= '0; ilen_q <= '0;
      grp_q <= '0; m_q <= '0; d_q <= '0; j_q <= '0; mask_q <= 1'b0;
    end else begin
      if (start) begin
        a_q <= src_a; b_q <= src_b; dst_q <= dst; idx_q <= idx_base;
        stride_q <= (stride == 0) ? ADDR_WIDTH'(1) : stride;
        len_q <= len; ilen_q <= idx_len;
        grp_q <= '0; m_q <= '0; d_q <= '0; j_q <= '0; mask_q <= 1'b0;
      end else begin
        if (step_group) grp_q <= grp_q + 16'd1;
        if (step_idx) begin
          j_q    <= idx_rdata[ADDR_WIDTH-1:0];
          mask_q <= idx_rdata[31];
          d_q    <= '0;
        end
        if (step_d) d_q <= d_q + ADDR_WIDTH'(1);
        if (step_m) begin
          m_q <= m_q + 16'd1;
          d_q <= '0;
        end
      end
    end
  end

  // ---------------- dense address generation ----------------
  logic [ADDR_WIDTH-1:0] gbase;
  assign gbase = ADDR_WIDTH'(grp_q) << 3;

  always_comb begin : lanes
    logic [ADDR_WIDTH-1:0] ia, ib, aaddr, baddr, waddr;
    for (int k = 0; k < PE_COUNT; k++) begin
      // element whose A-address lands in bank k
      ia = gbase + ADDR_WIDTH'((3'(k) - a_q[2:0]) & 3'h7);
      aaddr = a_q + ia;
      dn_offA[k] = off_of(aaddr);
      // element whose B-address lands in bank k
      ib = gbase + ADDR_WIDTH'((3'(k) - b_q[2:0]) & 3'h7);
      baddr = b_q + ib;
      dn_offB[k] = off_of(baddr);
      // PE k routing: element i = gbase + k
      dn_abank[k] = (a_q[2:0] + 3'(k)) & 3'h7;
      dn_bbank[k] = (b_q[2:0] + 3'(k)) & 3'h7;
      waddr = dst_q + gbase + ADDR_WIDTH'(k);
      dn_wbank[k] = bank_of(waddr);
      dn_woff[k]  = off_of(waddr);
      dn_lane_valid[k] = ({16'd0, gbase} + 32'(k)) < {16'd0, len_q};
    end
  end
  assign dn_group_last = ({16'd0, gbase} + 32'd8) >= {16'd0, len_q};

  // ---------------- sparse address generation ----------------
  logic [ADDR_WIDTH-1:0] idx_addr, data_addr, b_addr, emb_waddr, out_waddr;
  logic [31:0] row_base, emb_base;

  assign idx_addr  = idx_q + ADDR_WIDTH'(m_q);
  assign row_base  = 32'(j_q) * 32'(stride_q);
  assign emb_base  = 32'(m_q) * 32'(stride_q);
  assign data_addr = a_q + row_base[ADDR_WIDTH-1:0] + d_q;
  assign b_addr    = b_by_m ? (b_q + ADDR_WIDTH'(m_q)) : (b_q + d_q);
  assign emb_waddr = dst_q + emb_base[ADDR_WIDTH-1:0] + d_q;
  assign out_waddr = dst_q + ADDR_WIDTH'(m_q);

  assign sp_idx_bank  = bank_of(idx_addr);
  assign sp_idx_off   = off_of(idx_addr);
  assign sp_data_bank = bank_of(data_addr);
  assign sp_data_off  = off_of(data_addr);
  assign sp_b_bank    = bank_of(b_addr);
  assign sp_b_off     = off_of(b_addr);
  assign sp_emb_wbank = bank_of(emb_waddr);
  assign sp_emb_woff  = off_of(emb_waddr);
  assign sp_out_wbank = bank_of(out_waddr);
  assign sp_out_woff  = off_of(out_waddr);
  assign sp_mask_bit  = mask_q;
  assign sp_m         = m_q;
  assign sp_d         = d_q;
  assign sp_m_last    = (m_q + 16'd1) >= ilen_q;
  assign sp_d_last    = (d_q + ADDR_WIDTH'(1)) >= stride_q;

endmodule

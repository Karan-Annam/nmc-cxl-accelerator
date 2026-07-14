// The scatter/gather engine — the module this project exists for. Resolves
// dense strided and sparse index-list access patterns into per-bank SRAM
// addresses and routing selects, entirely on-device. The NMC engine FSM
// strobes this module; all address arithmetic (bank striping, two-level
// indirection, row/element stepping) lives here.
//
// Dense mode: 8 lanes per group; element i = 8g+p is processed by PE p, reading
//   operand A from bank (src_a+i)%8 and operand B from bank (src_b+i)%8.
// Sparse row modes (SPARSE dot / WGATHER / EMBEDDING): after the index fetch
//   resolves the row base, the row walk is 8-wide — structurally the dense
//   lane pattern with the row base substituted for src_a and a chunk counter
//   for the group counter. An 8-element aligned chunk of a unit-stride stream
//   touches each bank exactly once (addr%8 is a bijection over the chunk), and
//   the chunk base is a multiple of 8, so the lane→bank rotation is constant
//   for the whole row. A-stream owns port A, B-stream owns port B: no bank
//   port can double-book, whatever the index pattern.
// Timing: all row-base math is registered. The index list is prefetched TWO
//   rows ahead (idx[m+2] issues in row m's end-of-row bubble), so idx[m+1]
//   sits in j_nxt_q a full row before it is needed and the j*stride multiply
//   runs FF→DSP→FF at the row boundary (row_latch). The old fused prime
//   multiplied the live SRAM word into the next read address in one cycle —
//   a BRAM→DSP→adders→BRAM-setup cone that broke timing at 10 ns.
// REDUCTION keeps the original two-cycle-per-word schedule (idx fetch, then
//   one data fetch per element): its per-index gathers are arbitrary
//   addresses, the one pattern where 8 simultaneous fetches could collide on
//   a bank.
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
  input  logic                  step_idx,    // narrow: latch idx_rdata as j, d/chunk <= 0
  input  logic                  step_d,      // sparse narrow: advance inner element d
  input  logic                  step_chunk,  // sparse wide: advance 8-element chunk
  input  logic                  step_m,      // sparse: advance index m
  input  logic [DATA_WIDTH-1:0] idx_rdata,   // index word arriving from SRAM
  input  logic                  idx_arrive,  // wide: latch idx_rdata as NEXT row's index
  input  logic                  row_latch,   // wide: row_base_q <= j_nxt_q * stride
  input  logic                  red_mul,     // narrow: row_base_q <= j_q * stride

  // ---- dense outputs (all 8 lanes of the current group) ----
  output logic [BANK_AW-1:0]    dn_offA  [PE_COUNT],  // port-A offset for bank k
  output logic [BANK_AW-1:0]    dn_offB  [PE_COUNT],  // port-B offset for bank k
  output logic [2:0]            dn_abank [PE_COUNT],  // PE p's operand-A source bank
  output logic [2:0]            dn_bbank [PE_COUNT],  // PE p's operand-B source bank
  output logic [2:0]            dn_wbank [PE_COUNT],  // PE p's writeback target bank
  output logic [BANK_AW-1:0]    dn_woff  [PE_COUNT],
  output logic [PE_COUNT-1:0]   dn_lane_valid,        // element 8g+p < len
  output logic                  dn_group_last,        // this is the final group

  // ---- sparse outputs (narrow / shared) ----
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
  output logic                  sp_d_last,

  // ---- sparse outputs (8-wide row walk; chunk = 8 consecutive elements) ----
  output logic [BANK_AW-1:0]    sp8_offA [PE_COUNT],  // port-A offset for bank k
  output logic [BANK_AW-1:0]    sp8_offB [PE_COUNT],  // port-B offset for bank k (SPARSE)
  output logic [2:0]            sp8_arot,             // lane p's A data is in bank (arot+p)&7
  output logic [2:0]            sp8_brot,
  output logic [2:0]            sp8_wbank [PE_COUNT], // EMBEDDING: lane p's write target
  output logic [BANK_AW-1:0]    sp8_woff  [PE_COUNT],
  output logic [PE_COUNT-1:0]   sp8_lane_valid,       // chunk element 8c+p < stride (row len)
  output logic                  sp8_chunk_last,
  output logic                  sp8_mask,             // current row's mask (registered pipe)

  // ---- prefetch addresses (issued in the row-end bubble / priming) ----
  output logic [2:0]            sp_idx_pf_bank,       // idx_base + m + 1
  output logic [BANK_AW-1:0]    sp_idx_pf_off,
  output logic [2:0]            sp_idx_pf2_bank,      // idx_base + m + 2 (wide 2-ahead)
  output logic [BANK_AW-1:0]    sp_idx_pf2_off,
  output logic                  sp_idx_pf2_v,         // m + 2 < idx_len
  output logic [2:0]            sp_b_pf_bank,         // src_b + m + 1 (WGATHER weight)
  output logic [BANK_AW-1:0]    sp_b_pf_off
);

  // latched command
  logic [ADDR_WIDTH-1:0] a_q, b_q, dst_q, idx_q, stride_q;
  logic [15:0]           len_q, ilen_q;

  // dense group counter (group base = 8*grp_q)
  logic [15:0] grp_q;

  // sparse counters + latched index
  logic [15:0]           m_q;
  logic [ADDR_WIDTH-1:0] d_q;
  logic [15:0]           chk_q;   // wide-walk chunk counter (chunk base = 8*chk_q)
  logic [ADDR_WIDTH-1:0] j_q;
  logic                  mask_q;

  // registered row-base pipeline (wide walk): idx[m+1] parks in j_nxt_q for a
  // full row (2-ahead prefetch), then row_latch runs the one j*stride multiply
  // FF→DSP→FF at the row boundary. red_mul reuses the same multiplier for the
  // narrow REDUCTION path (j_q operand). emb_base is a running sum — m only
  // ever advances by 1, so the m*stride multiply is an add per step_m.
  logic [ADDR_WIDTH-1:0] j_nxt_q;
  logic                  mask_nxt_q;
  logic [31:0]           row_base_q;   // wide walk (j_nxt_q * stride)
  logic [31:0]           red_base_q;   // narrow REDUCTION (j_q * stride)
  logic                  mask_cur_q;
  logic [31:0]           emb_base_q;
  // incrementally-maintained index/weight addresses: recomputing
  // idx_base + m (+1/+2) every cycle put a 16-bit adder chain in front of
  // the BRAM address pins, which missed 6.67 ns
  logic [ADDR_WIDTH-1:0] idx_a_q;   // idx_base + m
  logic [ADDR_WIDTH-1:0] b_m_q;     // src_b + m (WGATHER weight stream)
  // registered end-of-list compares: the m+k vs idx_len adders were gating
  // the issue muxes in front of the BRAM address pins
  logic                  m_last_q;  // (m + 1) >= idx_len
  logic                  pf2_v_q;   // (m + 2) <  idx_len

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      a_q <= '0; b_q <= '0; dst_q <= '0; idx_q <= '0; stride_q <= ADDR_WIDTH'(1);
      len_q <= '0; ilen_q <= '0;
      grp_q <= '0; m_q <= '0; d_q <= '0; j_q <= '0; mask_q <= 1'b0;
      j_nxt_q <= '0; mask_nxt_q <= 1'b0;
      row_base_q <= '0; mask_cur_q <= 1'b0; emb_base_q <= '0;
    end else begin
      if (start) begin
        a_q <= src_a; b_q <= src_b; dst_q <= dst; idx_q <= idx_base;
        stride_q <= (stride == 0) ? ADDR_WIDTH'(1) : stride;
        len_q <= len; ilen_q <= idx_len;
        grp_q <= '0; m_q <= '0; d_q <= '0; chk_q <= '0; j_q <= '0; mask_q <= 1'b0;
        j_nxt_q <= '0; mask_nxt_q <= 1'b0;
        row_base_q <= '0; red_base_q <= '0; mask_cur_q <= 1'b0; emb_base_q <= '0;
        idx_a_q <= idx_base; b_m_q <= src_b;
        m_last_q <= (16'd1 >= idx_len);
        pf2_v_q  <= (32'd2 < 32'(idx_len));
      end else begin
        if (step_group) grp_q <= grp_q + 16'd1;
        if (step_idx) begin
          j_q    <= idx_rdata[ADDR_WIDTH-1:0];
          mask_q <= idx_rdata[31];
          d_q    <= '0;
          chk_q  <= '0;
        end
        if (idx_arrive) begin
          j_nxt_q    <= idx_rdata[ADDR_WIDTH-1:0];
          mask_nxt_q <= idx_rdata[31];
        end
        // separate multipliers per path: muxing the operands put an extra
        // LUT stage in front of an already-tight unpipelined DSP at 6.67 ns.
        // row_latch reads the PRE-update j_nxt_q / mask_nxt_q when idx_arrive
        // fires on the same edge (nonblocking).
        if (row_latch) begin
          row_base_q <= 32'(j_nxt_q) * 32'(stride_q);
          mask_cur_q <= mask_nxt_q;
        end
        if (red_mul) begin
          red_base_q <= 32'(j_q) * 32'(stride_q);
        end
        if (step_d) d_q <= d_q + ADDR_WIDTH'(1);
        if (step_chunk) chk_q <= chk_q + 16'd1;
        if (step_m) begin
          m_q        <= m_q + 16'd1;
          d_q        <= '0;
          chk_q      <= '0;
          emb_base_q <= emb_base_q + 32'(stride_q);
          idx_a_q    <= idx_a_q + ADDR_WIDTH'(1);
          b_m_q      <= b_m_q + ADDR_WIDTH'(1);
          m_last_q   <= (m_q + 16'd2) >= ilen_q;   // next m = m_q + 1
          pf2_v_q    <= (32'(m_q) + 32'd3) < 32'(ilen_q);
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
  // row_base_q / emb_base_q are registered (see above) — no live multiplies
  logic [ADDR_WIDTH-1:0] idx_addr, data_addr, b_addr, emb_waddr, out_waddr;

  assign idx_addr  = idx_a_q;
  assign data_addr = a_q + red_base_q[ADDR_WIDTH-1:0] + d_q;
  assign b_addr    = b_by_m ? b_m_q : (b_q + d_q);
  assign emb_waddr = dst_q + emb_base_q[ADDR_WIDTH-1:0] + d_q;
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
  assign sp_m_last    = m_last_q;
  assign sp_d_last    = (d_q + ADDR_WIDTH'(1)) >= stride_q;

  // ---------------- 8-wide sparse row walk ----------------
  // The dense lane pattern with the resolved row base substituted for src_a
  // and the chunk counter for the group counter. The chunk base (8*chk_q) is
  // a multiple of 8, so lane→bank rotations depend only on the row base and
  // stay constant for the whole row. The row base was registered at the
  // previous row boundary (row_latch), so chunk 0 still issues on the first
  // walk cycle — the old idx_rdata bypass multiply is gone.
  logic [ADDR_WIDTH-1:0] row_a, cbase;
  assign row_a = a_q + row_base_q[ADDR_WIDTH-1:0];
  assign cbase = ADDR_WIDTH'(chk_q) << 3;

  always_comb begin : sp8_lanes
    logic [ADDR_WIDTH-1:0] ia, ib, aaddr, baddr, waddr;
    for (int k = 0; k < PE_COUNT; k++) begin
      // row element whose A-address lands in bank k
      ia = cbase + ADDR_WIDTH'((3'(k) - row_a[2:0]) & 3'h7);
      aaddr = row_a + ia;
      sp8_offA[k] = off_of(aaddr);
      // B-stream element (SPARSE operand B[d]) whose address lands in bank k
      ib = cbase + ADDR_WIDTH'((3'(k) - b_q[2:0]) & 3'h7);
      baddr = b_q + ib;
      sp8_offB[k] = off_of(baddr);
      // EMBEDDING write target for lane k: dst + m*stride + (cbase + k)
      waddr = dst_q + emb_base_q[ADDR_WIDTH-1:0] + cbase + ADDR_WIDTH'(k);
      sp8_wbank[k] = bank_of(waddr);
      sp8_woff[k]  = off_of(waddr);
      sp8_lane_valid[k] = ({16'd0, cbase} + 32'(k)) < {16'd0, stride_q};
    end
  end
  assign sp8_arot = row_a[2:0];
  assign sp8_brot = b_q[2:0];
  assign sp8_chunk_last = ({16'd0, cbase} + 32'd8) >= {16'd0, stride_q};
  assign sp8_mask = mask_cur_q;

  // prefetch addresses issued in the row-end bubble (the one walk cycle with
  // idle issue ports): the WGATHER weight one row ahead, the index list TWO
  // rows ahead (so the j*stride multiply gets a full row of slack)
  logic [ADDR_WIDTH-1:0] idx_pf_addr, idx_pf2_addr, b_pf_addr;
  assign idx_pf_addr  = idx_a_q + ADDR_WIDTH'(1);
  assign idx_pf2_addr = idx_a_q + ADDR_WIDTH'(2);
  assign b_pf_addr    = b_m_q + ADDR_WIDTH'(1);
  assign sp_idx_pf_bank  = bank_of(idx_pf_addr);
  assign sp_idx_pf_off   = off_of(idx_pf_addr);
  assign sp_idx_pf2_bank = bank_of(idx_pf2_addr);
  assign sp_idx_pf2_off  = off_of(idx_pf2_addr);
  assign sp_idx_pf2_v    = pf2_v_q;
  assign sp_b_pf_bank    = bank_of(b_pf_addr);
  assign sp_b_pf_off     = off_of(b_pf_addr);

endmodule

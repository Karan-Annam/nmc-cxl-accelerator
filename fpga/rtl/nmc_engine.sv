// Dispatch engine: FSM, operand routing, accumulation, reduction tree,
// writeback. Drives the scatter_gather_engine for all address generation and
// the 8 configurable PEs for all arithmetic. Masking is handled entirely by
// the PE mask gate — a masked index entry contributes zero / is skipped with
// no engine special-casing anywhere.
//
// Dense: pipelined groups of 8 (issue group g+1 while executing group g) → ~8 elem/cyc.
// Sparse row modes (SPARSE/WGATHER/EMBEDDING): 8-wide row walk with the same
// issue/execute pipelining as dense (see scatter_gather_engine.sv for the
// bank-conflict argument). REDUCTION keeps the two-cycle-per-word schedule —
// its gathers are arbitrary addresses that could collide on a bank.
module nmc_engine
  import nmc_pkg::*;
(
  input  logic                  clk,
  input  logic                  rst_n,

  // command/config
  input  logic                  cmd_valid,
  input  nmc_cmd_t              cmd,
  input  logic                  cfg_valid,
  input  logic [CFG_WORD_W-1:0] cfg_word,
  output logic                  cmd_done,      // 1-cycle pulse
  output logic                  engine_busy,
  output logic                  engine_error,
  output logic [7:0]            engine_error_code,

  // SRAM access (owned while engine_busy && !sm_busy; muxed in nmc_top)
  output logic [BANK_AW-1:0]    eng_raddr_a [PE_COUNT],
  output logic [BANK_AW-1:0]    eng_raddr_b [PE_COUNT],
  input  logic [DATA_WIDTH-1:0] rdata_a     [PE_COUNT],
  input  logic [DATA_WIDTH-1:0] rdata_b     [PE_COUNT],
  output logic                  eng_we      [PE_COUNT],
  output logic [BANK_AW-1:0]    eng_woff    [PE_COUNT],
  output logic [DATA_WIDTH-1:0] eng_wdata   [PE_COUNT],

  // softmax unit handshake
  output logic                  sm_start,
  output logic [ADDR_WIDTH-1:0] sm_src,
  output logic [ADDR_WIDTH-1:0] sm_dst,
  output logic [15:0]           sm_len,
  input  logic                  sm_busy,
  input  logic                  sm_done,

  // perf
  output logic [3:0]            ops_inc
);

  // ------------------------------------------------------------------
  // config regfile + PEs
  // ------------------------------------------------------------------
  logic [PE_CFG_W-1:0] pe_cfg [PE_COUNT];

  config_regfile u_cfg (
    .clk(clk), .rst_n(rst_n),
    .cfg_valid(cfg_valid), .cfg_word(cfg_word), .pe_cfg(pe_cfg)
  );

  logic [DATA_WIDTH-1:0] pe_a   [PE_COUNT];
  logic [DATA_WIDTH-1:0] pe_b   [PE_COUNT];
  logic                  pe_msk [PE_COUNT];
  logic                  pe_en  [PE_COUNT];
  logic                  acc_rst;
  logic [DATA_WIDTH-1:0] pe_res [PE_COUNT];     // 1-cycle-latency ALU result
  logic [DATA_WIDTH-1:0] pe_acc [PE_COUNT];
  logic [DATA_WIDTH-1:0] pe_acc_eff [PE_COUNT];  // acc + in-flight accumulate
  logic [DATA_WIDTH-1:0] pe_mulres  [PE_COUNT];  // 2-cycle-latency a*b product

  for (genvar p = 0; p < PE_COUNT; p++) begin : g_pe
    configurable_pe u_pe (
      .clk(clk), .pe_cfg(pe_cfg[p]),
      .operand_a(pe_a[p]), .operand_b(pe_b[p]),
      .mask_bit(pe_msk[p]), .en(pe_en[p]), .acc_rst(acc_rst),
      .result(pe_res[p]), .acc_out(pe_acc[p]),
      .acc_out_eff(pe_acc_eff[p]), .mul_res(pe_mulres[p])
    );
  end

  logic [3:0] pe0_op;
  assign pe0_op = pe_cfg[0][3:0];
  logic acc_mode;   // dense reduction semantics (dot product / sum / max fold)
  assign acc_mode = (pe0_op == OP_MACC) || (pe0_op == OP_SACC) ||
                    ((pe0_op == OP_MAX) && (pe_cfg[0][5:4] == 2'b00));
  logic tree_is_max;
  assign tree_is_max = (pe0_op == OP_MAX);

  // ------------------------------------------------------------------
  // scatter/gather engine
  // ------------------------------------------------------------------
  nmc_cmd_t c_q;
  logic sg_start, step_group, step_idx, step_d, step_m;
  logic b_by_m;
  assign b_by_m = (c_q.cmd_op == CMD_WGATHER);

  logic [BANK_AW-1:0]  dn_offA [PE_COUNT], dn_offB [PE_COUNT], dn_woff [PE_COUNT];
  logic [2:0]          dn_abank [PE_COUNT], dn_bbank [PE_COUNT], dn_wbank [PE_COUNT];
  logic [PE_COUNT-1:0] dn_lane_valid;
  logic                dn_group_last;
  logic [2:0]          sp_idx_bank, sp_data_bank, sp_b_bank, sp_emb_wbank, sp_out_wbank;
  logic [BANK_AW-1:0]  sp_idx_off, sp_data_off, sp_b_off, sp_emb_woff, sp_out_woff;
  logic                sp_mask_bit, sp_m_last, sp_d_last;
  logic [15:0]         sp_m;
  logic [ADDR_WIDTH-1:0] sp_d;
  logic [DATA_WIDTH-1:0] idx_word;
  logic                step_chunk, idx_arrive, row_latch, red_mul;
  logic [BANK_AW-1:0]  sp8_offA [PE_COUNT], sp8_offB [PE_COUNT], sp8_woff [PE_COUNT];
  logic [2:0]          sp8_arot, sp8_brot;
  logic [2:0]          sp8_wbank [PE_COUNT];
  logic [PE_COUNT-1:0] sp8_lane_valid;
  logic                sp8_chunk_last, sp8_mask;
  logic [2:0]          sp_idx_pf_bank, sp_idx_pf2_bank, sp_b_pf_bank;
  logic [BANK_AW-1:0]  sp_idx_pf_off, sp_idx_pf2_off, sp_b_pf_off;
  logic                sp_idx_pf2_v;

  scatter_gather_engine u_sg (
    .clk(clk), .rst_n(rst_n),
    .start(sg_start),
    .src_a(c_q.src_a), .src_b(c_q.src_b), .dst(c_q.dst),
    .idx_base(c_q.idx_base), .stride(c_q.stride),
    .len(c_q.len), .idx_len(c_q.idx_len),
    .step_group(step_group), .step_idx(step_idx),
    .step_d(step_d), .step_chunk(step_chunk), .step_m(step_m),
    .idx_rdata(idx_word),
    .dn_offA(dn_offA), .dn_offB(dn_offB),
    .dn_abank(dn_abank), .dn_bbank(dn_bbank),
    .dn_wbank(dn_wbank), .dn_woff(dn_woff),
    .dn_lane_valid(dn_lane_valid), .dn_group_last(dn_group_last),
    .sp_idx_bank(sp_idx_bank), .sp_idx_off(sp_idx_off),
    .sp_data_bank(sp_data_bank), .sp_data_off(sp_data_off),
    .sp_b_bank(sp_b_bank), .sp_b_off(sp_b_off), .b_by_m(b_by_m),
    .sp_emb_wbank(sp_emb_wbank), .sp_emb_woff(sp_emb_woff),
    .sp_out_wbank(sp_out_wbank), .sp_out_woff(sp_out_woff),
    .sp_mask_bit(sp_mask_bit), .sp_m(sp_m), .sp_d(sp_d),
    .sp_m_last(sp_m_last), .sp_d_last(sp_d_last),
    .idx_arrive(idx_arrive), .row_latch(row_latch), .red_mul(red_mul),
    .sp8_offA(sp8_offA), .sp8_offB(sp8_offB),
    .sp8_arot(sp8_arot), .sp8_brot(sp8_brot),
    .sp8_wbank(sp8_wbank), .sp8_woff(sp8_woff),
    .sp8_lane_valid(sp8_lane_valid), .sp8_chunk_last(sp8_chunk_last),
    .sp8_mask(sp8_mask),
    .sp_idx_pf_bank(sp_idx_pf_bank), .sp_idx_pf_off(sp_idx_pf_off),
    .sp_idx_pf2_bank(sp_idx_pf2_bank), .sp_idx_pf2_off(sp_idx_pf2_off),
    .sp_idx_pf2_v(sp_idx_pf2_v),
    .sp_b_pf_bank(sp_b_pf_bank), .sp_b_pf_off(sp_b_pf_off)
  );

  // ------------------------------------------------------------------
  // FSM
  // ------------------------------------------------------------------
  // 2-cycle bank reads (outpost + BRAM register): dense and the wide walk
  // keep TWO requests in flight (issue c+2 while executing c), the narrow
  // path gets *_GAP wait states, and priming grows one stage per read.
  typedef enum logic [4:0] {
    E_IDLE, E_CFG, E_DISPATCH,
    E_DN_PRIME, E_DN_PRIME2, E_DN_RUN, E_DN_DRAIN,
    E_SP_IDX_ISSUE, E_SP_IDX_GAP, E_SP_IDX_WAIT, E_SP_MUL,
    E_SP_D_ISSUE, E_SP_D_GAP, E_SP_D_WAIT,
    E_SP8_PRIME, E_SP8_PRIME2, E_SP8_PRIME3, E_SP8_NEXT, E_SP8_RUN, E_SP8_DRAIN,
    E_TREE_CAP, E_TREE0, E_TREE1, E_TREE2, E_TREE_WB,
    E_WG_FLUSH, E_WG_WB,
    E_SM_RUN,
    E_DONE
  } estate_e;
  estate_e st_q;

  assign engine_busy = (st_q != E_IDLE);

  // dense pipeline registers: p = group issued last cycle (data in flight),
  // x = group issued two cycles ago (data live this cycle — exec stage)
  logic [PE_COUNT-1:0] lv_p, lv_x;
  logic [BANK_AW-1:0]  woff_p [PE_COUNT], woff_x [PE_COUNT];
  logic [2:0]          wbank_p [PE_COUNT], wbank_x [PE_COUNT];
  logic                glast_p, glast_x;

  // sparse pipeline registers (narrow / REDUCTION path)
  logic [2:0]  data_bank_x, b_bank_x, idx_bank_x;
  logic [DATA_WIDTH-1:0] w_q;   // WGATHER row weight B[m]

  // sparse wide-walk chunk pipeline: p = in flight, x = exec (2-cycle reads)
  logic [PE_COUNT-1:0] lv8_p, lv8_x;
  logic [2:0]          ra_x, rb_x;      // lane→bank rotations (constant per row)
  logic                clast_p, clast_x;
  logic [2:0]          chunk_p, chunk_x; // accvec chunk index (d = 8*chunk + lane)
  logic [2:0]          ewb_p [PE_COUNT], ewb_x [PE_COUNT]; // EMBEDDING targets
  logic [BANK_AW-1:0]  ewo_p [PE_COUNT], ewo_x [PE_COUNT];

  // WGATHER accumulator vector + accumulate pipe: lane-valids/chunk trail the
  // exec stage by 2 cycles to meet the PE's pipelined product (mul_res)
  logic [DATA_WIDTH-1:0] accvec_q [ACCVEC_DEPTH];
  logic [ADDR_WIDTH-1:0] wg_d_q;   // output-drain CHUNK counter (8 words/cycle)
  logic [PE_COUNT-1:0]   wg_lv_y, wg_lv_z, wg_lv_w;
  logic [2:0]            wg_ch_y, wg_ch_z, wg_ch_w;

  // dense delayed writeback pipe: every dense write fires TWO cycles after
  // exec with register-sourced data (BRAM-read → ALU → BRAM-write in one
  // cycle missed 10 ns; ALU-result → write mux → BRAM in one cycle missed
  // 6.67). ADD-family data is the ALU result captured at stage z; OP_MUL data
  // comes from the PE multiply pipe (mul_res), same cycle.
  // max_fanout on the write-valid pipes: each bit reaches every bank's
  // write-port mux — replication lets the placer keep a copy near each
  // BRAM cluster instead of routing one FF across the die
  logic                wb_late, wb_late_q;
  logic [PE_COUNT-1:0] dnw_v_y;
  (* max_fanout = 4 *) logic [PE_COUNT-1:0] dnw_v_z, dnw_v_w;
  logic [2:0]          dnw_bank_y [PE_COUNT], dnw_bank_z [PE_COUNT], dnw_bank_w [PE_COUNT];
  logic [BANK_AW-1:0]  dnw_off_y  [PE_COUNT], dnw_off_z  [PE_COUNT], dnw_off_w  [PE_COUNT];
  logic [DATA_WIDTH-1:0] dnw_data_z [PE_COUNT];
  assign wb_late = (pe0_op == OP_MUL) && !acc_mode;
  // registered config decode: keeps the cfg regfile out of the write-enable
  // cone (config is stable many cycles before any dense write fires).
  // max_fanout: this net reaches every bank's write mux — let it replicate
  // instead of routing one FF across all BRAM columns.
  (* max_fanout = 16 *)
  always_ff @(posedge clk) wb_late_q <= wb_late;

  // EMBEDDING delayed writeback pipe (2 cycles, registered data)
  logic [PE_COUNT-1:0] emw_v_y;
  (* max_fanout = 4 *) logic [PE_COUNT-1:0] emw_v_z;
  logic [2:0]          emw_bank_y [PE_COUNT], emw_bank_z [PE_COUNT];
  logic [BANK_AW-1:0]  emw_off_y  [PE_COUNT], emw_off_z  [PE_COUNT];
  logic [DATA_WIDTH-1:0] emw_data_z [PE_COUNT];

  // WGATHER drain pipe (1 cycle: address/valid/accvec-mux registered)
  (* max_fanout = 4 *) logic [PE_COUNT-1:0] wgw_v_y;
  logic [2:0]            wgw_bank_y [PE_COUNT];
  logic [BANK_AW-1:0]    wgw_off_y  [PE_COUNT];
  logic [DATA_WIDTH-1:0] wgw_data_y [PE_COUNT];

  // 2-cycle drain counter (dense writeback tail / WGATHER accumulate flush)
  logic [1:0] drain_q;

  // reduction tree registers (blocking path: dense acc-fold / REDUCTION)
  logic [DATA_WIDTH-1:0] tr4 [4], tr2 [2], tr1;

  // pipelined per-row reduction tree (wide SPARSE): row m's fold runs while
  // row m+1 gathers. Each stage carries a valid bit + the writeback target
  // captured at row end (before step_m advanced the index). pend0 delays the
  // snapshot one cycle past the boundary so the final chunk's product — still
  // in the PE multiply pipe at the boundary edge — is included (stage 0 reads
  // acc_out_eff = acc + in-flight product).
  logic                  pend0_v, pend0b_v, pend_v, pend2_v, p4_v, p2_v, p1_v;
  logic [2:0]            pend0_bank, pend0b_bank, pend_bank, pend2_bank,
                         p4_bank, p2_bank, p1_bank;
  logic [BANK_AW-1:0]    pend0_off, pend0b_off, pend_off, pend2_off,
                         p4_off, p2_off, p1_off;
  logic [DATA_WIDTH-1:0] ptr4 [4], ptr2 [2], ptr1;
  // acc_out_eff captured once per cycle: eff (acc + in-flight product) into
  // a tree node was two chained 32-bit adds — over 6.67 ns from the DSP
  logic [DATA_WIDTH-1:0] eff_q [PE_COUNT];

  function automatic logic [DATA_WIDTH-1:0] tnode(
      input logic is_max,
      input logic [DATA_WIDTH-1:0] x, input logic [DATA_WIDTH-1:0] y);
    if (is_max) return ($signed(x) > $signed(y)) ? x : y;
    else        return x + y;
  endfunction

  // row result for SPARSE per-index output
  logic error_q;
  logic [7:0] error_code_q;
  assign engine_error      = error_q;
  assign engine_error_code = error_code_q;

  assign idx_word = rdata_a[idx_bank_x];

  // ------------------------------------------------------------------
  // combinational: SRAM drive + PE routing
  // ------------------------------------------------------------------
  // SG step strobes must be combinational: SRAM read-port outputs are only valid
  // for the single cycle after their address was driven, so the SG counters have to
  // advance on the same clock edge that ends the state consuming the data.
  assign step_idx = (st_q == E_SP_IDX_WAIT);
  assign step_d   = (st_q == E_SP_D_WAIT) && !sp_d_last;
  assign step_m   = ((st_q == E_SP_D_WAIT) && sp_d_last && !sp_m_last) ||
                    ((st_q == E_SP8_RUN) && clast_x && !sp_m_last);
  // Wide-walk row-base pipeline (see scatter_gather_engine.sv): the NEXT row's
  // index word lands in j_nxt_q while the current row walks (idx_arrive), and
  // the j*stride multiply runs at the row boundary (row_latch). Reads are
  // 2-cycle now: idx[0]/idx[1] issued in IDX_ISSUE/PRIME arrive in
  // PRIME2/PRIME3. At m == 0 the E_SP8_NEXT arrival is gated off: idx[1]
  // already arrived in E_SP8_PRIME3 and nothing was issued two cycles before
  // (j_nxt_q must not be clobbered).
  assign idx_arrive = (st_q == E_SP8_PRIME2) || (st_q == E_SP8_PRIME3) ||
                      ((st_q == E_SP8_NEXT) && (sp_m != 16'd0));
  assign row_latch  = (st_q == E_SP8_PRIME3) ||
                      ((st_q == E_SP8_RUN) && clast_x && !sp_m_last);
  assign red_mul    = (st_q == E_SP_MUL);

  // PE accumulators clear between wide-SPARSE rows one cycle later than the
  // old design (acc_rst_sp_q trails E_SP8_NEXT): the clear lands on the same
  // edge the delayed snapshot reads acc_out_eff (nonblocking: pre-clear
  // values) and simultaneously discards the final chunk's redundant
  // accumulate — that product is already inside the snapshot.
  logic acc_rst_q, acc_rst_sp_q, acc_rst_sp2_q;
  assign acc_rst = acc_rst_q || acc_rst_sp2_q;

  always_comb begin
    // defaults (write-port signals are driven by the flat OR block below,
    // never from the state case — stacked per-writer overrides synthesized
    // as a ~24-LUT serial priority chain into the bank WE/DI pins)
    for (int k = 0; k < PE_COUNT; k++) begin
      eng_raddr_a[k] = '0;
      eng_raddr_b[k] = '0;
      pe_a[k]  = '0;
      pe_b[k]  = '0;
      pe_msk[k] = 1'b1;
      pe_en[k] = 1'b0;
    end
    step_group = 1'b0;
    step_chunk = 1'b0;
    ops_inc    = 4'd0;

    unique case (st_q)
      // ---------------- dense ----------------
      E_DN_PRIME: begin
        for (int k = 0; k < PE_COUNT; k++) begin
          eng_raddr_a[k] = dn_offA[k];
          eng_raddr_b[k] = dn_offB[k];
        end
        step_group = 1'b1;
      end
      E_DN_PRIME2: begin
        // second group issued while the first is still in the 2-cycle read
        for (int k = 0; k < PE_COUNT; k++) begin
          eng_raddr_a[k] = dn_offA[k];
          eng_raddr_b[k] = dn_offB[k];
        end
        step_group = !glast_p;
      end
      E_DN_RUN: begin
        // issue group grp_q (if any left; two groups stay in flight)
        for (int k = 0; k < PE_COUNT; k++) begin
          eng_raddr_a[k] = dn_offA[k];
          eng_raddr_b[k] = dn_offB[k];
        end
        step_group = !glast_p && !glast_x;
        // execute the group issued two cycles ago. ALL dense writes fire
        // from the registered dnw_* pipe below.
        for (int p = 0; p < PE_COUNT; p++) begin
          pe_a[p]  = rdata_a[dn_abank[p]];
          pe_b[p]  = rdata_b[dn_bbank[p]];
          pe_en[p] = lv_x[p];
          if (lv_x[p]) ops_inc = ops_inc + 4'd1;
        end
      end

      // ---------------- sparse ----------------
      E_SP_IDX_ISSUE: begin
        eng_raddr_a[sp_idx_bank] = sp_idx_off;
      end
      E_SP8_PRIME: begin
        // idx[0] still in flight (2-cycle read); issue idx[1] behind it
        if (!sp_m_last) eng_raddr_a[sp_idx_pf_bank] = sp_idx_pf_off;
      end
      E_SP8_PRIME2: begin
        // idx[0] arrives this cycle (idx_arrive). Issue the WGATHER weight
        // B[0] so it lands in E_SP8_NEXT two cycles from now.
        if (b_by_m) eng_raddr_b[sp_b_bank] = sp_b_off;
      end
      // E_SP8_PRIME3: idx[1] arrives; row 0's base multiply latches this
      // edge (row_latch reads the pre-update j_nxt_q). No port activity.
      E_SP_D_ISSUE: begin
        eng_raddr_a[sp_data_bank] = sp_data_off;
        if (!b_by_m) eng_raddr_b[sp_b_bank] = sp_b_off;  // SPARSE operand B[d]
      end
      E_SP_D_WAIT: begin
        // (REDUCTION only) gathered word arrives on data_bank_x port A
        pe_a[data_bank_x]   = rdata_a[data_bank_x];
        // REDUCTION folds the gathered value itself: present it on operand_b too,
        // so MAX-aggregation (a = own acc, result = max(acc, b)) sees it. SACC uses
        // operand_a and ignores b.
        pe_b[data_bank_x]   = rdata_a[data_bank_x];
        pe_msk[data_bank_x] = sp_mask_bit;
        pe_en[data_bank_x]  = 1'b1;
        ops_inc = 4'd1;
      end

      // ---------------- sparse 8-wide row walk ----------------
      E_SP8_NEXT: begin
        // Chunk-0 addresses come from the registered row base (latched at the
        // previous row boundary / E_SP8_PRIME2). The NEXT row's index word is
        // arriving right now for m >= 1 (idx_arrive parks it in j_nxt_q).
        for (int k = 0; k < PE_COUNT; k++) begin
          eng_raddr_a[k] = sp8_offA[k];
          if (c_q.cmd_op == CMD_SPARSE) eng_raddr_b[k] = sp8_offB[k];
        end
        step_chunk = 1'b1;
      end
      E_SP8_RUN: begin
        if (!clast_p && !clast_x) begin
          // issue chunk chk_q while two earlier chunks are in flight/exec
          for (int k = 0; k < PE_COUNT; k++) begin
            eng_raddr_a[k] = sp8_offA[k];
            if (c_q.cmd_op == CMD_SPARSE) eng_raddr_b[k] = sp8_offB[k];
          end
          step_chunk = 1'b1;
        end else if (clast_p) begin
          // first idle-issue cycle (last chunk is in flight) — prefetch the
          // index TWO rows ahead (arrives exactly at the next E_SP8_NEXT with
          // 2-cycle reads) and the next row's weight for WGATHER
          if (sp_idx_pf2_v) eng_raddr_a[sp_idx_pf2_bank] = sp_idx_pf2_off;
          if (!sp_m_last && b_by_m) eng_raddr_b[sp_b_pf_bank] = sp_b_pf_off;
        end
        // execute the chunk issued two cycles ago (EMBEDDING writes fire from
        // the registered emw_* pipe below)
        for (int p = 0; p < PE_COUNT; p++) begin
          pe_a[p]   = rdata_a[(ra_x + 3'(p)) & 3'h7];
          pe_b[p]   = (c_q.cmd_op == CMD_SPARSE) ? rdata_b[(rb_x + 3'(p)) & 3'h7]
                                                 : w_q;
          pe_msk[p] = sp8_mask;
          if (lv8_x[p]) begin
            if (c_q.cmd_op == CMD_SPARSE) pe_en[p] = 1'b1;
            ops_inc = ops_inc + 4'd1;
          end
        end
      end

      // E_TREE_WB / E_WG_WB drive no writes here — see the flat write block
      default: ;
    endcase
  end

  // ------------------------------------------------------------------
  // bank write-port drive: flat one-hot OR across all writeback sources.
  // The sources are mutually exclusive by construction — E_TREE_WB
  // (dense-fold / REDUCTION scalar), the pipelined SPARSE row tree (p1),
  // the dense delayed pipe (stage y for ADD-family via result_q, stage z
  // for OP_MUL via mul_res — wb_late_q selects exactly one per command),
  // the EMBEDDING pipe, and the WGATHER drain pipe never coexist, and each
  // vector source hits 8 distinct banks. OR-accumulation keeps the cone a
  // balanced tree; per-writer overrides synthesized as a serial priority
  // chain ~24 LUTs deep into the bank WE/DI pins.
  // ------------------------------------------------------------------
  always_comb begin
    for (int k = 0; k < PE_COUNT; k++) begin
      eng_we[k]    = 1'b0;
      eng_woff[k]  = '0;
      eng_wdata[k] = '0;

      // dense-reduction / REDUCTION scalar → dst[0]
      if ((st_q == E_TREE_WB) && (bank_of(c_q.dst) == 3'(k))) begin
        eng_we[k]     = 1'b1;
        eng_woff[k]  |= off_of(c_q.dst);
        eng_wdata[k] |= tr1;
      end
      // pipelined-tree writeback (wide SPARSE): fires while the next row
      // walks — the walk performs no other writes
      if (p1_v && (c_q.cmd_op == CMD_SPARSE) && (p1_bank == 3'(k))) begin
        eng_we[k]     = 1'b1;
        eng_woff[k]  |= p1_off;
        eng_wdata[k] |= ptr1;
      end
      for (int p = 0; p < PE_COUNT; p++) begin
        // dense ADD-family (2 cycles behind exec; ALU result captured)
        if (!wb_late_q && dnw_v_z[p] && (dnw_bank_z[p] == 3'(k))) begin
          eng_we[k]     = 1'b1;
          eng_woff[k]  |= dnw_off_z[p];
          eng_wdata[k] |= dnw_data_z[p];
        end
        // dense OP_MUL (3 cycles behind exec — the 3-stage multiply pipe)
        if (wb_late_q && dnw_v_w[p] && (dnw_bank_w[p] == 3'(k))) begin
          eng_we[k]     = 1'b1;
          eng_woff[k]  |= dnw_off_w[p];
          eng_wdata[k] |= pe_mulres[p];
        end
        // EMBEDDING (2 cycles behind exec; last chunk lands in E_WG_FLUSH)
        if (emw_v_z[p] && (emw_bank_z[p] == 3'(k))) begin
          eng_we[k]     = 1'b1;
          eng_woff[k]  |= emw_off_z[p];
          eng_wdata[k] |= emw_data_z[p];
        end
        // WGATHER drain (1 cycle behind E_WG_WB; last stage lands in E_DONE)
        if (wgw_v_y[p] && (wgw_bank_y[p] == 3'(k))) begin
          eng_we[k]     = 1'b1;
          eng_woff[k]  |= wgw_off_y[p];
          eng_wdata[k] |= wgw_data_y[p];
        end
      end
    end
  end

  // WGATHER drain addresses (precomputed so the case branch stays latch-free)
  logic [ADDR_WIDTH-1:0] wg_wa [PE_COUNT];
  logic [PE_COUNT-1:0]   wg_wv;
  always_comb begin
    for (int p = 0; p < PE_COUNT; p++) begin
      wg_wa[p] = c_q.dst + (ADDR_WIDTH'(wg_d_q) << 3) + ADDR_WIDTH'(p);
      wg_wv[p] = ((32'(wg_d_q) << 3) + 32'(p)) < 32'(c_q.stride);
    end
  end

  // ------------------------------------------------------------------
  // sequential
  // ------------------------------------------------------------------
  logic op_is_sparse;
  assign op_is_sparse = (c_q.cmd_op == CMD_SPARSE) || (c_q.cmd_op == CMD_WGATHER) ||
                        (c_q.cmd_op == CMD_REDUCTION) || (c_q.cmd_op == CMD_EMBEDDING);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st_q <= E_IDLE;
      c_q <= '0;
      cmd_done <= 1'b0;
      sg_start <= 1'b0;
      acc_rst_q <= 1'b0;
      sm_start <= 1'b0; sm_src <= '0; sm_dst <= '0; sm_len <= '0;
      error_q <= 1'b0; error_code_q <= ERR_NONE;
      lv_p <= '0; glast_p <= 1'b0; lv_x <= '0; glast_x <= 1'b0;
      data_bank_x <= '0; b_bank_x <= '0; idx_bank_x <= '0;
      w_q <= '0; wg_d_q <= '0;
      lv8_p <= '0; clast_p <= 1'b0; chunk_p <= '0;
      lv8_x <= '0; ra_x <= '0; rb_x <= '0; clast_x <= 1'b0; chunk_x <= '0;
      tr1 <= '0;
      for (int i = 0; i < 4; i++) tr4[i] <= '0;
      for (int i = 0; i < 2; i++) tr2[i] <= '0;
      pend0_v <= 1'b0; pend0b_v <= 1'b0; pend_v <= 1'b0; pend2_v <= 1'b0;
      p4_v <= 1'b0; p2_v <= 1'b0; p1_v <= 1'b0;
      pend0_bank <= '0; pend0b_bank <= '0; pend_bank <= '0; pend2_bank <= '0;
      p4_bank <= '0; p2_bank <= '0; p1_bank <= '0;
      pend0_off <= '0; pend0b_off <= '0; pend_off <= '0; pend2_off <= '0;
      p4_off <= '0; p2_off <= '0; p1_off <= '0;
      for (int i = 0; i < PE_COUNT; i++) eff_q[i] <= '0;
      ptr1 <= '0;
      for (int i = 0; i < 4; i++) ptr4[i] <= '0;
      for (int i = 0; i < 2; i++) ptr2[i] <= '0;
      acc_rst_sp_q <= 1'b0; acc_rst_sp2_q <= 1'b0;
      wg_lv_y <= '0; wg_lv_z <= '0; wg_lv_w <= '0;
      wg_ch_y <= '0; wg_ch_z <= '0; wg_ch_w <= '0;
      dnw_v_y <= '0; dnw_v_z <= '0; dnw_v_w <= '0;
      emw_v_y <= '0; emw_v_z <= '0; wgw_v_y <= '0;
      drain_q <= '0;
      for (int i = 0; i < PE_COUNT; i++) begin
        woff_p[i] <= '0; wbank_p[i] <= '0;
        woff_x[i] <= '0; wbank_x[i] <= '0;
        ewb_p[i] <= '0; ewo_p[i] <= '0;
        ewb_x[i] <= '0; ewo_x[i] <= '0;
        dnw_bank_y[i] <= '0; dnw_bank_z[i] <= '0; dnw_bank_w[i] <= '0;
        dnw_off_y[i] <= '0; dnw_off_z[i] <= '0; dnw_off_w[i] <= '0;
        dnw_data_z[i] <= '0;
        emw_bank_y[i] <= '0; emw_off_y[i] <= '0;
        emw_bank_z[i] <= '0; emw_off_z[i] <= '0; emw_data_z[i] <= '0;
        wgw_bank_y[i] <= '0; wgw_off_y[i] <= '0; wgw_data_y[i] <= '0;
      end
      for (int i = 0; i < ACCVEC_DEPTH; i++) accvec_q[i] <= '0;
    end else begin
      cmd_done <= 1'b0;
      sg_start <= 1'b0;
      acc_rst_q <= 1'b0;
      sm_start <= 1'b0;

      // ---- delayed sparse acc clear (see acc_rst comment above; two stages
      // to match the 3-stage multiply pipe) ---------------------------------
      acc_rst_sp_q  <= (st_q == E_SP8_NEXT) && (c_q.cmd_op == CMD_SPARSE);
      acc_rst_sp2_q <= acc_rst_sp_q;

      // ---- continuous acc_out_eff capture (see eff_q declaration) --------
      for (int p = 0; p < PE_COUNT; p++) eff_q[p] <= pe_acc_eff[p];

      // ---- pipelined per-row reduction tree (wide SPARSE) ----------------
      // Advances every cycle. The boundary snapshot walks pend0 → pend0b →
      // pend (the cycle acc_out_eff holds acc + the final chunk's product,
      // which eff_q captures on the same edge acc_rst_sp2_q clears the
      // accumulators — nonblocking: pre-clear values) → pend2 (stage 0 folds
      // the CAPTURED eff_q).
      if (pend2_v)
        for (int i = 0; i < 4; i++)
          ptr4[i] <= tnode(tree_is_max, eff_q[2*i], eff_q[2*i+1]);
      p4_v <= pend2_v; p4_bank <= pend2_bank; p4_off <= pend2_off;
      if (p4_v)
        for (int i = 0; i < 2; i++)
          ptr2[i] <= tnode(tree_is_max, ptr4[2*i], ptr4[2*i+1]);
      p2_v <= p4_v; p2_bank <= p4_bank; p2_off <= p4_off;
      if (p2_v) ptr1 <= tnode(tree_is_max, ptr2[0], ptr2[1]);
      p1_v <= p2_v; p1_bank <= p2_bank; p1_off <= p2_off;
      pend2_v  <= pend_v;   pend2_bank  <= pend_bank;   pend2_off  <= pend_off;
      pend_v   <= pend0b_v; pend_bank   <= pend0b_bank; pend_off   <= pend0b_off;
      pend0b_v <= pend0_v;  pend0b_bank <= pend0_bank;  pend0b_off <= pend0_off;
      pend0_v <= 1'b0;   // set below at a wide-SPARSE row boundary

      // ---- WGATHER accumulate pipe: trail exec by 3 cycles to meet mul_res
      // (the 3-stage multiply pipe). The RMW is out-of-state so in-flight
      // products land during E_SP8_NEXT / E_WG_FLUSH after the walk moves on.
      wg_lv_y <= ((st_q == E_SP8_RUN) && (c_q.cmd_op == CMD_WGATHER)) ? lv8_x : '0;
      wg_ch_y <= chunk_x;
      wg_lv_z <= wg_lv_y;
      wg_ch_z <= wg_ch_y;
      wg_lv_w <= wg_lv_z;
      wg_ch_w <= wg_ch_z;
      for (int p = 0; p < PE_COUNT; p++) begin
        if (wg_lv_w[p])
          accvec_q[{wg_ch_w, 3'(p)}] <= accvec_q[{wg_ch_w, 3'(p)}]
                                        + pe_mulres[p];
      end

      // ---- dense delayed writeback pipe (all non-acc dense ops) -----------
      // stage z captures the ALU result (pe_res is valid during the y cycle)
      dnw_v_y <= ((st_q == E_DN_RUN) && !acc_mode) ? lv_x : '0;
      dnw_v_z <= dnw_v_y;
      dnw_v_w <= dnw_v_z;
      for (int p = 0; p < PE_COUNT; p++) begin
        dnw_bank_y[p] <= wbank_x[p];
        dnw_off_y[p]  <= woff_x[p];
        dnw_bank_z[p] <= dnw_bank_y[p];
        dnw_off_z[p]  <= dnw_off_y[p];
        dnw_data_z[p] <= pe_res[p];
        dnw_bank_w[p] <= dnw_bank_z[p];
        dnw_off_w[p]  <= dnw_off_z[p];
      end

      // ---- EMBEDDING delayed writeback pipe --------------------------------
      emw_v_y <= ((st_q == E_SP8_RUN) && (c_q.cmd_op == CMD_EMBEDDING)) ? lv8_x : '0;
      emw_v_z <= emw_v_y;
      for (int p = 0; p < PE_COUNT; p++) begin
        emw_bank_y[p] <= ewb_x[p];
        emw_off_y[p]  <= ewo_x[p];
        emw_bank_z[p] <= emw_bank_y[p];
        emw_off_z[p]  <= emw_off_y[p];
        emw_data_z[p] <= pe_res[p];
      end

      // ---- WGATHER drain pipe ----------------------------------------------
      wgw_v_y <= (st_q == E_WG_WB) ? wg_wv : '0;
      for (int p = 0; p < PE_COUNT; p++) begin
        wgw_bank_y[p] <= bank_of(wg_wa[p]);
        wgw_off_y[p]  <= off_of(wg_wa[p]);
        wgw_data_y[p] <= accvec_q[{wg_d_q[2:0], 3'(p)}];
      end

      unique case (st_q)
        E_IDLE: begin
          if (cmd_valid) begin
            c_q <= cmd;
            sg_start <= 1'b1;
            error_q <= 1'b0;
            error_code_q <= ERR_NONE;
            st_q <= E_CFG;
          end
        end

        E_CFG: begin
          acc_rst_q <= 1'b1;
          for (int i = 0; i < ACCVEC_DEPTH; i++) accvec_q[i] <= '0;
          wg_d_q <= '0;
          st_q <= E_DISPATCH;
        end

        E_DISPATCH: begin
          // validate + dispatch
          if (c_q.cmd_op > 4'(CMD_EMBEDDING)) begin
            error_q <= 1'b1; error_code_q <= ERR_BAD_OP; st_q <= E_DONE;
          end else if (c_q.cmd_op == CMD_WGATHER &&
                       32'(c_q.stride) > ACCVEC_DEPTH) begin
            error_q <= 1'b1; error_code_q <= ERR_STRIDE_CAP; st_q <= E_DONE;
          end else if (c_q.cmd_op == CMD_DENSE) begin
            if (c_q.len == 0) st_q <= E_DONE;
            else st_q <= E_DN_PRIME;
          end else if (c_q.cmd_op == CMD_SOFTMAX) begin
            sm_start <= 1'b1;
            sm_src <= c_q.src_a; sm_dst <= c_q.dst; sm_len <= c_q.len;
            st_q <= E_SM_RUN;
          end else begin
            if (c_q.idx_len == 0) st_q <= E_DONE;
            else st_q <= E_SP_IDX_ISSUE;
          end
        end

        // ---------------- dense ----------------
        E_DN_PRIME: begin
          lv_p    <= dn_lane_valid;
          glast_p <= dn_group_last;
          for (int p = 0; p < PE_COUNT; p++) begin
            woff_p[p]  <= dn_woff[p];
            wbank_p[p] <= dn_wbank[p];
          end
          st_q <= E_DN_PRIME2;
        end
        E_DN_PRIME2: begin
          // group 0 advances to the exec stage; group 1 (if any) enters the
          // in-flight stage — two 2-cycle reads now outstanding
          lv_x    <= lv_p;
          glast_x <= glast_p;
          for (int p = 0; p < PE_COUNT; p++) begin
            woff_x[p]  <= woff_p[p];
            wbank_x[p] <= wbank_p[p];
          end
          if (!glast_p) begin
            lv_p    <= dn_lane_valid;
            glast_p <= dn_group_last;
            for (int p = 0; p < PE_COUNT; p++) begin
              woff_p[p]  <= dn_woff[p];
              wbank_p[p] <= dn_wbank[p];
            end
          end else begin
            lv_p    <= '0;
            glast_p <= 1'b0;
          end
          st_q <= E_DN_RUN;
        end
        E_DN_RUN: begin
          if (glast_x) begin
            // drain: the last group's products (OP_MUL writes / MACC
            // accumulates) are still in the PE multiply pipe
            drain_q <= '0;
            st_q <= E_DN_DRAIN;
          end else begin
            lv_x    <= lv_p;
            glast_x <= glast_p;
            for (int p = 0; p < PE_COUNT; p++) begin
              woff_x[p]  <= woff_p[p];
              wbank_x[p] <= wbank_p[p];
            end
            if (!glast_p) begin
              lv_p    <= dn_lane_valid;
              glast_p <= dn_group_last;
              for (int p = 0; p < PE_COUNT; p++) begin
                woff_p[p]  <= dn_woff[p];
                wbank_p[p] <= dn_wbank[p];
              end
            end else begin
              lv_p    <= '0;
              glast_p <= 1'b0;
            end
          end
        end
        E_DN_DRAIN: begin
          // 3 cycles: covers the 3-stage multiply pipe (OP_MUL writes, MACC
          // accumulates) and the 2-stage ADD-family write pipe
          drain_q <= drain_q + 2'd1;
          if (drain_q == 2'd2) st_q <= acc_mode ? E_TREE_CAP : E_DONE;
        end

        // ---------------- sparse ----------------
        E_SP_IDX_ISSUE: begin
          idx_bank_x <= sp_idx_bank;
          st_q <= (c_q.cmd_op == CMD_REDUCTION) ? E_SP_IDX_GAP : E_SP8_PRIME;
        end
        E_SP_IDX_GAP: st_q <= E_SP_IDX_WAIT;   // 2-cycle read in flight
        E_SP_IDX_WAIT: begin
          // (REDUCTION only) SG latches j + mask this edge (step_idx comb)
          st_q <= E_SP_MUL;
        end
        E_SP_MUL: begin
          // (REDUCTION only) row_base_q <= j*stride latches this edge
          // (red_mul comb) — the one registered multiply on the narrow path
          st_q <= E_SP_D_ISSUE;
        end

        // -- wide-walk priming: park idx[0] and idx[1] in the SG's index
        //    pipeline so every row base is a registered multiply. Reads take
        //    2 cycles: idx[0] (issued in IDX_ISSUE) arrives in PRIME2, idx[1]
        //    (issued in PRIME) arrives in PRIME3.
        E_SP8_PRIME: begin
          // idx[0] in flight; idx[1] issuing this cycle
          st_q <= E_SP8_PRIME2;
        end
        E_SP8_PRIME2: begin
          // idx[0] lands in j_nxt_q this edge (idx_arrive); WGATHER weight
          // B[0] issuing on port B; switch the idx read mux to idx[1]'s bank
          idx_bank_x <= sp_idx_pf_bank;
          b_bank_x   <= sp_b_bank;
          st_q <= E_SP8_PRIME3;
        end
        E_SP8_PRIME3: begin
          // row 0's base latches this edge (row_latch reads j_nxt_q = idx[0]
          // before idx_arrive overwrites it with idx[1] — nonblocking)
          st_q <= E_SP8_NEXT;
        end

        // -- REDUCTION: original two-cycle-per-word schedule (arbitrary
        //    per-index addresses could collide on a bank if fetched 8-wide)
        E_SP_D_ISSUE: begin
          data_bank_x <= sp_data_bank;
          st_q <= E_SP_D_GAP;
        end
        E_SP_D_GAP: st_q <= E_SP_D_WAIT;   // 2-cycle read in flight
        E_SP_D_WAIT: begin
          if (!sp_d_last) begin
            st_q <= E_SP_D_ISSUE;                 // step_d advances combinationally
          end else if (sp_m_last) begin
            st_q <= E_TREE_CAP;                   // global fold done
          end else begin
            st_q <= E_SP_IDX_ISSUE;               // step_m combinational
          end
        end

        // -- SPARSE / WGATHER / EMBEDDING: 8-wide row walk with 2-ahead index
        //    prefetch. E_SP8_NEXT issues chunk 0 from the registered row base
        //    while the NEXT row's index word arrives; the row's tree fold
        //    runs pipelined behind the next row's walk.
        E_SP8_NEXT: begin
          if (b_by_m) w_q <= rdata_b[b_bank_x];   // prefetched weight B[m]
          ra_x    <= sp8_arot;                    // row-constant rotations
          rb_x    <= sp8_brot;
          // chunk 0 enters the in-flight stage; exec stage bubbles while the
          // 2-cycle read completes
          lv8_p   <= sp8_lane_valid;
          clast_p <= sp8_chunk_last;
          chunk_p <= 3'd0;
          for (int p = 0; p < PE_COUNT; p++) begin
            ewb_p[p] <= sp8_wbank[p];
            ewo_p[p] <= sp8_woff[p];
          end
          lv8_x   <= '0;
          clast_x <= 1'b0;
          st_q <= E_SP8_RUN;
        end
        E_SP8_RUN: begin
          // (WGATHER accvec accumulate lives in the unconditional wg_* pipe
          // above — products emerge from the PE pipe 3 cycles after exec)
          // in-flight → exec advance, every cycle
          lv8_x   <= lv8_p;
          clast_x <= clast_p;
          chunk_x <= chunk_p;
          for (int p = 0; p < PE_COUNT; p++) begin
            ewb_x[p] <= ewb_p[p];
            ewo_x[p] <= ewo_p[p];
          end
          if (!clast_p && !clast_x) begin
            // in-flight regs for the chunk being issued this cycle
            lv8_p   <= sp8_lane_valid;
            clast_p <= sp8_chunk_last;
            chunk_p <= chunk_p + 3'd1;
            for (int p = 0; p < PE_COUNT; p++) begin
              ewb_p[p] <= sp8_wbank[p];
              ewo_p[p] <= sp8_woff[p];
            end
          end else begin
            lv8_p   <= '0;
            clast_p <= 1'b0;
          end
          if (clast_x) begin
            // row boundary: the final chunk's operands entered the PE pipe at
            // this edge — the tree snapshot is staged through pend0/pend0b.
            // Capture the writeback target before step_m advances m; latch
            // the prefetch banks (idx[m+2] was issued last cycle and lands at
            // the next E_SP8_NEXT; row_latch resolves idx[m+1]'s base now).
            if (c_q.cmd_op == CMD_SPARSE) begin
              pend0_v    <= 1'b1;
              pend0_bank <= sp_out_wbank;
              pend0_off  <= sp_out_woff;
            end
            idx_bank_x <= sp_idx_pf2_bank;
            b_bank_x   <= sp_b_pf_bank;
            drain_q    <= '0;
            unique case (c_q.cmd_op)
              CMD_SPARSE:  st_q <= sp_m_last ? E_SP8_DRAIN : E_SP8_NEXT;
              // WGATHER and EMBEDDING both need the flush: the last chunk's
              // products/writes are still in the registered pipes
              default:     st_q <= sp_m_last ? E_WG_FLUSH  : E_SP8_NEXT;
            endcase                               // step_m combinational
          end
        end
        E_SP8_DRAIN: begin
          // let the snapshot stages + tree pipeline finish the final rows'
          // writebacks
          if (!pend0_v && !pend0b_v && !pend_v && !pend2_v &&
              !p4_v && !p2_v && !p1_v)
            st_q <= E_DONE;
        end

        // ---------------- reduction tree (3 registered levels) ----------------
        E_TREE_CAP: st_q <= E_TREE0;   // eff_q captures acc_out_eff this edge
        E_TREE0: begin
          // eff_q: for REDUCTION the final gathered word's accumulate was
          // still in the PE pipe at E_TREE_CAP and eff_q caught it; for the
          // dense path E_DN_DRAIN already let acc settle (eff == acc).
          for (int i = 0; i < 4; i++)
            tr4[i] <= tnode(tree_is_max, eff_q[2*i], eff_q[2*i+1]);
          st_q <= E_TREE1;
        end
        E_TREE1: begin
          for (int i = 0; i < 2; i++)
            tr2[i] <= tnode(tree_is_max, tr4[2*i], tr4[2*i+1]);
          st_q <= E_TREE2;
        end
        E_TREE2: begin
          tr1 <= tnode(tree_is_max, tr2[0], tr2[1]);
          st_q <= E_TREE_WB;
        end
        E_TREE_WB: st_q <= E_DONE;   // dense acc-fold / REDUCTION scalar

        // ---------------- WGATHER output vector (8 words/cycle) -------------
        E_WG_FLUSH: begin
          // 3 cycles: the last chunk's products/writes are still crossing the
          // registered pipes (3-stage accvec accumulate for WGATHER, the emw
          // write pipe for EMBEDDING) — drain before reading back / finishing
          drain_q <= drain_q + 2'd1;
          if (drain_q == 2'd2)
            st_q <= (c_q.cmd_op == CMD_WGATHER) ? E_WG_WB : E_DONE;
        end
        E_WG_WB: begin
          if ((32'(wg_d_q) << 3) + 32'd8 >= 32'(c_q.stride)) st_q <= E_DONE;
          else wg_d_q <= wg_d_q + ADDR_WIDTH'(1);
        end

        // ---------------- softmax ----------------
        E_SM_RUN: begin
          if (sm_done) st_q <= E_DONE;
        end

        E_DONE: begin
          cmd_done <= 1'b1;
          st_q <= E_IDLE;
        end

        default: st_q <= E_IDLE;
      endcase
    end
  end

endmodule

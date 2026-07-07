// Dispatch engine: FSM, operand routing, accumulation, reduction tree,
// writeback. Drives the scatter_gather_engine for all address generation and
// the 8 configurable PEs for all arithmetic. Masking is handled entirely by
// the PE mask gate — a masked index entry contributes zero / is skipped with
// no engine special-casing anywhere.
//
// Dense: pipelined groups of 8 (issue group g+1 while executing group g) → ~8 elem/cyc.
// Sparse: two-cycle-per-word schedule (see scatter_gather_engine.sv).
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
  logic [DATA_WIDTH-1:0] pe_res [PE_COUNT];
  logic [DATA_WIDTH-1:0] pe_acc [PE_COUNT];

  for (genvar p = 0; p < PE_COUNT; p++) begin : g_pe
    configurable_pe u_pe (
      .clk(clk), .pe_cfg(pe_cfg[p]),
      .operand_a(pe_a[p]), .operand_b(pe_b[p]),
      .mask_bit(pe_msk[p]), .en(pe_en[p]), .acc_rst(acc_rst),
      .result(pe_res[p]), .acc_out(pe_acc[p]),
      /* verilator lint_off PINCONNECTEMPTY */
      .mul_out()
      /* verilator lint_on PINCONNECTEMPTY */
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

  scatter_gather_engine u_sg (
    .clk(clk), .rst_n(rst_n),
    .start(sg_start),
    .src_a(c_q.src_a), .src_b(c_q.src_b), .dst(c_q.dst),
    .idx_base(c_q.idx_base), .stride(c_q.stride),
    .len(c_q.len), .idx_len(c_q.idx_len),
    .step_group(step_group), .step_idx(step_idx),
    .step_d(step_d), .step_m(step_m),
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
    .sp_m_last(sp_m_last), .sp_d_last(sp_d_last)
  );

  // ------------------------------------------------------------------
  // FSM
  // ------------------------------------------------------------------
  typedef enum logic [4:0] {
    E_IDLE, E_CFG, E_DISPATCH,
    E_DN_PRIME, E_DN_RUN,
    E_SP_IDX_ISSUE, E_SP_IDX_WAIT, E_SP_D_ISSUE, E_SP_D_WAIT,
    E_TREE0, E_TREE1, E_TREE2, E_TREE_WB,
    E_SP_ROW_WB,
    E_WG_WB,
    E_SM_RUN,
    E_DONE
  } estate_e;
  estate_e st_q;

  assign engine_busy = (st_q != E_IDLE);

  // dense pipeline registers (exec stage = group issued last cycle)
  logic [PE_COUNT-1:0] lv_x;
  logic [BANK_AW-1:0]  woff_x [PE_COUNT];
  logic [2:0]          wbank_x [PE_COUNT];
  logic                glast_x;

  // sparse pipeline registers
  logic [2:0]  data_bank_x, b_bank_x, idx_bank_x;
  logic [ADDR_WIDTH-1:0] d_x;
  logic [2:0]  emb_wbank_x;
  logic [BANK_AW-1:0] emb_woff_x;
  logic [DATA_WIDTH-1:0] w_q;   // WGATHER row weight B[m]

  // WGATHER accumulator vector
  logic [DATA_WIDTH-1:0] accvec_q [ACCVEC_DEPTH];
  logic [ADDR_WIDTH-1:0] wg_d_q;

  // reduction tree registers
  logic [DATA_WIDTH-1:0] tr4 [4], tr2 [2], tr1;

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
  assign step_m   = ((st_q == E_SP_D_WAIT) && sp_d_last && !sp_m_last &&
                     (c_q.cmd_op != CMD_SPARSE)) ||
                    ((st_q == E_TREE_WB) && (c_q.cmd_op == CMD_SPARSE) && !sp_m_last);

  always_comb begin
    // defaults
    for (int k = 0; k < PE_COUNT; k++) begin
      eng_raddr_a[k] = '0;
      eng_raddr_b[k] = '0;
      eng_we[k]      = 1'b0;
      eng_woff[k]    = '0;
      eng_wdata[k]   = '0;
      pe_a[k]  = '0;
      pe_b[k]  = '0;
      pe_msk[k] = 1'b1;
      pe_en[k] = 1'b0;
    end
    step_group = 1'b0;
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
      E_DN_RUN: begin
        // issue group grp_q (if any left)
        for (int k = 0; k < PE_COUNT; k++) begin
          eng_raddr_a[k] = dn_offA[k];
          eng_raddr_b[k] = dn_offB[k];
        end
        step_group = !glast_x;
        // execute group issued last cycle
        for (int p = 0; p < PE_COUNT; p++) begin
          pe_a[p]  = rdata_a[dn_abank[p]];
          pe_b[p]  = rdata_b[dn_bbank[p]];
          pe_en[p] = lv_x[p];
          if (lv_x[p]) begin
            if (!acc_mode) begin
              eng_we[wbank_x[p]]    = 1'b1;
              eng_woff[wbank_x[p]]  = woff_x[p];
              eng_wdata[wbank_x[p]] = pe_res[p];
            end
            ops_inc = ops_inc + 4'd1;
          end
        end
      end

      // ---------------- sparse ----------------
      E_SP_IDX_ISSUE: begin
        eng_raddr_a[sp_idx_bank] = sp_idx_off;
        if (b_by_m) eng_raddr_b[sp_b_bank] = sp_b_off;   // WGATHER weight B[m]
      end
      E_SP_D_ISSUE: begin
        eng_raddr_a[sp_data_bank] = sp_data_off;
        if (!b_by_m) eng_raddr_b[sp_b_bank] = sp_b_off;  // SPARSE operand B[d]
      end
      E_SP_D_WAIT: begin
        // gathered word arrives on data_bank_x port A
        pe_a[data_bank_x]   = rdata_a[data_bank_x];
        // REDUCTION folds the gathered value itself: present it on operand_b too,
        // so MAX-aggregation (a = own acc, result = max(acc, b)) sees it. SACC uses
        // operand_a and ignores b. Other sparse ops take the B stream / row weight.
        pe_b[data_bank_x]   = (c_q.cmd_op == CMD_REDUCTION) ? rdata_a[data_bank_x]
                              : (b_by_m ? w_q : rdata_b[b_bank_x]);
        pe_msk[data_bank_x] = sp_mask_bit;
        ops_inc = 4'd1;
        unique case (c_q.cmd_op)
          CMD_SPARSE, CMD_REDUCTION: pe_en[data_bank_x] = 1'b1;
          CMD_EMBEDDING: begin
            eng_we[emb_wbank_x]    = 1'b1;
            eng_woff[emb_wbank_x]  = emb_woff_x;
            eng_wdata[emb_wbank_x] = pe_res[data_bank_x];
          end
          default: ;   // WGATHER accumulates into accvec (in ff below)
        endcase
      end

      // ---------------- writebacks ----------------
      E_TREE_WB: begin
        // dense-reduction / REDUCTION scalar → dst[0]; SPARSE row → dst+m
        if (c_q.cmd_op == CMD_SPARSE) begin
          eng_we[sp_out_wbank]    = 1'b1;
          eng_woff[sp_out_wbank]  = sp_out_woff;
          eng_wdata[sp_out_wbank] = tr1;
        end else begin
          eng_we[bank_of(c_q.dst)]    = 1'b1;
          eng_woff[bank_of(c_q.dst)]  = off_of(c_q.dst);
          eng_wdata[bank_of(c_q.dst)] = tr1;
        end
      end
      E_WG_WB: begin : wgwb
        logic [ADDR_WIDTH-1:0] wa;
        wa = c_q.dst + wg_d_q;
        eng_we[bank_of(wa)]    = 1'b1;
        eng_woff[bank_of(wa)]  = off_of(wa);
        eng_wdata[bank_of(wa)] = accvec_q[wg_d_q[5:0]];
      end
      default: ;
    endcase
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
      acc_rst <= 1'b0;
      sm_start <= 1'b0; sm_src <= '0; sm_dst <= '0; sm_len <= '0;
      error_q <= 1'b0; error_code_q <= ERR_NONE;
      lv_x <= '0; glast_x <= 1'b0;
      data_bank_x <= '0; b_bank_x <= '0; idx_bank_x <= '0;
      d_x <= '0; emb_wbank_x <= '0; emb_woff_x <= '0;
      w_q <= '0; wg_d_q <= '0;
      tr1 <= '0;
      for (int i = 0; i < 4; i++) tr4[i] <= '0;
      for (int i = 0; i < 2; i++) tr2[i] <= '0;
      for (int i = 0; i < PE_COUNT; i++) begin
        woff_x[i] <= '0; wbank_x[i] <= '0;
      end
      for (int i = 0; i < ACCVEC_DEPTH; i++) accvec_q[i] <= '0;
    end else begin
      cmd_done <= 1'b0;
      sg_start <= 1'b0;
      acc_rst  <= 1'b0;
      sm_start <= 1'b0;

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
          acc_rst <= 1'b1;
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
          lv_x    <= dn_lane_valid;
          glast_x <= dn_group_last;
          for (int p = 0; p < PE_COUNT; p++) begin
            woff_x[p]  <= dn_woff[p];
            wbank_x[p] <= dn_wbank[p];
          end
          st_q <= E_DN_RUN;
        end
        E_DN_RUN: begin
          if (glast_x) begin
            st_q <= acc_mode ? E_TREE0 : E_DONE;
          end else begin
            lv_x    <= dn_lane_valid;
            glast_x <= dn_group_last;
            for (int p = 0; p < PE_COUNT; p++) begin
              woff_x[p]  <= dn_woff[p];
              wbank_x[p] <= dn_wbank[p];
            end
          end
        end

        // ---------------- sparse ----------------
        E_SP_IDX_ISSUE: begin
          idx_bank_x <= sp_idx_bank;
          b_bank_x   <= sp_b_bank;     // WGATHER weight bank
          st_q <= E_SP_IDX_WAIT;
        end
        E_SP_IDX_WAIT: begin
          // SG latches j + mask this edge (step_idx is combinational)
          if (b_by_m) w_q <= rdata_b[b_bank_x];
          st_q <= E_SP_D_ISSUE;
        end
        E_SP_D_ISSUE: begin
          data_bank_x <= sp_data_bank;
          b_bank_x    <= sp_b_bank;
          d_x         <= sp_d;
          emb_wbank_x <= sp_emb_wbank;
          emb_woff_x  <= sp_emb_woff;
          st_q <= E_SP_D_WAIT;
        end
        E_SP_D_WAIT: begin
          // WGATHER: accumulate mask-gated product into accvec[d]
          if (c_q.cmd_op == CMD_WGATHER)
            accvec_q[d_x[5:0]] <= accvec_q[d_x[5:0]] + pe_res[data_bank_x];

          if (!sp_d_last) begin
            st_q <= E_SP_D_ISSUE;                 // step_d advances combinationally
          end else begin
            unique case (c_q.cmd_op)
              CMD_SPARSE: st_q <= E_TREE0;        // per-row tree → dst[m]
              CMD_REDUCTION: begin
                if (sp_m_last) st_q <= E_TREE0;   // global fold done
                else st_q <= E_SP_IDX_ISSUE;      // step_m combinational
              end
              CMD_EMBEDDING, CMD_WGATHER: begin
                if (sp_m_last) begin
                  st_q <= (c_q.cmd_op == CMD_WGATHER) ? E_WG_WB : E_DONE;
                end else begin
                  st_q <= E_SP_IDX_ISSUE;         // step_m combinational
                end
              end
              default: st_q <= E_DONE;
            endcase
          end
        end

        // ---------------- reduction tree (3 registered levels) ----------------
        E_TREE0: begin
          for (int i = 0; i < 4; i++)
            tr4[i] <= tnode(tree_is_max, pe_acc[2*i], pe_acc[2*i+1]);
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
        E_TREE_WB: begin
          if (c_q.cmd_op == CMD_SPARSE) begin
            if (sp_m_last) st_q <= E_DONE;
            else begin
              acc_rst <= 1'b1;         // fresh accumulators for the next row
              st_q <= E_SP_IDX_ISSUE;  // step_m combinational
            end
          end else begin
            st_q <= E_DONE;
          end
        end

        // ---------------- WGATHER output vector ----------------
        E_WG_WB: begin
          if (wg_d_q + ADDR_WIDTH'(1) >= c_q.stride) st_q <= E_DONE;
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

// Three-pass fixed-point softmax over a score vector in HDM, Q16.16
// throughout. Pass 0 scans the running maximum; pass 1 computes exp(x - max)
// via a 256-entry ROM over [-8,8) with linear interpolation, written to dst
// as scratch, sum S accumulated (48-bit); pass 2: weight = exp<<16 / S with a
// serial restoring divider — exact, and cycle count isn't a claimed metric
// here, so simple beats fast.
// The max-subtraction makes the result exact for logits of ANY magnitude
// (softmax is shift-invariant): post-shift inputs are always <= 0, inside the
// ROM's covered half, and values below -8 correctly underflow to weight 0.
//
// Timing: the exp evaluation is a 5-stage pipeline walked serially by the FSM
// (WAIT: x-max subtract; IDX: clamp/bias -> ROM index; LUT: sync ROM read;
// MUL: delta*frac DSP; ADD: interpolate). The old single-cycle version (two
// async LUT reads + multiply + add off the live SRAM read) was the design's
// worst path at -22 ns @ 10 ns.
// ROM entries are {delta, y0} pairs so one read serves the interpolation; the
// table is $readmemh'd from exp_lut_q16.mem (see scripts/gen_exp_lut.sh),
// which also keeps synthesis free of real-math $exp() elaboration.
// Memory interface: at most one read + one write command per cycle, granted
// the bank ports by nmc_top while sm_busy is high.
module softmax_unit
  import nmc_pkg::*;
(
  input  logic                  clk,
  input  logic                  rst_n,

  input  logic                  sm_start,
  input  logic [ADDR_WIDTH-1:0] sm_src,     // scores base (Q16.16)
  input  logic [ADDR_WIDTH-1:0] sm_dst,     // weights out (Q16.16)
  input  logic [15:0]           sm_len,
  output logic                  sm_busy,
  output logic                  sm_done,    // 1-cycle pulse

  // memory access (via nmc_top port mux)
  output logic                  rd_en,
  output logic [2:0]            rd_bank,
  output logic [BANK_AW-1:0]    rd_off,
  input  logic [DATA_WIDTH-1:0] rd_data,    // valid cycle after rd_en
  output logic                  wr_en,
  output logic [2:0]            wr_bank,
  output logic [BANK_AW-1:0]    wr_off,
  output logic [DATA_WIDTH-1:0] wr_data
);

  // ---------------- exp ROM: {delta[k], y0[k]}, y0[k] = exp(-8 + k/16) in Q16.16 ----------------
  // delta[k] = y0[k+1] - y0[k] (0 at k=255, matching the old y1=y0 clamp),
  // precomputed so interpolation needs a single synchronous read — maps to
  // one block RAM instead of two async LUT-mux trees.
  logic [7:0]  k_q;                // ROM index (set in S_P1_WAIT)
  logic [63:0] exp_rom [0:255];
  logic [63:0] rom_q;
  initial $readmemh("fpga/rtl/exp_lut_q16.mem", exp_rom);
  always_ff @(posedge clk) rom_q <= exp_rom[k_q];

  // ---------------- FSM ----------------
  // *_GAP states: bank reads are 2-cycle (outpost register + BRAM register),
  // so each issue gets one extra wait before the data-capture state
  typedef enum logic [4:0] {
    S_IDLE, S_MX_ISSUE, S_MX_GAP, S_MX_WAIT, S_MX_CMP,
    S_P1_ISSUE, S_P1_GAP, S_P1_WAIT, S_P1_SUB, S_P1_IDX, S_P1_LUT, S_P1_MUL,
    S_P1_ADD, S_P1_WRITE,
    S_P2_ISSUE, S_P2_GAP, S_P2_WAIT, S_DIV, S_P2_WRITE, S_DONE
  } state_e;
  state_e st_q;

  logic [ADDR_WIDTH-1:0] src_q, dst_q;
  logic [15:0] len_q, i_q;
  logic signed [31:0] max_q;       // pass-0 running maximum (numeric stability)
  logic [47:0] sum_q;              // sum of exp values (Q16.16, up to 64K entries)
  logic [31:0] exp_q;

  // the SRAM word is captured into rd_cap_q before ANY arithmetic touches
  // it — BRAM clock-to-out + the 8:1 bank mux already spend half the 6.67 ns
  // budget, so compares/subtracts run register-to-register a cycle later
  logic [31:0] rd_cap_q;

  // x - max in 33 bits (Q16.16 differences can exceed 32-bit range), clamped
  // to the exp underflow floor: anything <= -8.0 underflows to 0 downstream
  logic signed [32:0] diff33;
  logic signed [31:0] shifted;
  assign diff33  = {rd_cap_q[31], rd_cap_q} - {max_q[31], max_q};
  assign shifted = (diff33 <= -33'sd524288) ? -32'sd524288 : diff33[31:0];

  // exp stage A1 (consumed in S_P1_WAIT): the BRAM-read → bank-mux → 33-bit
  // subtract cone ends at shifted_q — chaining the clamp/bias adds behind it
  // in the same cycle missed 10 ns
  logic signed [31:0] shifted_q;
  logic               uf;          // underflow: exp(x <= -8.0) -> 0
  assign uf = (diff33 <= -33'sd524288);

  // exp stage A2 (comb from shifted_q, consumed in S_P1_IDX): clamp to
  // [-8, 8) and split into ROM index + interpolation fraction
  logic signed [31:0] xc;
  logic [31:0] span;               // shifted + 8.0 in Q16.16, in [0, 16.0)
  assign xc   = (shifted_q >= 32'sd524272) ? 32'sd524272 : shifted_q; // clamp just below +8.0
  assign span = $unsigned(xc + 32'sd524288);

  // exp pipeline registers (stage B is the sync ROM read above)
  logic [11:0]        frac_q;      // interp fraction (set in S_P1_IDX)
  logic               uf_q;
  logic [31:0]        y0_q;        // ROM y0          (set in S_P1_MUL)
  logic signed [44:0] prod_q;      // delta * frac    (set in S_P1_MUL, DSP)

  // serial restoring divider: num / den, 48 iterations
  logic [63:0] div_num_q;          // exp << 16
  logic [47:0] div_den_q;
  logic [63:0] div_rem_q;
  logic [47:0] div_quo_q;
  logic [5:0]  div_i_q;

  logic [ADDR_WIDTH-1:0] cur_src_addr, cur_dst_addr;
  assign cur_src_addr = src_q + ADDR_WIDTH'(i_q);
  assign cur_dst_addr = dst_q + ADDR_WIDTH'(i_q);

  assign sm_busy = (st_q != S_IDLE);

  always_comb begin
    rd_en   = 1'b0;
    rd_bank = '0;
    rd_off  = '0;
    wr_en   = 1'b0;
    wr_bank = '0;
    wr_off  = '0;
    wr_data = '0;
    unique case (st_q)
      S_MX_ISSUE: begin
        rd_en   = 1'b1;
        rd_bank = bank_of(cur_src_addr);
        rd_off  = off_of(cur_src_addr);
      end
      S_P1_ISSUE: begin
        rd_en   = 1'b1;
        rd_bank = bank_of(cur_src_addr);
        rd_off  = off_of(cur_src_addr);
      end
      S_P1_WRITE: begin
        wr_en   = 1'b1;
        wr_bank = bank_of(cur_dst_addr);
        wr_off  = off_of(cur_dst_addr);
        wr_data = exp_q;
      end
      S_P2_ISSUE: begin
        rd_en   = 1'b1;
        rd_bank = bank_of(cur_dst_addr);   // pass 2 reads the scratch exp values
        rd_off  = off_of(cur_dst_addr);
      end
      S_P2_WRITE: begin
        wr_en   = 1'b1;
        wr_bank = bank_of(cur_dst_addr);
        wr_off  = off_of(cur_dst_addr);
        wr_data = {div_quo_q[31:0]};
      end
      default: ;
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st_q <= S_IDLE;
      src_q <= '0; dst_q <= '0; len_q <= '0; i_q <= '0;
      sum_q <= '0; exp_q <= '0; sm_done <= 1'b0; max_q <= 32'sh8000_0000;
      k_q <= '0; frac_q <= '0; uf_q <= 1'b0; y0_q <= '0; prod_q <= '0;
      shifted_q <= '0; rd_cap_q <= '0;
      div_num_q <= '0; div_den_q <= '0; div_rem_q <= '0; div_quo_q <= '0; div_i_q <= '0;
    end else begin
      sm_done <= 1'b0;
      unique case (st_q)
        S_IDLE: begin
          if (sm_start) begin
            src_q <= sm_src; dst_q <= sm_dst; len_q <= sm_len;
            i_q <= '0; sum_q <= '0;
            max_q <= 32'sh8000_0000;
            st_q <= (sm_len == 0) ? S_DONE : S_MX_ISSUE;
          end
        end
        S_MX_ISSUE: st_q <= S_MX_GAP;
        S_MX_GAP:   st_q <= S_MX_WAIT;
        S_MX_WAIT: begin
          rd_cap_q <= rd_data;    // capture only; compare runs from registers
          st_q     <= S_MX_CMP;
        end
        S_MX_CMP: begin
          if ($signed(rd_cap_q) > max_q) max_q <= $signed(rd_cap_q);
          if (i_q + 16'd1 >= len_q) begin
            i_q  <= '0;
            st_q <= S_P1_ISSUE;
          end else begin
            i_q  <= i_q + 16'd1;
            st_q <= S_MX_ISSUE;
          end
        end
        S_P1_ISSUE: st_q <= S_P1_GAP;
        S_P1_GAP:   st_q <= S_P1_WAIT;
        S_P1_WAIT: begin
          rd_cap_q <= rd_data;    // capture only
          st_q     <= S_P1_SUB;
        end
        S_P1_SUB: begin
          // stage A1: x - max, register-to-register
          shifted_q <= shifted;
          uf_q      <= uf;
          st_q      <= S_P1_IDX;
        end
        S_P1_IDX: begin
          // stage A2: clamp + bias, split into ROM index / fraction
          k_q    <= span[19:12];
          frac_q <= span[11:0];
          st_q   <= S_P1_LUT;
        end
        S_P1_LUT: st_q <= S_P1_MUL;   // stage B: rom_q <= exp_rom[k_q] (above)
        S_P1_MUL: begin
          // stage C: interpolation product (one DSP, registered operands)
          y0_q   <= rom_q[31:0];
          prod_q <= $signed(rom_q[63:32]) * $signed({1'b0, frac_q});
          st_q   <= S_P1_ADD;
        end
        S_P1_ADD: begin
          // stage D: y0 + (delta*frac >> 12); same widths/truncation as the
          // old single-cycle interp expression, so results are bit-identical
          exp_q <= uf_q ? 32'd0 : (y0_q + 32'(prod_q >>> 12));
          st_q  <= S_P1_WRITE;
        end
        S_P1_WRITE: begin
          sum_q <= sum_q + {16'd0, exp_q};
          if (i_q + 16'd1 >= len_q) begin
            i_q  <= '0;
            st_q <= S_P2_ISSUE;
          end else begin
            i_q  <= i_q + 16'd1;
            st_q <= S_P1_ISSUE;
          end
        end
        S_P2_ISSUE: st_q <= S_P2_GAP;
        S_P2_GAP:   st_q <= S_P2_WAIT;
        S_P2_WAIT: begin
          // start division: (exp << 16) / sum
          div_num_q <= {16'd0, rd_data, 16'd0};
          div_den_q <= (sum_q == 0) ? 48'd1 : sum_q;
          div_rem_q <= '0;
          div_quo_q <= '0;
          div_i_q   <= 6'd0;
          st_q      <= S_DIV;
        end
        S_DIV: begin : divstep
          logic [63:0] rem_shift;
          rem_shift = {div_rem_q[62:0], div_num_q[63]};
          div_num_q <= {div_num_q[62:0], 1'b0};
          if (rem_shift >= {16'd0, div_den_q}) begin
            div_rem_q <= rem_shift - {16'd0, div_den_q};
            div_quo_q <= {div_quo_q[46:0], 1'b1};
          end else begin
            div_rem_q <= rem_shift;
            div_quo_q <= {div_quo_q[46:0], 1'b0};
          end
          if (div_i_q == 6'd63) st_q <= S_P2_WRITE;
          div_i_q <= div_i_q + 6'd1;
        end
        S_P2_WRITE: begin
          if (i_q + 16'd1 >= len_q) st_q <= S_DONE;
          else begin
            i_q  <= i_q + 16'd1;
            st_q <= S_P2_ISSUE;
          end
        end
        S_DONE: begin
          sm_done <= 1'b1;
          st_q    <= S_IDLE;
        end
        default: st_q <= S_IDLE;
      endcase
    end
  end

endmodule

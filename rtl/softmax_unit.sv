// Two-pass fixed-point softmax over a score vector in HDM, Q16.16 throughout.
// Pass 1: exp(x) via a 256-entry LUT over [-8,8) with linear interpolation,
// written to dst as scratch, sum S accumulated (48-bit). Pass 2:
// weight = exp<<16 / S with a serial restoring divider — exact, and cycle
// count isn't a claimed metric here, so simple beats fast.
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

  // ---------------- exp LUT: exp(-8 + k/16) in Q16.16, k = 0..255 ----------------
  logic [31:0] exp_lut [0:255];
  initial begin
    // Generated values: exp(-8 + k*0.0625) * 65536, rounded.
    // Filled by $readmemh from an embedded table would complicate the build;
    // computed here procedurally instead using real math (simulation-only init,
    // synthesis would use a ROM macro with the same contents).
    for (int k = 0; k < 256; k++) begin
      real x;
      x = -8.0 + 0.0625 * k;
      exp_lut[k] = 32'($rtoi($exp(x) * 65536.0 + 0.5));
    end
  end

  // exp of a Q16.16 input with clamp to [-8, 8)
  function automatic logic [31:0] exp_q16(input logic signed [31:0] x);
    logic signed [31:0] xc;
    logic [31:0] span;      // x + 8.0 in Q16.16, in [0, 16.0)
    logic [7:0]  k;
    logic [11:0] frac;
    logic [31:0] y0, y1;
    logic signed [63:0] interp;
    if (x <= -32'sd524288) return 32'd0;               // x <= -8.0 → 0
    xc = (x >= 32'sd524272) ? 32'sd524272 : x;          // clamp just below +8.0
    span = $unsigned(xc + 32'sd524288);                 // 0 .. <1048576 (16.0 Q16.16)
    k    = span[19:12];                                 // /4096 → LUT index
    frac = span[11:0];
    y0 = exp_lut[k];
    y1 = (k == 8'hFF) ? exp_lut[k] : exp_lut[k+1];
    interp = $signed({32'd0, y0}) +
             ((($signed({32'd0, y1}) - $signed({32'd0, y0})) * $signed({52'd0, frac})) >>> 12);
    return interp[31:0];
  endfunction

  // ---------------- FSM ----------------
  typedef enum logic [3:0] {
    S_IDLE, S_P1_ISSUE, S_P1_WAIT, S_P1_WRITE,
    S_P2_ISSUE, S_P2_WAIT, S_DIV, S_P2_WRITE, S_DONE
  } state_e;
  state_e st_q;

  logic [ADDR_WIDTH-1:0] src_q, dst_q;
  logic [15:0] len_q, i_q;
  logic [47:0] sum_q;              // sum of exp values (Q16.16, up to 64K entries)
  logic [31:0] exp_q;

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
      sum_q <= '0; exp_q <= '0; sm_done <= 1'b0;
      div_num_q <= '0; div_den_q <= '0; div_rem_q <= '0; div_quo_q <= '0; div_i_q <= '0;
    end else begin
      sm_done <= 1'b0;
      unique case (st_q)
        S_IDLE: begin
          if (sm_start) begin
            src_q <= sm_src; dst_q <= sm_dst; len_q <= sm_len;
            i_q <= '0; sum_q <= '0;
            st_q <= (sm_len == 0) ? S_DONE : S_P1_ISSUE;
          end
        end
        S_P1_ISSUE: st_q <= S_P1_WAIT;
        S_P1_WAIT: begin
          exp_q <= exp_q16($signed(rd_data));
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
        S_P2_ISSUE: st_q <= S_P2_WAIT;
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

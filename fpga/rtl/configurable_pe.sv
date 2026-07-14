// Runtime-configurable processing element, one per SRAM bank.
// pe_cfg: [3:0] op, [5:4] src_sel (00 own acc, 01 bankA, 10 bankB, 11 zero), [6] mask_en.
//
// Fully registered-input datapath: operands latch into opa_q/opb_q first and
// the ALU, accumulator, and writeback data all compute from the registers —
// a single-cycle BRAM-read → ALU → consumer cone misses 10 ns. Consequences:
//   - `result` is the mask-gated ALU value one cycle AFTER the operands were
//     presented (the engine's delayed write pipes consume it in that cycle);
//   - the multiply is three stages (fabric capture FF + DSP AREG + MREG):
//     OP_MUL products appear on `mul_res` three cycles after the operands;
//   - accumulating ops (MACC / SACC / MAX-with-src00) update acc_q via
//     delayed valid flags (macc_v3 / sacc_v1 / maxa_v1) that ride the pipe
//     with the operands' mask, always combining the LIVE acc_q with the
//     registered operand — II=1 accumulation with no stale-acc hazard;
//   - `acc_out_eff` exposes acc_q plus whatever accumulate is still in
//     flight, so row-boundary / fold-boundary snapshots need no extra wait.
// acc_rst loads the op-appropriate identity (0, or INT_MIN for MAX) and only
// touches acc_q — pipe registers keep flowing.
module configurable_pe
  import nmc_pkg::*;
(
  input  logic                  clk,
  input  logic [PE_CFG_W-1:0]   pe_cfg,
  input  logic [DATA_WIDTH-1:0] operand_a,   // bank read port A (primary/gathered)
  input  logic [DATA_WIDTH-1:0] operand_b,   // bank read port B (secondary stream)
  input  logic                  mask_bit,    // from index list entry MSB
  input  logic                  en,          // engine strobe: this PE executes this cycle
  input  logic                  acc_rst,
  output logic [DATA_WIDTH-1:0] result,      // ALU value, 1-cycle latency, mask-gated
  output logic [DATA_WIDTH-1:0] acc_out,     // accumulator value (for reduction tree)
  output logic [DATA_WIDTH-1:0] acc_out_eff, // acc_q + in-flight accumulate
  output logic [DATA_WIDTH-1:0] mul_res      // a*b product, 2-cycle latency, mask-gated
);

  logic [3:0] op;
  logic [1:0] src_sel;
  logic       mask_en;
  assign op      = pe_cfg[3:0];
  assign src_sel = pe_cfg[5:4];
  assign mask_en = pe_cfg[6];

  logic signed [DATA_WIDTH-1:0] acc_q;
  assign acc_out = acc_q;

  // Operand A source mux
  logic signed [DATA_WIDTH-1:0] a, b;
  always_comb begin
    unique case (src_sel)
      2'b00:   a = acc_q;
      2'b01:   a = operand_a;
      2'b10:   a = operand_b;
      default: a = '0;
    endcase
  end
  assign b = operand_b;

  // Mask gate: when mask_en and the entry is masked out, the PE contributes nothing.
  logic active;
  assign active = !mask_en || mask_bit;

  // ---- stage 1: operand capture registers (every cycle), then a second
  // operand stage the DSP absorbs as AREG/BREG, then the product register
  // (MREG). Three stages because the bank-mux → DSP-column route alone eats
  // most of a 6.67 ns cycle: stage 1 is a fabric FF near the banks, stage 2
  // lives inside the DSP. Valid/mask bits ride the pipe in lockstep. ----
  logic signed [DATA_WIDTH-1:0]   opa_q, opb_q, opa_q2, opb_q2;
  logic signed [2*DATA_WIDTH-1:0] prod_q;
  logic                           act_q, act_qq, act_q3;
  logic                           macc_v1, macc_v2, macc_v3, sacc_v1, maxa_v1;
  always_ff @(posedge clk) begin
    opa_q   <= a;
    opb_q   <= b;
    opa_q2  <= opa_q;
    opb_q2  <= opb_q;
    prod_q  <= opa_q2 * opb_q2;      // stage 3 (DSP MREG)
    act_q   <= active;
    act_qq  <= act_q;
    act_q3  <= act_qq;
    macc_v1 <= en && active && (op == OP_MACC);
    macc_v2 <= macc_v1;
    macc_v3 <= macc_v2;
    sacc_v1 <= en && active && (op == OP_SACC);
    maxa_v1 <= en && active && (op == OP_MAX) && (src_sel == 2'b00);
  end

  assign mul_res = act_q3 ? prod_q[DATA_WIDTH-1:0] : '0;

  // ALU on the registered operands
  logic signed [DATA_WIDTH-1:0] alu;
  always_comb begin
    unique case (op)
      OP_ADD:    alu = opa_q + opb_q;
      OP_SUB:    alu = opa_q - opb_q;
      OP_MUL:    alu = '0;   // product served by mul_res (one more stage)
      OP_MAX:    alu = (opa_q > opb_q) ? opa_q : opb_q;
      OP_MIN:    alu = (opa_q < opb_q) ? opa_q : opb_q;
      OP_AND:    alu = opa_q & opb_q;
      OP_OR:     alu = opa_q | opb_q;
      OP_XOR:    alu = opa_q ^ opb_q;
      OP_MACC:   alu = '0;   // accumulate handled via macc_v2 below
      OP_SACC:   alu = acc_q + opa_q;
      OP_PASS_A: alu = opa_q;
      OP_PASS_B: alu = opb_q;
      OP_NEG:    alu = -opa_q;
      OP_ABS:    alu = (opa_q < 0) ? -opa_q : opa_q;
      OP_SHR:    alu = $signed($unsigned(opa_q) >> opb_q[4:0]);  // logical right shift
      default:   alu = '0;   // OP_ZERO
    endcase
  end

  assign result = act_q ? alu : '0;

  // In-flight accumulate view: live acc combined with the registered operand
  logic signed [DATA_WIDTH-1:0] maxab;
  assign maxab = (acc_q > opb_q) ? acc_q : opb_q;
  assign acc_out_eff = macc_v3 ? (acc_q + prod_q[DATA_WIDTH-1:0]) :
                       sacc_v1 ? (acc_q + opa_q) :
                       maxa_v1 ? maxab : acc_q;

  // Accumulating ops: delayed valids, live acc_q, registered operands.
  always_ff @(posedge clk) begin
    if (acc_rst)
      acc_q <= ((op == OP_MAX) && (src_sel == 2'b00)) ? 32'sh8000_0000 : '0;
    else if (macc_v3)
      acc_q <= acc_q + prod_q[DATA_WIDTH-1:0];
    else if (sacc_v1)
      acc_q <= acc_q + opa_q;
    else if (maxa_v1)
      acc_q <= maxab;
  end

endmodule

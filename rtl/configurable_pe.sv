// Runtime-configurable processing element, one per SRAM bank.
// Combinational ALU + one accumulator register (the only state).
// pe_cfg: [3:0] op, [5:4] src_sel (00 own acc, 01 bankA, 10 bankB, 11 zero), [6] mask_en.
// The accumulator updates when `en` is asserted and the configured op accumulates:
// MACC, SACC, and MAX-with-src_sel==00 (GNN max-aggregation against own acc).
// acc_rst loads the op-appropriate identity (0, or INT_MIN for MAX-accumulate).
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
  output logic [DATA_WIDTH-1:0] result,      // combinational, mask-gated
  output logic [DATA_WIDTH-1:0] acc_out,     // accumulator value (for reduction tree)
  output logic [DATA_WIDTH-1:0] mul_out      // raw a*b product tap (WGATHER path)
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

  logic signed [DATA_WIDTH-1:0] alu;
  logic signed [2*DATA_WIDTH-1:0] prod;
  assign prod    = a * b;
  assign mul_out = prod[DATA_WIDTH-1:0];

  always_comb begin
    unique case (op)
      OP_ADD:    alu = a + b;
      OP_SUB:    alu = a - b;
      OP_MUL:    alu = prod[DATA_WIDTH-1:0];
      OP_MAX:    alu = (a > b) ? a : b;
      OP_MIN:    alu = (a < b) ? a : b;
      OP_AND:    alu = a & b;
      OP_OR:     alu = a | b;
      OP_XOR:    alu = a ^ b;
      OP_MACC:   alu = acc_q + prod[DATA_WIDTH-1:0];
      OP_SACC:   alu = acc_q + a;
      OP_PASS_A: alu = a;
      OP_PASS_B: alu = b;
      OP_NEG:    alu = -a;
      OP_ABS:    alu = (a < 0) ? -a : a;
      OP_SHR:    alu = $signed($unsigned(a) >> b[4:0]);  // logical right shift
      default:   alu = '0;   // OP_ZERO
    endcase
  end

  // Mask gate: when mask_en and the entry is masked out, the PE contributes nothing.
  logic active;
  assign active = !mask_en || mask_bit;
  assign result = active ? alu : '0;

  // Accumulating ops
  logic acc_op;
  assign acc_op = (op == OP_MACC) || (op == OP_SACC) ||
                  ((op == OP_MAX) && (src_sel == 2'b00));

  always_ff @(posedge clk) begin
    if (acc_rst)
      acc_q <= ((op == OP_MAX) && (src_sel == 2'b00)) ? 32'sh8000_0000 : '0;
    else if (en && acc_op && active)
      acc_q <= alu;
  end

endmodule

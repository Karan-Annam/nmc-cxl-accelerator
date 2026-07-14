// Stores the 56-bit operation configuration as 8 x 7-bit PE fields.
// Written on the CFG_SUBMIT pulse; persists across commands until rewritten.
module config_regfile
  import nmc_pkg::*;
(
  input  logic                  clk,
  input  logic                  rst_n,
  input  logic                  cfg_valid,
  input  logic [CFG_WORD_W-1:0] cfg_word,
  output logic [PE_CFG_W-1:0]   pe_cfg [PE_COUNT]
);

  logic [CFG_WORD_W-1:0] cfg_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)         cfg_q <= '0;
    else if (cfg_valid) cfg_q <= cfg_word;
  end

  for (genvar k = 0; k < PE_COUNT; k++) begin : g_split
    assign pe_cfg[k] = cfg_q[k*PE_CFG_W +: PE_CFG_W];
  end

endmodule

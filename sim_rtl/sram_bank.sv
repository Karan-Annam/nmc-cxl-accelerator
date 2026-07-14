// Behavioral synchronous memory bank, 1 write port + 2 read ports, registered
// reads (1-cycle latency). This is a model of memory, not a physical SRAM
// claim — a real device would put DRAM/HBM behind a controller honoring the
// same port contract, which is why cycle counts aren't quoted as latency.
module sram_bank
  import nmc_pkg::*;
#(
  parameter int DEPTH = BANK_DEPTH,
  parameter int WIDTH = DATA_WIDTH
)(
  input  logic                     clk,
  input  logic                     we,
  input  logic [$clog2(DEPTH)-1:0] waddr,
  input  logic [WIDTH-1:0]         wdata,
  input  logic [$clog2(DEPTH)-1:0] raddr_a,
  output logic [WIDTH-1:0]         rdata_a,
  input  logic [$clog2(DEPTH)-1:0] raddr_b,
  output logic [WIDTH-1:0]         rdata_b
);

  logic [WIDTH-1:0] mem [0:DEPTH-1];

  always_ff @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    rdata_a <= mem[raddr_a];
    rdata_b <= mem[raddr_b];
  end

endmodule

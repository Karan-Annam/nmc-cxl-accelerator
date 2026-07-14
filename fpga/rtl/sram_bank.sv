// Behavioral synchronous memory bank, 1 write port + 2 read ports.
// TWO-cycle registered reads: the address/write inputs land in per-bank
// "outpost" registers first (placeable next to this bank's BRAMs), then the
// BRAM's internal address register. This is the Tier-1 timing change: the
// central engine only has to reach the nearest outpost FF instead of driving
// every BRAM pin across the die combinationally (~9 ns of wire on xc7a100t).
// Writes commit one cycle later than before; read-after-write ordering
// through the port is unchanged (both directions shift together).
// This is a model of memory, not a physical SRAM claim — a real device would
// put DRAM/HBM behind a controller honoring the same port contract.
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

  // Force full block-RAM mapping: left to itself Vivado splits the depth into
  // BRAM plus a distributed-RAM remainder whose asynchronous read
  // (RAMD64E + MUXF7/F8) lands after the address adders in the same cycle —
  // the BRAM's internal address register is what makes the read timing-clean.
  (* ram_style = "block" *) logic [WIDTH-1:0] mem [0:DEPTH-1];

  // outpost registers (stage 1 of the 2-cycle read)
  logic                     we_q;
  logic [$clog2(DEPTH)-1:0] waddr_q, ra_q, rb_q;
  logic [WIDTH-1:0]         wdata_q;

  always_ff @(posedge clk) begin
    we_q    <= we;
    waddr_q <= waddr;
    wdata_q <= wdata;
    ra_q    <= raddr_a;
    rb_q    <= raddr_b;
  end

  always_ff @(posedge clk) begin
    if (we_q) mem[waddr_q] <= wdata_q;
    rdata_a <= mem[ra_q];
    rdata_b <= mem[rb_q];
  end

endmodule

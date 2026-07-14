// CRC-16-CCITT (poly 0x1021, init 0xFFFF) over the first 66 bytes of a flit,
// combinationally. Standard polynomial, not claimed bit-identical to the
// Consortium's — the hardware design problem (parallel CRC over a 544-bit
// frame) is the same either way.
module cxl_crc16
  import nmc_pkg::*;
(
  input  logic [CXL_FLIT_W-1:0] flit,
  output logic [15:0]           crc
);

  assign crc = flit_crc(flit);

endmodule

// Parse an rx flit: CRC check, header fields, slot extraction.
// Purely combinational; the link layer decides what to do with the results.
module cxl_flit_unpack
  import nmc_pkg::*;
(
  input  logic [CXL_FLIT_W-1:0]       flit,
  output logic                        crc_ok,
  output logic [CXL_SEQ_W-1:0]        rx_seq,
  output logic [1:0]                  slot_type [CXL_SLOTS_PER_FLIT],
  output logic [8*CXL_SLOT_BYTES-1:0] slot_data [CXL_SLOTS_PER_FLIT]
);

  logic [15:0] crc_calc, crc_field;
  cxl_crc16 u_crc (.flit(flit), .crc(crc_calc));

  assign crc_field = {flit[8*(CXL_FLIT_BYTES-1) +: 8], flit[8*(CXL_FLIT_BYTES-2) +: 8]};
  assign crc_ok    = (crc_calc == crc_field);
  assign rx_seq    = flit[8 +: CXL_SEQ_W];

  always_comb begin
    for (int s = 0; s < CXL_SLOTS_PER_FLIT; s++) begin
      slot_type[s] = flit[2*s +: 2];
      slot_data[s] = flit[8*(2 + CXL_SLOT_BYTES*s) +: 8*CXL_SLOT_BYTES];
    end
  end

endmodule

// Assemble a tx flit from up to 4 slots + header fields, append CRC.
// Purely combinational: the arb/mux presents the slots, this module lays out bytes.
module cxl_flit_pack
  import nmc_pkg::*;
(
  input  logic [1:0]                 slot_type [CXL_SLOTS_PER_FLIT],
  input  logic [8*CXL_SLOT_BYTES-1:0] slot_data [CXL_SLOTS_PER_FLIT],
  input  logic [CXL_SEQ_W-1:0]       tx_seq,
  output logic [CXL_FLIT_W-1:0]      flit
);

  logic [CXL_FLIT_W-1:0] body;

  always_comb begin
    body = '0;
    // header byte 0: slot type fields
    body[7:0] = {slot_type[3], slot_type[2], slot_type[1], slot_type[0]};
    // header byte 1: [3:0] sequence number
    body[8 +: 8] = {4'd0, tx_seq};
    // slots: slot s occupies bytes 2+16s .. 17+16s
    for (int s = 0; s < CXL_SLOTS_PER_FLIT; s++)
      body[8*(2 + CXL_SLOT_BYTES*s) +: 8*CXL_SLOT_BYTES] = slot_data[s];
  end

  logic [15:0] crc;
  cxl_crc16 u_crc (.flit(body), .crc(crc));

  always_comb begin
    flit = body;
    flit[8*(CXL_FLIT_BYTES-2) +: 8] = crc[7:0];
    flit[8*(CXL_FLIT_BYTES-1) +: 8] = crc[15:8];
  end

endmodule

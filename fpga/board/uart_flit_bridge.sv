// UART <-> CXL-flit transport bridge. Carries whole 68-byte flits over a
// plain 8N1 serial link so nmc_top's flit interface can be driven from a PC:
// the CXL link layer (CRC, credits, sequence numbers, retry) still runs as
// real hardware — only the physical transport under it is substituted, since
// no Artix-class part can terminate actual CXL.
//
// Framing (both directions): 0xA5 0x5A preamble, then the 68 flit bytes
// (byte k = flit bits [8k+7:8k]). The receiver hunts for the preamble, so a
// dropped byte costs one frame, not the link. UART is ~1000x slower than the
// core clock, so a single flit buffer per direction is enough: the device
// consumes an rx flit in a handful of cycles, and tx flits serialize under
// the flit_tx_ready backpressure the link layer already honors.
//
// Single clock domain: the UART engines run on the core clock with a
// DIVISOR-cycle bit period (DIVISOR = core_clk_hz / baud).
module uart_flit_bridge
  import nmc_pkg::*;
#(
  parameter int unsigned DIVISOR = 50   // 150 MHz / 3 Mbaud
)(
  input  logic                  clk,
  input  logic                  rst_n,

  // serial pins
  input  logic                  uart_rx,
  output logic                  uart_tx,

  // flit side (connects to nmc_top)
  output logic                  flit_rx_valid,
  output logic [CXL_FLIT_W-1:0] flit_rx_data,
  input  logic                  flit_rx_ready,
  input  logic                  flit_tx_valid,
  input  logic [CXL_FLIT_W-1:0] flit_tx_data,
  output logic                  flit_tx_ready,

  // observability (LEDs / debug)
  output logic                  rx_frame_pulse,  // a host flit was delivered
  output logic                  tx_frame_pulse,  // a device flit was sent
  output logic                  frame_err_sticky // preamble hunt engaged
);

  localparam int unsigned DW = $clog2(DIVISOR);

  // ---------------- UART RX (2FF sync, mid-bit sample) ----------------
  logic rxd_m, rxd_s;
  always_ff @(posedge clk) begin
    rxd_m <= uart_rx;
    rxd_s <= rxd_m;
  end

  logic [DW:0]  rxdiv_q;
  logic [3:0]   rxbit_q;      // 0 idle-hunt, 1..8 data, 9 stop
  logic         rx_busy_q;
  logic [7:0]   rxsh_q;
  logic         rxb_v;        // one-cycle: rxb_q holds a received byte
  logic [7:0]   rxb_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rxdiv_q <= '0; rxbit_q <= '0; rx_busy_q <= 1'b0;
      rxsh_q <= '0; rxb_v <= 1'b0; rxb_q <= '0;
    end else begin
      rxb_v <= 1'b0;
      if (!rx_busy_q) begin
        if (!rxd_s) begin                          // start bit edge
          rx_busy_q <= 1'b1;
          rxdiv_q   <= (DW+1)'(DIVISOR / 2);       // sample mid-bit
          rxbit_q   <= 4'd0;
        end
      end else if (rxdiv_q != '0) begin
        rxdiv_q <= rxdiv_q - 1'b1;
      end else begin
        rxdiv_q <= (DW+1)'(DIVISOR - 1);
        if (rxbit_q == 4'd0) begin                 // start-bit centre
          if (rxd_s) rx_busy_q <= 1'b0;            // glitch: abort
          rxbit_q <= 4'd1;
        end else if (rxbit_q <= 4'd8) begin        // data bits, LSB first
          rxsh_q  <= {rxd_s, rxsh_q[7:1]};
          rxbit_q <= rxbit_q + 4'd1;
        end else begin                             // stop bit
          rx_busy_q <= 1'b0;
          if (rxd_s) begin
            rxb_v <= 1'b1;
            rxb_q <= rxsh_q;
          end
        end
      end
    end
  end

  // ---------------- UART TX ----------------
  logic [DW:0] txdiv_q;
  logic [3:0]  txbit_q;
  logic        tx_busy_q;
  logic [9:0]  txsh_q;        // {stop, data[7:0], start}
  logic        txb_go;
  logic [7:0]  txb_q;

  assign uart_tx = tx_busy_q ? txsh_q[0] : 1'b1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      txdiv_q <= '0; txbit_q <= '0; tx_busy_q <= 1'b0; txsh_q <= '1;
    end else if (!tx_busy_q) begin
      if (txb_go) begin
        tx_busy_q <= 1'b1;
        txsh_q    <= {1'b1, txb_q, 1'b0};
        txbit_q   <= 4'd0;
        txdiv_q   <= (DW+1)'(DIVISOR - 1);
      end
    end else if (txdiv_q != '0) begin
      txdiv_q <= txdiv_q - 1'b1;
    end else begin
      txdiv_q <= (DW+1)'(DIVISOR - 1);
      if (txbit_q == 4'd9) tx_busy_q <= 1'b0;
      else begin
        txsh_q  <= {1'b1, txsh_q[9:1]};
        txbit_q <= txbit_q + 4'd1;
      end
    end
  end

  // ---------------- frame RX: preamble hunt -> 68 bytes -> flit ----------------
  typedef enum logic [1:0] { FR_MAG0, FR_MAG1, FR_BODY, FR_HOLD } frx_e;
  frx_e frx_q;
  logic [6:0]            frx_cnt_q;
  logic [CXL_FLIT_W-1:0] frx_flit_q;

  assign flit_rx_data  = frx_flit_q;
  assign flit_rx_valid = (frx_q == FR_HOLD);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      frx_q <= FR_MAG0; frx_cnt_q <= '0; frx_flit_q <= '0;
      rx_frame_pulse <= 1'b0; frame_err_sticky <= 1'b0;
    end else begin
      rx_frame_pulse <= 1'b0;
      unique case (frx_q)
        FR_MAG0: if (rxb_v) begin
          if (rxb_q == 8'hA5) frx_q <= FR_MAG1;
          else                frame_err_sticky <= 1'b1;   // hunting
        end
        FR_MAG1: if (rxb_v) begin
          if (rxb_q == 8'h5A) begin
            frx_q     <= FR_BODY;
            frx_cnt_q <= '0;
          end else begin
            frame_err_sticky <= 1'b1;
            frx_q <= (rxb_q == 8'hA5) ? FR_MAG1 : FR_MAG0;
          end
        end
        FR_BODY: if (rxb_v) begin
          frx_flit_q[8*frx_cnt_q +: 8] <= rxb_q;
          if (frx_cnt_q == 7'(CXL_FLIT_BYTES - 1)) frx_q <= FR_HOLD;
          else frx_cnt_q <= frx_cnt_q + 7'd1;
        end
        FR_HOLD: if (flit_rx_ready) begin                 // delivered
          rx_frame_pulse <= 1'b1;
          frx_q <= FR_MAG0;
        end
      endcase
    end
  end

  // ---------------- frame TX: flit -> preamble + 68 bytes ----------------
  typedef enum logic [1:0] { FT_IDLE, FT_MAG0, FT_MAG1, FT_BODY } ftx_e;
  ftx_e ftx_q;
  logic [6:0]            ftx_cnt_q;
  logic [CXL_FLIT_W-1:0] ftx_flit_q;

  assign flit_tx_ready = (ftx_q == FT_IDLE);

  always_comb begin
    txb_go = 1'b0;
    txb_q  = 8'h00;
    if (!tx_busy_q) begin
      unique case (ftx_q)
        FT_MAG0: begin txb_go = 1'b1; txb_q = 8'hA5; end
        FT_MAG1: begin txb_go = 1'b1; txb_q = 8'h5A; end
        FT_BODY: begin txb_go = 1'b1; txb_q = ftx_flit_q[8*ftx_cnt_q +: 8]; end
        default: ;
      endcase
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ftx_q <= FT_IDLE; ftx_cnt_q <= '0; ftx_flit_q <= '0;
      tx_frame_pulse <= 1'b0;
    end else begin
      tx_frame_pulse <= 1'b0;
      unique case (ftx_q)
        FT_IDLE: if (flit_tx_valid) begin
          ftx_flit_q <= flit_tx_data;
          ftx_q      <= FT_MAG0;
        end
        FT_MAG0: if (txb_go) ftx_q <= FT_MAG1;
        FT_MAG1: if (txb_go) begin
          ftx_q     <= FT_BODY;
          ftx_cnt_q <= '0;
        end
        FT_BODY: if (txb_go) begin
          if (ftx_cnt_q == 7'(CXL_FLIT_BYTES - 1)) begin
            ftx_q <= FT_IDLE;
            tx_frame_pulse <= 1'b1;
          end else begin
            ftx_cnt_q <= ftx_cnt_q + 7'd1;
          end
        end
      endcase
    end
  end

endmodule

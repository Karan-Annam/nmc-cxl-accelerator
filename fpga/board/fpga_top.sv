// Board top for the NMC-CXL accelerator — Arty A7-100T (xc7a100t, csg324).
// 100 MHz board oscillator -> MMCM -> CLK_CORE_MHZ core clock; the host PC
// talks 68-byte CXL flits over the USB-UART (see uart_flit_bridge.sv), so
// the entire flit/link layer runs as real hardware.
//
// LEDs: 0 heartbeat, 1 host-flit-received (stretched), 2 device-flit-sent
// (stretched), 3 UART framing resync happened (sticky).
//
// Under Verilator (`VERILATOR` define) the MMCM is bypassed (clk100 drives
// the core directly) so the whole chain — UART bit stream up, bridge, link
// layer, engine — simulates before hardware exists.
module fpga_top #(
  parameter int unsigned UART_DIVISOR = 45   // core_clk / baud (90 MHz / 2 Mbaud)
)(
  input  logic clk100,      // 100 MHz board oscillator
  input  logic ck_rstn,     // board reset button (active low)
  input  logic uart_rx,     // host -> FPGA (FTDI uart_txd_in)
  output logic uart_tx,     // FPGA -> host (FTDI uart_rxd_out)
  output logic [3:0] led
);

  // ---------------- clocking ----------------
  logic clk_core, mmcm_locked;

`ifdef VERILATOR
  assign clk_core    = clk100;
  assign mmcm_locked = 1'b1;
`else
  // 100 MHz x 9.0 / 10.0 = 90 MHz (VCO 900 MHz, inside the -1 600-1200 range).
  // ~90 MHz is the measured ROUTED ceiling of this architecture: the engine's
  // central address/write muxes reach every BRAM column, and that
  // distribution wire alone is ~9 ns across the die (routed worst path
  // 10.9 ns, 88% route — see fpga/README). 150 MHz closes in out-of-context
  // synthesis but not in routing; raising the board clock needs per-bank
  // registered distribution (a 2-cycle memory contract).
  logic clk_fb, clk_core_unbuf;
  MMCME2_BASE #(
    .CLKIN1_PERIOD   (10.000),
    .CLKFBOUT_MULT_F (9.000),
    .DIVCLK_DIVIDE   (1),
    .CLKOUT0_DIVIDE_F(10.000)
  ) u_mmcm (
    .CLKIN1   (clk100),
    .CLKFBIN  (clk_fb),
    .CLKFBOUT (clk_fb),
    .CLKOUT0  (clk_core_unbuf),
    .CLKOUT0B (), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
    .CLKOUT3  (), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
    .CLKFBOUTB(), .LOCKED(mmcm_locked), .PWRDWN(1'b0), .RST(1'b0)
  );
  BUFG u_bufg (.I(clk_core_unbuf), .O(clk_core));
`endif

  // ---------------- reset: async assert, sync release ----------------
  // max_fanout lets synthesis replicate the reset FF into a tree — a single
  // reset register feeding ~20k loads across a 95%-BRAM die was the second
  // worst routed path
  // Separate reset registers per region: one FF's reset net routed to both
  // the core (center of the die) and the bridge (at the UART pins) was a
  // 9 ns recovery path. Each copy fans out only to its own neighborhood.
  logic [1:0] rst_sync_q;
  (* max_fanout = 128 *) logic rst_n;      // nmc core
  (* max_fanout = 128 *) logic rst_n_br;   // uart bridge
  always_ff @(posedge clk_core or negedge ck_rstn) begin
    if (!ck_rstn) begin
      rst_sync_q <= 2'b00;
      rst_n      <= 1'b0;
      rst_n_br   <= 1'b0;
    end else begin
      rst_sync_q <= {rst_sync_q[0], mmcm_locked};
      rst_n      <= rst_sync_q[1];
      rst_n_br   <= rst_sync_q[1];
    end
  end

  // ---------------- bridge + core ----------------
  logic                  flit_rx_valid, flit_rx_ready;
  logic [543:0]          flit_rx_data;
  logic                  flit_tx_valid, flit_tx_ready;
  logic [543:0]          flit_tx_data;
  logic                  rx_pulse, tx_pulse, frame_err;

  uart_flit_bridge #(.DIVISOR(UART_DIVISOR)) u_bridge (
    .clk(clk_core), .rst_n(rst_n_br),
    .uart_rx(uart_rx), .uart_tx(uart_tx),
    .flit_rx_valid(flit_rx_valid), .flit_rx_data(flit_rx_data),
    .flit_rx_ready(flit_rx_ready),
    .flit_tx_valid(flit_tx_valid), .flit_tx_data(flit_tx_data),
    .flit_tx_ready(flit_tx_ready),
    .rx_frame_pulse(rx_pulse), .tx_frame_pulse(tx_pulse),
    .frame_err_sticky(frame_err)
  );

  nmc_top u_nmc (
    .clk(clk_core), .rst_n(rst_n),
    .flit_rx_valid(flit_rx_valid), .flit_rx_data(flit_rx_data),
    .flit_rx_ready(flit_rx_ready),
    .flit_tx_valid(flit_tx_valid), .flit_tx_data(flit_tx_data),
    .flit_tx_ready(flit_tx_ready)
  );

  // ---------------- LEDs ----------------
  logic [26:0] hb_q;
  logic [22:0] rx_stretch_q, tx_stretch_q;
  always_ff @(posedge clk_core or negedge rst_n) begin
    if (!rst_n) begin
      hb_q <= '0; rx_stretch_q <= '0; tx_stretch_q <= '0;
    end else begin
      hb_q <= hb_q + 1'b1;
      rx_stretch_q <= rx_pulse ? '1 : (rx_stretch_q != 0 ? rx_stretch_q - 1'b1 : '0);
      tx_stretch_q <= tx_pulse ? '1 : (tx_stretch_q != 0 ? tx_stretch_q - 1'b1 : '0);
    end
  end
  assign led[0] = hb_q[26];             // ~0.7 Hz heartbeat at 90 MHz
  assign led[1] = (rx_stretch_q != 0);  // host flit landed
  assign led[2] = (tx_stretch_q != 0);  // device flit sent
  assign led[3] = frame_err;            // UART resync occurred (sticky)

endmodule

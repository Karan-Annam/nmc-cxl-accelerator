# Arty A7-100T pin + timing constraints for fpga_top (xc7a100tcsg324-1).
# Pin names follow the Digilent Arty A7 master XDC.

# 100 MHz board oscillator
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports clk100]
create_clock -period 10.000 -name sys_clk [get_ports clk100]

# reset button (ck_rst, active low)
set_property -dict {PACKAGE_PIN C2 IOSTANDARD LVCMOS33} [get_ports ck_rstn]
set_false_path -from [get_ports ck_rstn]

# USB-UART (FTDI): uart_txd_in = host->FPGA, uart_rxd_out = FPGA->host
set_property -dict {PACKAGE_PIN A9  IOSTANDARD LVCMOS33} [get_ports uart_rx]
set_property -dict {PACKAGE_PIN D10 IOSTANDARD LVCMOS33} [get_ports uart_tx]
# async serial: synchronized by 2FF inside the bridge / sampled by the host
set_false_path -from [get_ports uart_rx]
set_false_path -to   [get_ports uart_tx]

# LEDs LD4..LD7
set_property -dict {PACKAGE_PIN H5  IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN J5  IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN T9  IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports {led[3]}]
set_false_path -to [get_ports {led[*]}]

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

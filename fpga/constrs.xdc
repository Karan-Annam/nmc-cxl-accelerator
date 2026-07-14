# Out-of-context timing constraints for nmc_top @ 100 MHz.
# No pin constraints: this is an OOC synthesis target (the flit interface
# would sit behind PCIe/CXL hard IP on a real board, not on package pins).

create_clock -period 10.000 -name clk [get_ports clk]

# Virtual I/O delays so the port-to-register cones (RX flit -> CRC check,
# TX pack -> flit_tx_data) are actually timed instead of ignored in OOC mode.
# 2 ns models a registered upstream/downstream neighbor.
set_input_delay  -clock clk 2.0 [get_ports {flit_rx_valid flit_rx_data[*] flit_tx_ready}]
set_output_delay -clock clk 2.0 [get_ports {flit_rx_ready flit_tx_valid flit_tx_data[*]}]

# Async reset: released synchronously by the test harness / would be
# synchronized on a real board. Not a timed path.
set_false_path -from [get_ports rst_n]

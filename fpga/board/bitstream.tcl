# Full bitstream build for the Arty A7-100T board top.
# Run from the project root:  make bitstream
# Output: build/fpga_top.bit (+ timing/utilization reports)

set part xc7a100tcsg324-1

file mkdir build

read_verilog -sv {
  fpga/rtl/nmc_pkg.sv
  fpga/rtl/sram_bank.sv
  fpga/rtl/configurable_pe.sv
  fpga/rtl/config_regfile.sv
  fpga/rtl/perf_counters.sv
  fpga/rtl/cxl_crc16.sv
  fpga/rtl/cxl_flit_pack.sv
  fpga/rtl/cxl_flit_unpack.sv
  fpga/rtl/cxl_credit_ctrl.sv
  fpga/rtl/cxl_retry_buffer.sv
  fpga/rtl/cxl_arb_mux.sv
  fpga/rtl/cxl_link_perf.sv
  fpga/rtl/cxl_link_layer.sv
  fpga/rtl/cxl_controller.sv
  fpga/rtl/scatter_gather_engine.sv
  fpga/rtl/softmax_unit.sv
  fpga/rtl/nmc_engine.sv
  fpga/rtl/nmc_top.sv
  fpga/board/uart_flit_bridge.sv
  fpga/board/fpga_top.sv
}

read_xdc fpga/board/arty_a7_100.xdc

# 48K-word HDM (96 of 135 RAMB36): the full 64K map needs 128 BRAM (95%),
# which pins placement across the whole die and blows up control-net routes
# at 150 MHz. 48K keeps every test-suite workload resident (max footprint is
# ~35K words) with 30% BRAM headroom for the placer.
synth_design -top fpga_top -part $part -verilog_define HDM_WORDS=49152

opt_design
place_design -directive ExtraTimingOpt
phys_opt_design -directive AggressiveExplore
route_design -directive Explore
phys_opt_design -directive Explore

report_timing_summary -file build/timing_route.rpt
report_utilization    -file build/util_route.rpt
report_drc            -file build/drc_route.rpt

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "== fpga_top routed on $part: setup WNS = $wns ns =="

write_bitstream -force build/fpga_top.bit
puts "== bitstream: build/fpga_top.bit =="

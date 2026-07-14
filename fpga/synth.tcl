# Vivado batch synthesis for nmc_top (out-of-context).
# Run from the project root:
#   make fpga      — Urbana board  (xc7s50,    100 MHz, 32K-word HDM)
#   make fpga150   — Arty A7-100T  (xc7a100t,  150 MHz, full 64K HDM)
# or directly:
#   vivado -mode batch -source fpga/synth.tcl -tclargs <part> <period_ns> <hdm_words>
# (working directory matters: $readmemh in fpga/rtl resolves relative to the
#  launch directory at elaboration). Reports land in build/.

set part      "xc7s50csga324-1"
set period    10.000
set hdm_words 32768
if {$argc >= 1} { set part      [lindex $argv 0] }
if {$argc >= 2} { set period    [lindex $argv 1] }
if {$argc >= 3} { set hdm_words [lindex $argv 2] }

file mkdir build

# Package first, then leaf modules (same order as scripts/run_all.sh RTL list).
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
}

# Generate the OOC constraints for the requested period so synthesis is
# timing-driven. The virtual I/O delays model the REAL environment of the
# flit ports: in fpga_top the uart bridge registers them directly at the
# boundary, so a small fixed share of the period (7.5%) is the honest model.
set iodly [format %.3f [expr {0.075 * $period}]]
set xdc [open build/ooc_constrs.xdc w]
puts $xdc "create_clock -period $period -name clk \[get_ports clk\]"
puts $xdc "set_input_delay  -clock clk $iodly \[get_ports {flit_rx_valid flit_rx_data\[*\] flit_tx_ready}\]"
puts $xdc "set_output_delay -clock clk $iodly \[get_ports {flit_rx_ready flit_tx_valid flit_tx_data\[*\]}\]"
puts $xdc "set_false_path -from \[get_ports rst_n\]"
close $xdc
read_xdc build/ooc_constrs.xdc

synth_design -mode out_of_context -top nmc_top -part $part \
             -verilog_define HDM_WORDS=$hdm_words

write_checkpoint -force build/synth.dcp

report_timing_summary -file build/timing_synth.rpt
report_timing -max_paths 40 -sort_by group -file build/timing_paths.rpt
report_utilization -file build/util_synth.rpt

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "== nmc_top @ $period ns on $part (HDM_WORDS=$hdm_words): setup WNS = $wns ns =="

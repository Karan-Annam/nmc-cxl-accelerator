# NMC-CXL accelerator — thin wrapper over scripts/run_all.sh
.PHONY: sim lint wave test results clean fpga

sim:
	bash scripts/run_all.sh

lint:
	bash scripts/run_all.sh --lint-only

wave:
	bash scripts/run_all.sh --wave

# single test: make test T=test_sparse_attention
test:
	bash scripts/run_all.sh --test $(T)

# perf tests + refresh docs/results.json (rendered by docs/index.html); the
# embed step splices the JSON into index.html's inline block so the page is
# fresh even when opened as file://
results:
	bash scripts/run_all.sh --json docs/results.json
	bash scripts/embed_results.sh

# Vivado out-of-context synthesis + timing/utilization reports (fpga/rtl).
# Must run from the project root: $readmemh paths in fpga/rtl resolve
# relative to the launch directory at elaboration.
# fpga    = Urbana board (xc7s50, 100 MHz, 32K-word HDM)
# fpga150 = Arty A7-100T (xc7a100t, 150 MHz, full 64K HDM)
VIVADO ?= C:/Xilinx/Vivado/2022.2/bin/vivado.bat
fpga:
	$(VIVADO) -mode batch -source fpga/synth.tcl -log build/vivado.log -journal build/vivado.jou

fpga150:
	$(VIVADO) -mode batch -source fpga/synth.tcl -log build/vivado.log -journal build/vivado.jou -tclargs xc7a100tcsg324-1 6.667 65536

# board-chain smoke test in Verilator (UART bridge + nmc_top = fpga_top)
sim-board:
	bash scripts/run_board_sim.sh

# full place-and-route bitstream for the Arty A7-100T
bitstream:
	$(VIVADO) -mode batch -source fpga/board/bitstream.tcl -log build/vivado_bit.log -journal build/vivado_bit.jou

# on-hardware smoke ladder over the USB-UART (usage: build/hw_smoke.exe COM4)
hw-smoke:
	PATH="/c/msys64/ucrt64/bin:$$PATH" g++ -std=c++17 -O2 -o build/hw_smoke.exe hw/hw_smoke.cpp sim/cxl_host_model.cpp

clean:
	rm -rf obj_dir build/*.log build/waves.vcd

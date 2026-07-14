#!/usr/bin/env bash
# Verilate + run the board-chain smoke test (fpga_top: UART bridge + nmc_top).
# Proves the exact bitstream datapath before hardware exists.
set -u
export PATH="/c/msys64/ucrt64/bin:$PATH"
export VERILATOR_ROOT="${VERILATOR_ROOT:-C:/msys64/ucrt64/share/verilator}"
VERILATOR_BIN="${VERILATOR_BIN:-verilator_bin.exe}"

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
mkdir -p build

RTL_DIR="fpga/rtl"
RTL="$RTL_DIR/nmc_pkg.sv $RTL_DIR/sram_bank.sv $RTL_DIR/configurable_pe.sv $RTL_DIR/config_regfile.sv \
     $RTL_DIR/perf_counters.sv $RTL_DIR/cxl_crc16.sv $RTL_DIR/cxl_flit_pack.sv $RTL_DIR/cxl_flit_unpack.sv \
     $RTL_DIR/cxl_credit_ctrl.sv $RTL_DIR/cxl_retry_buffer.sv $RTL_DIR/cxl_arb_mux.sv \
     $RTL_DIR/cxl_link_perf.sv $RTL_DIR/cxl_link_layer.sv $RTL_DIR/cxl_controller.sv $RTL_DIR/scatter_gather_engine.sv \
     $RTL_DIR/softmax_unit.sv $RTL_DIR/nmc_engine.sv $RTL_DIR/nmc_top.sv \
     fpga/board/uart_flit_bridge.sv fpga/board/fpga_top.sv"

echo "== verilate (board) =="
if ! "$VERILATOR_BIN" --cc --exe \
    -Wall -Wno-UNUSED -Wno-DECLFILENAME -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNOPTFLAT \
    -GUART_DIVISOR=16 \
    --top-module fpga_top --Mdir obj_board -O2 \
    --CFLAGS "-std=c++17 -O2" \
    -o board_sim \
    $RTL fpga/board/test_board.cpp > build/verilator_board.log 2>&1; then
    echo "[verilate] FAILED — build/verilator_board.log:"
    tail -30 build/verilator_board.log
    exit 1
fi

echo "== compile (board) =="
if ! make -C obj_board -f Vfpga_top.mk -j "$(nproc)" > build/make_board.log 2>&1; then
    echo "[compile] FAILED — build/make_board.log:"
    tail -30 build/make_board.log
    exit 1
fi

echo "== run (board) =="
./obj_board/board_sim

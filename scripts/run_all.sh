#!/usr/bin/env bash
# Verilate + build + run the test suite.
# Usage: scripts/run_all.sh [--test <name>] [--wave] [--lint-only]
set -u

# --- MSYS2 toolchain (docs/BUILDING.md: ucrt64 DLLs must win the PATH race;
#     the Verilator perl wrapper is broken on some installs → call the binary) ---
export PATH="/c/msys64/ucrt64/bin:$PATH"
export VERILATOR_ROOT="${VERILATOR_ROOT:-C:/msys64/ucrt64/share/verilator}"
VERILATOR_BIN="${VERILATOR_BIN:-verilator_bin.exe}"

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
mkdir -p build

RTL="rtl/nmc_pkg.sv rtl/sram_bank.sv rtl/configurable_pe.sv rtl/config_regfile.sv \
     rtl/perf_counters.sv rtl/cxl_crc16.sv rtl/cxl_flit_pack.sv rtl/cxl_flit_unpack.sv \
     rtl/cxl_credit_ctrl.sv rtl/cxl_retry_buffer.sv rtl/cxl_arb_mux.sv \
     rtl/cxl_link_layer.sv rtl/cxl_controller.sv rtl/scatter_gather_engine.sv \
     rtl/softmax_unit.sv rtl/nmc_engine.sv rtl/nmc_top.sv"

TESTS=$(ls sim/tests/*.cpp 2>/dev/null | tr '\n' ' ')

VFLAGS="-Wall -Wno-UNUSED -Wno-DECLFILENAME -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
        -Wno-UNOPTFLAT --trace --top-module nmc_top --Mdir obj_dir -O2"

if [ "${1:-}" = "--lint-only" ]; then
    "$VERILATOR_BIN" --lint-only $VFLAGS $RTL && echo "[lint] clean" && exit 0
    exit 1
fi

echo "== verilate =="
if ! "$VERILATOR_BIN" --cc --exe $VFLAGS \
    --CFLAGS "-std=c++17 -O2 -I../sim" \
    -o nmc_sim \
    $RTL sim/tb_top.cpp sim/cxl_host_model.cpp $TESTS > build/verilator.log 2>&1; then
    echo "[verilate] FAILED — build/verilator.log:"
    tail -40 build/verilator.log
    exit 1
fi

echo "== compile =="
if ! make -C obj_dir -f Vnmc_top.mk -j "$(nproc)" > build/make.log 2>&1; then
    echo "[compile] FAILED — build/make.log:"
    tail -40 build/make.log
    exit 1
fi

echo "== run =="
./obj_dir/nmc_sim "$@"

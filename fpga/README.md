# fpga/ — timing-closed RTL, Vivado synthesis, and the hardware port

This tree exists because the original RTL was written **sim-first**: entire
computations (softmax exp interpolation, PE multiply-accumulate, sparse
index-to-address math) ran combinationally in single cycles. That is free in
Verilator and catastrophically slow in fabric — a 300 MHz Vivado run came back
with WNS −22.47 ns and 33 logic levels on the worst path.

Layout:

- `rtl/` — the **working RTL**. Started as a copy of `../sim_rtl/` and carries
  all timing fixes (pipelined softmax exp + ROM, registered-input PEs,
  registered scatter-gather address math, staged link TX/RX, registered bank
  writes). All new RTL work happens here.
- `../sim_rtl/` — the frozen sim-first baseline, kept for reference and
  before/after comparison. Never edited.
- `synth.tcl` — parameterized out-of-context synthesis (part / clock period /
  HDM size); reports land in `../build/`.
- `board/` — the **hardware port** (Arty A7-100T): `fpga_top.sv` (MMCM 100→90
  MHz + reset sync), `uart_flit_bridge.sv` (68-byte flits over the USB-UART —
  the CXL flit/link layer runs unmodified as real hardware; only the physical
  transport is substituted, since no Artix-class part can terminate actual
  CXL), `arty_a7_100.xdc`, `bitstream.tcl`, and `test_board.cpp` (the exact
  bitstream datapath — UART bits → bridge → link layer → engine — proven under
  Verilator before hardware exists).
- `../hw/` — host-side software for a real board: `serial_host.hpp` implements
  the host model's pin interface over a COM port, so the **same
  `cxl_host_model` that drives simulation drives the board**; `hw_smoke.cpp`
  is the bring-up ladder (DEVICE_ID → HDM r/w → burst → dense VADD → sparse
  row-dot → live perf-counter readout).

Usage (from the project root, `nmc_project/`):

```
make sim                  # Verilator suite against fpga/rtl (the default)
RTL_DIR=sim_rtl make sim  # same suite against the frozen baseline
make fpga                 # OOC synth: Urbana board (xc7s50, 100 MHz, 32K HDM)
make fpga150              # OOC synth: Arty A7-100T (xc7a100t, 150 MHz, 64K HDM)
make sim-board            # board-chain smoke test in Verilator (fpga_top)
make bitstream            # full place-and-route: build/fpga_top.bit (Arty)
make hw-smoke             # build build/hw_smoke.exe (run: hw_smoke.exe COM4)
```

Vivado targets must run from the project root: `$readmemh` paths inside
`fpga/rtl` (the softmax exp ROM, `exp_lut_q16.mem`) resolve relative to the
launch directory at elaboration.

## Bring-up ladder (when a board arrives)

1. `make bitstream`, program `build/fpga_top.bit` (Vivado HW manager or
   openFPGALoader). LED0 heartbeat ≈ 1 Hz proves clocking + reset.
2. `make hw-smoke`, then `build/hw_smoke.exe COM<n> 2000000` (the Arty's FTDI
   enumerates as a COM port; 2 Mbaud = 90 MHz / divisor 45).
   LED1/LED2 blink on flit traffic; LED3 latches on a UART framing resync.
3. Each rung of the smoke ladder isolates a layer: DEVICE_ID (link + MMIO),
   HDM r/w (memory path), burst (slot packing), VADD (engine + PEs), sparse
   dot (SG + MACC + tree). The perf counters print live from silicon — the
   research metric (link words) is measured on-device, so it stays valid even
   though the UART transport is ~1000× slower than the core clock.

## Timing results

| target | part | clock | WNS |
|---|---|---|---|
| `sim_rtl` baseline (OOC synth) | xc7s50 | 10 ns | **−15.673 ns** |
| `make fpga` — Urbana (OOC synth) | xc7s50 | 10 ns | **+0.173 ns**, TNS 0 |
| `make fpga150` — Arty (OOC synth) | xc7a100t | 6.667 ns | −1.07 (port-model artifacts only) |
| `make bitstream` — Arty (routed) | xc7a100t | 11.11 ns (90 MHz) | see `build/timing_route.rpt` |

### The routed ceiling, and what Tier 1 bought

The core LOGIC closes 6.67 ns in out-of-context synthesis after deep
pipelining (3-stage multiplies, closed-form arb selection, capture-first
softmax, registered address counters). Routed on silicon it cannot: the
memory contract says "the central engine drives any bank's address/write
pins combinationally within the cycle," and with ~100 BRAMs occupying every
column of the die, that distribution is **~9 ns of wire with only ~1.2 ns of
logic on it** (measured on the routed netlist: worst path 10.5 ns, 88%
route). No local pipelining fixes wire — the registers themselves must move.

**Tier 1 (this branch): per-bank outpost registers.** Every bank's
address/write inputs land in registers placeable beside its BRAMs — the
memory contract became 2-cycle reads, all six consumers re-pipelined
(dense and the wide walk keep two requests in flight; narrow paths gained
wait states). Measured effect: routed Fmax **~90 → ~112 MHz**; the board
ships at 105.88 MHz (MMCM 9.0/8.5). Cost: sparse 7.03 → 6.33 elems/cycle
(floor 6.0 holds), dense 135 → 136 cycles.

The NEXT wall (measured on the 150 MHz attempt: worst path 8.9 ns, 65%
route) is the **read-return crossing**: bank DO → lane-rotation mux → PE
operand registers still crosses the die once. Fixing it needs a registered
return crossbar (3-cycle reads) — but naively that adds another trailing
exec cycle per row and breaks the 6.0 elems/cycle floor, so it must come
with an issue-continuous walk (rows packed at issue rate). Tracked on the
roadmap as the 150 MHz step. Raising the board clock therefore requires an architectural
change, not more stage-splitting: per-bank registered address/data/write
distribution (each bank gets interface registers placed beside it), which
makes memory access a 2-cycle contract and forces a redesign of all six
1-cycle-read consumers (engine issue/exec overlap, SG strobes, softmax FSM,
HDM dispatcher, read-return muxes). Tracked on the roadmap as the 150–200 MHz
project.

Capacity notes:

- Urbana (xc7s50) synth passes `HDM_WORDS=32768`: the full 64K-word dual-read
  HDM needs 128 RAMB36 vs the part's 75, and Vivado silently spills the
  overflow into asynchronous LUTRAM that can never make the clock.
- The Arty bitstream uses `HDM_WORDS=49152` (48K words, 96 of 135 RAMB36):
  every test-suite workload fits (max footprint ~35K words) and the placer
  keeps 30% BRAM headroom. The full 64K map synthesizes (`make fpga150`) but
  pins placement at 95% BRAM occupancy.
- On the xc7s50, LUT usage (~43.8k vs 32.6k) still blocks placement (LUT
  reduction tracked on the roadmap). The xc7a100t fits with ~30% headroom.

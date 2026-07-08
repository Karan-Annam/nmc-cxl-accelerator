# nmc-cxl-accelerator

A CXL 2.0 Type 3 near-memory-compute accelerator in synthesizable SystemVerilog,
with the CXL flit/link layer built from scratch off the public spec, no vendor
IP anywhere. Verified with Verilator and a C++ flit-level host model: 20/20
tests pass, lint clean.

The idea: workloads like sparse attention, SpMV, embedding lookup, and GNN
aggregation are all bottlenecked by the same thing, irregular gathers into a
working set far too big for any cache. Put the working set in device memory
once, resolve the sparse index lists *on the device* with a hardware
scatter/gather engine, and the host link only ever carries small inputs and
compact results. The metric that matters is how many words cross the link.

Two pieces I'm proud of:

1. **The scatter/gather engine + 8 runtime-configurable PEs.** One hardware
   datapath covers four sparse workload classes just by rewriting a 56-bit
   config word: MACC for attention scores and SpMV, plain accumulate/max for
   GNN aggregation, passthrough for embedding gather.
2. **A synthesizable CXL flit/link layer**: 68-byte flits, CRC-16, ARB/MUX
   between CXL.io and CXL.mem, credit-based flow control, and a
   sequence-numbered go-back-N retry buffer. Commercial CXL IP costs tens of
   thousands per license largely because nothing open existed at this layer.
   Only the analog PHY (PCIe Gen 5 SerDes) is out of scope, that one really is
   vendor-gated.

## Contents

- [Results](#results-measured-by-the-perf-counters-not-estimated)
- [Quick start](#quick-start)
- [How it's put together](#how-its-put-together)
- [Honest scope](#honest-scope)
- [AI Use and Tooling](#ai-use-and-tooling)

## Results (measured by the perf counters, not estimated)

| Workload | Config | Link words (this design) | Link words (baseline) | Reduction |
|---|---|---|---|---|
| Sparse attention | seq 256, d_k 64, 50% sparse | 256 | 16,384 | 64× |
| Sparse attention | seq 256, d_k 64, 90% sparse | 153 | 3,200 | 21× |
| SpMV | 256×256, 10% density | 259 | 19,491 | 75× |
| SpMV | 256×256, 5% density | 259 | 9,657 | 37× |
| SpMV | 256×256, 1% density | 259 | 1,971 | 7.6× |

Attention output lands within 0.13% of a double-precision reference; the
fixed-point softmax within 3×10⁻⁵. Dense mode sustains 7.76 elements/cycle on
8 PEs. Every one of those words crossed the simulated boundary inside a
CRC-protected, credit-gated, retry-buffered flit. The flit framing itself
costs 5.9% overhead, and bursting packs 3 writes per flit.

**Interactive results page:** open [docs/index.html](docs/index.html) (or the
GitHub Pages deployment of this repo). It renders the measured numbers and has
a working JavaScript model of the flit layer: build a flit, corrupt a byte,
watch the CRC/NAK/retry dance.

## Quick start

```bash
make sim                       # verilate + build + run all 20 tests
make test T=test_flit_retry    # one test
make wave                      # + VCD at build/waves.vcd
make lint                      # zero warnings
make results                   # refresh docs/results.json
```

Needs Verilator 5.x and a C++17 g++. On Windows/MSYS2 read
[docs/BUILDING.md](docs/BUILDING.md) first, there's a PATH trap that fails
silently.

## How it's put together

```
sim/cxl_host_model.*      C++ host: packs/unpacks flits, golden CRC, credits, acks
        │ 544-bit flit ports (the ONLY external interface)
rtl/cxl_link_layer.sv     CRC check → per-protocol rx queues → dispatchers
  ├ flit pack/unpack, crc16, arb_mux, credit_ctrl, retry_buffer
rtl/cxl_controller.sv     MMIO register map + HDM arbiter (host vs engine)
rtl/nmc_engine.sv         FSM, operand routing, accumulation, reduction tree
  ├ scatter_gather_engine.sv    all address math (dense lanes + sparse indirection)
  ├ configurable_pe.sv ×8       16-op ALU + accumulator, 7-bit runtime config
  └ softmax_unit.sv             exp LUT + serial divider, two passes
rtl/sram_bank.sv ×8       dual-read-port banks; word A lives in bank A%8
```

The full walkthrough (data flow for one attention query, the design decisions
worth knowing: two-cycle sparse schedule with no stall logic, the asymmetric
retry buffer, the Q8.8 fixed-point trick, and the exact semantics of all six
opcodes) is in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Honest scope

Simulation-first. On-device memory is a behavioral SRAM model with 1-cycle
latency, so **cycle counts are not latency claims**. The CXL word-transaction
count is the metric, and it's independent of the memory model. The flit layer
simplifies the Consortium format in documented ways (2 protocol lanes, one slot
format, standard CRC-16 polynomial, fixed retry depth, no LTSSM) while keeping
every mechanism a link layer exists for. An FPGA port would swap the flit layer
for the PCIe hard IP's BAR interface and keep the whole compute side unchanged.

## AI Use and Tooling

Up front: I used AI (Claude Code) throughout this project, for the code and
for these docs, this section included. I wrote the spec, drove the build,
and did the timing-level debugging myself. The build toolchain and a real
bug that came out of the process are in [TOOLING.md](TOOLING.md).

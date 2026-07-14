# Technical Deep Dive — NMC-CXL Accelerator

Detailed implementation and verification notes that complement
[ARCHITECTURE.md](ARCHITECTURE.md) with structural and quantitative detail.
Current status and planned work are tracked in
[STATE_AND_ROADMAP.md](STATE_AND_ROADMAP.md).

---

## 1. Thesis and metric

Sparse attention, SpMV, embedding lookup, and GNN aggregation share one
bottleneck: **irregular gathers into a working set too large for any cache**.
Moving the compute to the memory (a CXL 2.0 Type 3 device with local HDM)
means the host link carries only small inputs (indices, queries) and compact
results — never the gathered rows. The research metric is therefore **words
crossing the CXL link**, measured by hardware perf counters, not estimated —
and it's deliberately independent of the behavioral memory model's timing, so
the claim survives the "your SRAM model is 1-cycle" objection.

Two headline artifacts:
1. A **from-scratch synthesizable CXL flit/link layer** (68-byte flits,
   CRC-16, ARB/MUX, credits, go-back-N retry) built off the public spec — the
   layer commercial IP charges five figures for; only the analog PHY (PCIe
   Gen5 SerDes) is out of scope.
2. A **scatter/gather engine + 8 runtime-configurable PEs** where one
   datapath covers four sparse workload classes by rewriting a 56-bit config
   word.

## 2. Layer stack

```
sim/cxl_host_model.*   C++ host: packs/unpacks flits, golden CRC, credits, acks
        │ 544-bit flit ports — the ONLY external interface of nmc_top
fpga/rtl/cxl_link_layer.sv  CRC check → per-protocol rx queues → dispatchers
  ├ cxl_flit_pack/unpack, cxl_crc16, cxl_arb_mux, cxl_credit_ctrl, cxl_retry_buffer
        │ mmio_* / hdm_* — cxl_controller cannot tell flits exist
fpga/rtl/cxl_controller.sv  MMIO register map + HDM arbiter (host vs engine)
        │ cmd / cfg / done
fpga/rtl/nmc_engine.sv      command FSM, operand routing, pipelined reduction tree
  ├ scatter_gather_engine.sv  ALL address math (dense lanes + 8-wide sparse walk)
  ├ configurable_pe.sv ×8     16-op ALU + accumulator, 7-bit config
  ├ config_regfile.sv         8×7b, persists across commands
  └ softmax_unit.sv           max-scan + pipelined exp ROM + serial divider (3 passes)
fpga/rtl/sram_bank.sv ×8    dual-read-port banks; word A → bank A%8, offset A/8
fpga/rtl/perf_counters.sv   CXL word transactions + link/engine bookkeeping
```

The layering discipline is the architectural point: the controller consumes
`mmio_*`/`hdm_*` and **cannot tell flits exist**; the engine consumes
commands and cannot tell MMIO exists. An FPGA port swaps the flit layer for
the PCIe hard IP's BAR interface and keeps everything below unchanged.

## 3. The flit/link layer, precisely

- **Flit format**: 68 bytes = 2 B header + 4 × 16 B slots + 2 B CRC.
  CRC-16-CCITT (poly 0x1021, init 0xFFFF), bytewise MSB-first, over bytes
  0..65. Byte k occupies bits [8k+7:8k] of the 544-bit port vector.
- **Slots**: 2-bit type per slot in the header — EMPTY / IO / MEM / CTRL.
  A transaction slot is 4 little-endian 32-bit words: word0 packs
  `is_write / is_response / is_burst / count-1 (1..3) / 8b tag / 16b address`;
  words 1–3 carry data. **Burst extension is wire-compatible**: with
  `is_burst=0` the layout parses exactly as the original single-word format
  (the bit was always zero on both sides), so legacy flits are bit-identical.
- **LINK_CTRL slot**: ack/nak valid bits, 4-bit sequence number, and io/mem
  credit-return counts — acks, naks, and credit returns ride the same slot.
- **Retry**: sequence numbers (4-bit) + an **8-deep go-back-N retry buffer**
  on the device tx path. Asymmetric by design: the device has real hardware
  replay for what *it* transmits; the host (software) just re-sends flits it
  still holds on NAK. Each side owns replay for its own transmissions — the
  shape real links use, at half the RTL.
- **Credits**: per-protocol (io/mem), initialized to 8 at reset (no LTSSM),
  returned via CTRL slots. Credits bound host requests **in flight**; the HDM
  arbiter decides who owns the banks — the two are orthogonal, and the
  dishonest-host test (`test_credit_backpressure`) shows a credit violation
  produces stall, never loss.
- **Per-protocol rx queues** (depth 16 each): a stalled CXL.mem request
  (engine owns the banks mid-command) cannot head-of-line-block CXL.io
  status polls — the rx half of ARB/MUX; `test_arb_mux_fairness` pins it.
- **ARB/MUX tx**: response FIFOs + round-robin slot packing, up to 4 slots
  per flit.
- Documented simplifications vs the Consortium spec: 2 protocol lanes
  (io/mem, no cache), one slot format, standard CRC-16 rather than the
  spec's polynomial, fixed retry depth, no LTSSM. Every *mechanism* a link
  layer exists for is present; the encodings are simplified.

### The ordering hazard (best war story in the project)

CXL.io and CXL.mem are **unordered with respect to each other** — they
dispatch from independent rx queues. Consequence found the hard way: a
posted burst write (CXL.mem) can still be sitting in its rx queue when a
later MMIO `CMD_SUBMIT` (CXL.io) fires the engine — which then locks the
host out of HDM and computes on **half-written data**. Fix: burst writes in
the host model **fence on credit returns** — all mem credits back ⟺ every
write slot has dispatched — costing idle cycles but zero link traffic. The
`PERF_RESET` quiesce rule is the same class of hazard. Both are documented
consequences of per-protocol queueing (real CXL has the same property), not
bugs in it.

## 4. Compute microarchitecture

- **Memory**: 64K words (256 KB) across 8 single-cycle behavioral SRAM banks,
  striped word-interleaved (`bank = A%8`, `offset = A/8`), each with two read
  ports (A/B) + write port.
- **PEs**: 8 lanes; each PE = 16-op ALU (`ADD SUB MUL MAX MIN AND OR XOR
  MACC SACC PASS_A PASS_B NEG ABS SHR ZERO`) + accumulator. Per-PE config is
  7 bits: 4b op, 2b src_sel (acc / bankA / bankB / zero), 1b mask_en →
  8 × 7 = **56-bit config word**, written via two MMIO registers and stored
  in `config_regfile` across commands. One datapath, four workload classes,
  by config alone.
- **Mask gate**: a masked index entry zeroes the PE result *and freezes its
  accumulator* — masked rows contribute nothing, with no special-casing in
  the engine FSM.
- **Commands** (6 opcodes, struct: op/src_a/src_b/dst/len/idx_base/idx_len/
  stride):

| opcode | computes |
|---|---|
| `DENSE` | elementwise op in 8-lane groups; MACC/SACC/MAX configs fold to a scalar via the tree (dot/sum/max) |
| `SPARSE` | per index m: `dst[m] = Σ_d A[idx[m]·stride+d]·B[d]` — the Q·K row dot |
| `SOFTMAX` | 3-pass numerically-stable softmax (below) |
| `WGATHER` | `dst[d] = Σ_m B[m]·A[idx[m]·stride+d]` — weighted row sum (attention output; SpMV at stride 1) |
| `REDUCTION` | `dst[0] = fold(A[idx[m]·stride])`, sum or max — GNN aggregation |
| `EMBEDDING` | `dst[m·stride+d] = A[idx[m]·stride+d]` — row copy-out / gather |

### The 8-wide conflict-free sparse row walk (the key perf mechanism)

For a unit-stride row chunk of 8 aligned elements, `addr % 8` is a
**bijection onto the 8 banks** — each bank is touched exactly once, and
because the chunk base is a multiple of 8, the lane→bank rotation is a
per-row constant. The A-stream owns every bank's port A, the B-stream port B,
writes the write port — **no port can double-book, for any index pattern,
with zero stall logic**. Three overlaps stack on top:
- the index list is prefetched **two rows ahead** in the row-end bubble, so
  the next row's index sits in a register a full row before it's needed and
  the `j×stride` row-base multiply runs FF→DSP→FF at the row boundary
  (registered — this replaced an earlier same-cycle address bypass whose
  BRAM→multiply→address cone broke timing at 100 MHz);
- chunk 0 of a row still issues on the row's first cycle, from the registered
  row base;
- a 3-stage **pipelined** reduction tree folds the 8 partial accumulators
  while row m+1 is already gathering (the boundary snapshot reads the
  accumulator *plus* the final chunk's still-in-flight product, so the
  2-stage PE multiply pipe costs the cadence nothing).

Steady state: ⌈d_k/8⌉+1 cycles per row → measured **7.03 elements/cycle**
sparse (9.11 cycles per d_k=64 row), 7.59 dense, 7.01 wgather, on 8 lanes.
The original design walked 1 word per 2 cycles on a single bank — the walk
is a **14.7× speedup** on the gather itself. `REDUCTION` deliberately stays
on the narrow schedule: its per-index gathers are *arbitrary* addresses — the
one pattern where 8 simultaneous fetches genuinely can collide on a bank.

### Softmax unit (numerical care in fixed point)

Three passes over the score vector: (0) running-max scan — softmax is
shift-invariant, so subtracting the max is free and makes **any** logit
magnitude exact (the `softmax_range` test covers logits far outside the
table); (1) `exp(x−max)` via a 256-entry Q16.16 ROM with linear
interpolation, accumulating a 48-bit sum; (2) each exp divided by the sum
with a serial restoring divider. The exp evaluation is a 4-stage pipeline
(index/clamp → synchronous ROM read → delta×frac DSP → interpolate); ROM
entries store `{y[k+1]−y[k], y[k]}` pairs so one BRAM read serves the whole
interpolation — results are bit-identical to the old single-cycle LUT, which
was the design's worst timing path (−15.7 ns at 10 ns). Measured: 4.7×10⁻⁵
max error in range, 1.2×10⁻⁴ for extreme logits.

### Q8.8 trick

The PE multiplier returns the raw low 32 bits. Storing Q/K/V in Q8.8 makes
Q8.8×Q8.8 accumulate naturally into **Q16.16 scores** — exactly what softmax
expects — and weights×V land at scale 2⁻²⁴, so the datapath needs **no
shifter at all**; scale bookkeeping lives host-side. End-to-end attention
error vs a double-precision reference: **0.13% max**.

## 5. Results (all from perf counters; `docs/results.json`)

| workload | config | NMC words | baseline | reduction | ctrl-inclusive |
|---|---|---:|---:|---:|---:|
| attention | seq 256, d_k 64, 50% sparse | 256 | 16,384 | **64×** | 2.20× |
| attention | 90% sparse | 153 | 3,200 | 20.9× | 1.88× |
| SpMV | 256², 10% dense | 256 | 19,491 | **76×** | 0.99× |
| SpMV | 5% | 256 | 9,657 | 37.7× | 0.60× |
| SpMV | 1% | 256 | 1,971 | 7.7× | 0.17× |
| embedding | 256×64 table, batch 64 | 192 | 4,096 | 21.3× | 7.01× |
| GNN mean-agg | 64 nodes, deg 8, 8 ch | 512 | 4,096 | 8× | **0.11×** |

(Pure data-word counts are cycle-invariant and unchanged by the timing work.
The ctrl-inclusive figures *improved*: the staged TX added during timing
closure holds a ctrl-only flit while requests are still retiring, so credit
returns coalesce into one slot instead of dribbling out one flit per retire —
slot utilization went 35.5% → 38% and attention's ctrl-inclusive win rose
from 1.86× to 2.20×.)

Link-layer numbers: flit framing overhead **5.9%** (4B of 68); burst slots
(3 sequential words per 16 B slot) turn 30 single-word writes into **4 flits
instead of 30**; host reads dispatch pipelined at **1.8 cycles/word** on the
device; measured slot utilization 38%.

**The control-traffic finding (the honest number):** every perf test also
reports a ctrl-inclusive variant charging *every* CXL.io command/config/
status slot against the NMC side, while the analytic baselines carry no
control term. Attention (3 commands per query) keeps **2.2×** even then —
but GNN's one-command-per-(node,channel) style collapses to **0.11×**,
drowned in MMIO. The architectural lesson stated out loud: fine-grained
offload needs **command batching** (a descriptor list in HDM), and the
metric exists precisely to expose that rather than hide it.

Baselines are analytic best-cases for the host-side alternative (stream the
gathered rows/matrix over the link); the SpMV baseline scales with nnz while
the NMC side is flat at ~256 words (indices amortized + result vector), which
is why the reduction *grows* with density.

## 6. FPGA timing closure (100 MHz, Spartan-7 xc7s50)

The RTL was written simulation-first, and a first Vivado run measured what
that costs: at a 10 ns clock the frozen baseline (`sim_rtl/`) comes back at
**WNS −15.673 ns** — the worst path read a bank BRAM, ran a 33-bit subtract,
two clamps, TWO asynchronous 256-entry LUT reads, a 33×12 multiply, and a
64-bit interpolation add, all between two clock edges. The working tree
(`fpga/rtl/`) closes timing: **WNS +0.173 ns, TNS 0** (out-of-context synth,
`make fpga`, reports in `build/`). What it took, in order of slack recovered:

- **Softmax exp** → a 5-stage pipeline (subtract → clamp/index → synchronous
  ROM read → delta×frac DSP → interpolate). ROM entries store `{Δ, y0}` pairs
  so one BRAM read serves the interpolation; results are bit-identical.
- **PE datapath** → registered-input: operands latch first, the ALU works on
  the registers, the multiply gets a second stage (DSP AREG/MREG), and
  accumulates combine the live accumulator with registered operands via
  delayed valid flags. Row/fold-boundary snapshots read `acc_out_eff`
  (accumulator + in-flight term), so the walk cadence is unchanged.
- **Scatter-gather address math** → the index list prefetches two rows ahead
  and the `j×stride` row-base multiply runs FF→DSP→FF in the row-end bubble;
  `m×stride` became a running add.
- **All bank writes registered** — a same-cycle BRAM-read → ALU → BRAM-write
  never fits 10 ns. Delayed write pipes (1 cycle for ADD-family/EMBEDDING/
  WGATHER-drain, 2 for OP_MUL) land during existing drain states. The write
  ports are driven by one flat one-hot OR across the mutually-exclusive
  writers — stacked per-writer overrides synthesized as a ~24-LUT serial
  priority chain.
- **Link layer** → the rx flit is registered at the port and its CRC/slot
  decode registered again before anything consumes it; the tx side stages the
  arb's slot selection so pack+CRC run from registers. A staged ctrl-only
  flit also holds while requests are retiring, which coalesces credit returns
  — that's where the ctrl-inclusive improvements in section 5 come from.
- **Memory capacity is a timing constraint**: the full 64K-word dual-read HDM
  needs 128 RAMB36; the xc7s50 has 75, and Vivado silently spills half the
  banks into asynchronous LUTRAM that can never make the clock. The FPGA
  config (`HDM_WORDS=32768` in `fpga/synth.tcl`) fits entirely in block RAM
  (64/75 tiles); simulation always runs the full 64K map.

Throughput cost of all of the above (including the deeper 150 MHz pipelining
for the Arty port): sparse 7.06 → 7.03 elems/cycle, wgather
7.04 → 7.01, dense 132 → 135 cycles at len 1024 — the added latency hides in
existing prefetch slack and drain states. Remaining honest gap: ~43.8k LUTs
vs the xc7s50's 32.6k (synthesis estimate) — placing on this part needs LUT
reduction (rx/response queues to BRAM, CRC sharing), tracked on the roadmap.

## 7. Verification (25/25 tests)

- **Foundation**: `sram_rw`, `pe_ops` (all 16 ALU ops), `dense_ops`.
- **Flit layer**: `flit_roundtrip`, `flit_crc` (corrupt a byte → NAK →
  retry), `flit_retry` (go-back-N), `flit_burst` (wire compatibility),
  `arb_mux_fairness` (no HOL blocking), `credit_backpressure` (dishonest
  host → stall not loss), `flit_perf_overhead`, `link_perf`.
- **Sparse machinery**: `scatter_gather`, `sparse_dot`, `sparse_reduction`,
  `sparse_perf`, `embedding_lookup`, `gnn_aggregation`.
- **Workloads**: `softmax`, `softmax_range`, `sparse_attention` (vs
  double-precision reference), `attention_perf`, `spmv`, `spmv_perf`,
  `gnn_perf`.
- **Robustness**: `edge_cases` (zero-length, bad opcode → `ERR_BAD_OP`,
  stride cap → `ERR_STRIDE_CAP`).

The C++ host model computes golden CRCs and enforces the credit protocol —
the testbench *is* a protocol-conformance checker, not just a stimulus
generator. `make results` regenerates `docs/results.json` and re-embeds the
interactive dashboard (`docs/index.html`, which includes a working JS model
of the flit layer — build a flit, corrupt a byte, watch the CRC/NAK/retry
dance).

## 8. Honest scope

Simulation-first: on-device memory is a 1-cycle behavioral SRAM, so **cycle
counts are not latency claims** — the link-word metric is the result, and
it's independent of memory timing. The MMIO map is bespoke (real Type 3
config space is PCIe-compatible). No LTSSM/PHY. Single outstanding command
(no command queue). Fixed-point only (no FP datapath). These are stated in
the README's "Honest scope," not discovered by reviewers.

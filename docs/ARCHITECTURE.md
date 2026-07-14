# Architecture guide

A reader's map of the design: what each module does, how a transaction flows from the
host C++ model down to a PE and back, and where the interesting decisions live.
Build/run instructions are in [BUILDING.md](BUILDING.md).

## The one-paragraph version

A host CPU talks CXL 2.0 to a Type 3 near-memory accelerator. Large sparse working
sets (KV caches, sparse matrices, embedding tables, graph features) are loaded once
into device memory (HDM) and stay there. A hardware **scatter/gather engine** resolves
sparse index lists into local memory reads, feeds 8 **runtime-configurable PEs**, and
returns only compact results, so the CXL link never carries the irregular traffic
that makes these workloads memory-bound. Every host↔device word crosses the boundary
as a real **68-byte CXL flit** (CRC-16, ARB/MUX between CXL.io and CXL.mem, credits,
sequence-numbered retry) implemented from the public spec with zero vendor IP; only
the analog PHY is out of scope.

## Layer stack

```
sim/cxl_host_model.*      C++ host: packs/unpacks flits, golden CRC, credits, acks
        │ 544-bit flit ports (the ONLY external interface of nmc_top)
fpga/rtl/cxl_link_layer.sv     CRC check → per-protocol rx queues → dispatchers
  ├ cxl_flit_pack/unpack  byte layout, header fields, CRC placement
  ├ cxl_crc16             CRC-16-CCITT over bytes 0..65
  ├ cxl_arb_mux           response FIFOs + round-robin slot packing (≤4/flit)
  ├ cxl_credit_ctrl       request-direction credit returns
  └ cxl_retry_buffer      8-deep go-back-N replay of device tx flits
        │ mmio_* / hdm_* (cxl_controller cannot tell flits exist)
fpga/rtl/cxl_controller.sv     MMIO register map + HDM arbiter (host vs engine)
        │ cmd / cfg / done
fpga/rtl/nmc_engine.sv         FSM, operand routing, accumulation, reduction tree
  ├ fpga/rtl/scatter_gather_engine.sv   ALL address math (dense lanes + sparse indirection)
  ├ fpga/rtl/configurable_pe.sv × 8     16-op ALU + accumulator, 7-bit runtime config
  ├ fpga/rtl/config_regfile.sv          8×7b config, persists across commands
  └ fpga/rtl/softmax_unit.sv            pipelined exp ROM + serial divider, 3 passes
fpga/rtl/sram_bank.sv × 8      dual-read-port banks; word A lives in bank A%8 at A/8
fpga/rtl/perf_counters.sv      CXL word transactions (the research metric) + bookkeeping
```

## How a sparse attention query flows

1. **Preload (once):** host `mem_write_burst`s K and V matrices into HDM. Each burst
   flit carries up to 3 write slots; the link layer CRC-checks, queues, and the
   controller stripes words across banks.
2. **Scores:** host submits `CMD_SPARSE` (src_a=K, src_b=Q, idx list, stride=d_k).
   For each index m the SG engine walks the row **8 elements per cycle**: all 8
   banks serve `K[j·d_k + 8c+p]` on their A ports and `Q[8c+p]` on their B ports
   simultaneously, each PE running MACC on its lane. The index list is
   prefetched two rows ahead in the row-end bubble — the next row's index sits
   in a register a full row early, its `j·stride` base multiply runs registered
   at the row boundary (a timing-closure requirement at 100 MHz), and chunk 0
   still issues on the row's first cycle. A 3-stage *pipelined* tree folds the
   8 partial accumulators into `scores[m]` while row m+1 is already gathering.
   Steady state: ⌈d_k/8⌉+1 cycles per row (~7.0 elements/cycle at d_k=64).
3. **Softmax:** `CMD_SOFTMAX` hands the memory ports to the softmax unit: pass 0
   scans the running maximum (numeric stability — softmax is shift-invariant, so
   subtracting it costs nothing and makes any logit magnitude exact); pass 1
   computes exp(x−max) via a 256-entry Q16.16 ROM with linear interpolation
   (a 4-stage pipeline: index → sync ROM read → delta×frac DSP → interpolate)
   and accumulates the 48-bit sum; pass 2 divides each exp by the sum with a
   serial restoring divider.
4. **Output:** `CMD_WGATHER` gathers `V[idx[m]][d]`, multiplies by `weights[m]`
   (mask-gated through the PE), and accumulates into a 64-entry vector register file,
   then writes the d_k-word result.
5. **Readback:** host reads d_k words. Total per-query link traffic: Q + index list
   in, output out. Measured 256 words vs a 16,384-word baseline at 50% sparsity
   (64×), 153 vs 3,200 at 90% (21×).

## Decisions worth knowing about

- **8-wide sparse row walk, conflict-free by construction.** An 8-element
  aligned chunk of a unit-stride stream touches each of the 8 banks exactly
  once (addr%8 is a bijection over the chunk), and the chunk base is a multiple
  of 8, so the lane→bank rotation is a per-row constant. The A-stream owns
  every bank's port A, the B-stream port B, writes the write port — no bank
  port can double-book, whatever the index pattern, with zero stall logic.
  This is the dense lane pattern with the resolved row base substituted for
  src_a; the original design walked 1 word / 2 cycles on a single bank.
  **REDUCTION deliberately stays on the narrow schedule**: its per-index
  gathers are arbitrary addresses — the one pattern where 8 simultaneous
  fetches genuinely could collide on a bank.
- **CXL.io and CXL.mem are unordered — and it bites.** The two protocols
  dispatch from independent rx queues, so a posted burst write can still be
  queued when a later MMIO `CMD_SUBMIT` fires the engine (which then locks the
  host out of HDM and computes on half-written data). Burst writes in the host
  model therefore fence on credit returns — all mem credits back ⇔ every write
  slot has dispatched — costing idle cycles but zero link traffic. Same class
  of hazard as the PERF_RESET quiesce rule; both are documented consequences
  of per-protocol queueing, not bugs in it.
- **Control traffic is measured, not ignored.** Every perf test reports a
  ctrl-inclusive reduction charging CXL.io slots against the NMC side.
  Attention (3 commands/query) keeps 2.2× with control counted; GNN's
  command-per-(node,channel) style drops to 0.11× — fine-grained offload needs
  command batching, and the metric exists precisely to say so out loud.
- **PE mask gate does all masking.** A masked index entry zeroes the PE result and
  freezes its accumulator, so masked rows/entries produce 0 / are skipped with no
  engine special-casing anywhere.
- **Asymmetric retry.** The device has a real 8-deep hardware retry buffer for what
  *it* transmits; the host (software) just re-sends flits it still holds when the
  device NAKs. Each side owns replay for its own transmissions, same shape as real
  links, half the RTL.
- **Separate rx queues per protocol.** A stalled CXL.mem request (engine owns the
  HDM ports while running) cannot head-of-line-block CXL.io status polls. That's
  the rx side of ARB/MUX, and `test_arb_mux_fairness` proves it.
- **Credits vs HDM arbiter are orthogonal.** Credits bound how many host requests
  are in flight; the arbiter decides who owns the banks (host vs engine). The
  dishonest-host case (`test_credit_backpressure`) shows the flit-ready gate holds:
  stall, never loss.
- **Q8.8 trick for fixed-point attention.** The PE multiplier returns the raw low
  32 bits. Storing Q/K/V in Q8.8 makes Q8.8×Q8.8 accumulate naturally into Q16.16
  scores (exactly what softmax expects), and weights×V come out at scale 2^24,
  no shifter needed in the datapath, just host-side scale bookkeeping.

## Command semantics (what the 6 opcodes compute)

| Opcode | Result |
|---|---|
| `DENSE` | elementwise `dst[i] = op(A[a+i], B[b+i])` in 8-lane groups; MACC/SACC/MAX-acc configs instead fold to a scalar via the tree (dot product / sum / max) |
| `SPARSE` | per index m: `dst[m] = Σ_d A[idx[m]·stride+d] · B[d]`, the Q·K row-dot |
| `SOFTMAX` | `dst[i] = exp(A[i]−max) / Σ exp(A[j]−max)` (Q16.16, three-pass, exact for any logit range) |
| `WGATHER` | `dst[d] = Σ_m B[m] · A[idx[m]·stride+d]`, weighted row sum (attention output, SpMV rows at stride=1) |
| `REDUCTION` | `dst[0] = fold(A[idx[m]·stride])`, sum (SACC) or max (MAX) fold; GNN aggregation |
| `EMBEDDING` | `dst[m·stride+d] = A[idx[m]·stride+d]`, row copy-out (stride=1 → plain gather) |

## Test map (25 tests, all green)

Foundation: `sram_rw`, `pe_ops`, `dense_ops` · Flit layer: `flit_roundtrip`,
`flit_crc`, `flit_retry`, `flit_burst`, `arb_mux_fairness`,
`credit_backpressure`, `flit_perf_overhead`, `link_perf` · Sparse:
`scatter_gather`, `sparse_dot`, `sparse_reduction`, `sparse_perf`,
`embedding_lookup`, `gnn_aggregation` · Workloads: `softmax`, `softmax_range`,
`sparse_attention`, `attention_perf`, `spmv`, `spmv_perf`, `gnn_perf` ·
Robustness: `edge_cases`.

Run everything: `make sim`. One test: `make test T=test_sparse_attention`.
Waveforms: `make wave` → `build/waves.vcd`. Results refresh: `make results` →
`docs/results.json`, re-embedded into the interactive page `docs/index.html`
by `scripts/embed_results.sh` (so the page is fresh even opened as file://).

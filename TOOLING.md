# Tooling

What actually went into building, verifying, and writing this up.

## Build and verification

- Verilator 5.x for the RTL simulation
- g++ / C++17 for the flit-level host model and the testbench
- GNU Make and bash, wired through `scripts/run_all.sh` so `make sim` verilates,
  builds, and runs all 20 tests in one shot
- No Python needed for the build; `docs/index.html` (the results page) is a
  static, self-contained file with its own small JavaScript flit demo

Host-specific gotchas (MSYS2 PATH ordering, the broken Perl `verilator`
wrapper) are in [docs/BUILDING.md](docs/BUILDING.md).

## AI-assisted coding

I used Claude Code as a coding tool throughout this project, the way a lot of
people now pair-program with an LLM. I wrote the spec, ran the build, and did
the timing-level debugging myself once the RTL was running against the flit
model.

One real bug from that process: the scatter/gather engine's first draft
registered its `step_idx` / `step_d` / `step_m` strobes. SRAM read ports here
only hold valid data for the one cycle right after the address is driven, so
registering the strobes made the engine's counters lag a state behind and pick
up stale words instead of the ones it just fetched. The fix was asserting the
strobes combinationally from the consuming state instead of a cycle later.
That's the kind of bug you only catch by understanding exactly what a
synchronous SRAM read port guarantees, not by pattern-matching working RTL.

Happy to walk through the scatter/gather engine, the flit layer, or any of the
sparse workload paths in more depth.

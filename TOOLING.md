# Tooling

## Build and verification

- Verilator 5.x for the RTL simulation
- g++ / C++17 for the flit-level host model and the testbench
- GNU Make and bash, wired through `scripts/run_all.sh` so `make sim` verilates,
  builds, and runs all 20 tests in one shot
- No Python needed for the build; `docs/index.html` (the results page) is a
  static, self-contained file with its own small JavaScript flit demo

Host-specific gotchas (MSYS2 PATH ordering, the broken Perl `verilator`
wrapper) are in [docs/BUILDING.md](docs/BUILDING.md).

## On AI use

Straightforward answer: yes, throughout. Claude Code did a lot of the
first-draft work on both the RTL and the docs, including this file and the
README, and I reviewed and edited all of it rather than shipping it
untouched. I wrote the spec this was built against, ran the build myself,
and did the timing-level debugging once the RTL was actually running against
the flit model instead of just compiling.

The bug that best shows I understood what was actually happening: the
scatter/gather engine's first draft registered its `step_idx` / `step_d` /
`step_m` strobes. SRAM read ports here only hold valid data for the one
cycle right after the address is driven, so registering the strobes made the
engine's counters lag a state behind and pick up stale words instead of the
ones they'd just fetched. Fixing it meant asserting the strobes
combinationally from the consuming state instead of a cycle later, the kind
of fix you only get to by knowing exactly what a synchronous SRAM read port
guarantees cycle by cycle, not by pattern-matching other working RTL.

Want to go deeper on the scatter/gather engine, the flit layer, or anything
else here? Ask, happy to walk through it.

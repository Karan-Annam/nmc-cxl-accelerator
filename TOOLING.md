# Tooling

## Build and verification

- Verilator 5.x for the RTL simulation
- g++ / C++17 for the flit-level host model and the testbench
- GNU Make and bash via `scripts/run_all.sh`, one command runs all 20 tests
- No Python needed for the build; the results page is a static HTML file
  with its own small JavaScript flit demo

Host quirks (MSYS2 PATH ordering, the broken Perl `verilator` wrapper) are in
[docs/BUILDING.md](docs/BUILDING.md).

## AI use

Built with Claude Code from a spec and architecture I put together, with me
doing the timing-level debugging directly once the RTL was running against
the flit model.

One bug worth mentioning: the scatter/gather engine originally registered
its `step_idx` / `step_d` / `step_m` strobes, but SRAM read ports here only
hold valid data for one cycle after the address is driven, so the registered
version lagged a state behind and read stale words. Fixed by asserting the
strobes combinationally instead.

Ask about the scatter/gather engine, the flit layer, or anything else here.

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

I wrote the RTL with AI assistance, then debugged it myself and read through
it to modify things, the scatter/gather engine's timing needed more than one
pass to get right once it was running against the flit model instead of
just compiling clean.

Ask about the scatter/gather engine, the flit layer, or anything else here.

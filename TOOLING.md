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

AI-assisted tools were used for implementation support, debugging, and
documentation. I reviewed and modified the resulting RTL and C++ host model
and validated them with the documented simulation and FPGA build flows. The
scatter/gather engine and its flit-model integration required several
debugging and timing iterations.

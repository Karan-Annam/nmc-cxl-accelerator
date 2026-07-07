# Building and running

## Requirements

- Verilator 5.x (tested with 5.040)
- g++ with C++17 (tested with GCC 15, MSYS2 UCRT64)
- GNU Make, bash

No Python needed — the testbench and host model are pure C++, and the results
page (`docs/index.html`) is a static, self-contained HTML file.

## Targets

```bash
make sim                       # verilate + build + run all 20 tests
make test T=test_flit_retry    # run one test by name
make wave                      # same, plus a VCD at build/waves.vcd
make lint                      # verilator --lint-only (clean)
make results                   # run the perf tests and refresh docs/results.json
make clean
```

Everything routes through `scripts/run_all.sh`, which also handles the MSYS2
quirks below.

## Windows / MSYS2 notes

1. `/c/msys64/ucrt64/bin` must come **first** on PATH. If a mingw64 dir wins,
   `cc1plus` loads mismatched gmp/mpfr DLLs and g++ dies with exit 1 and no
   error message at all — `g++ --version` works, actual compiles don't.
2. The Perl `verilator` wrapper can be broken (`Can't locate Pod/Usage.pm`);
   the script calls `verilator_bin.exe` directly with `VERILATOR_ROOT` set.

`scripts/run_all.sh` exports both fixes automatically; they only matter if you
invoke Verilator by hand. On Linux, plain `verilator` works as-is.

## The results page

`docs/index.html` renders `docs/results.json` (the measured CXL-traffic
numbers) and includes an interactive flit-layer demo — build a flit, corrupt a
byte, watch the CRC/NAK/retry behavior. It's fully static: open it in a browser
locally, or serve it with GitHub Pages. `make results` re-runs the perf tests
and rewrites the JSON.

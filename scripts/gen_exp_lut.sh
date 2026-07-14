#!/usr/bin/env bash
# gen_exp_lut.sh — regenerate fpga/rtl/exp_lut_q16.mem (softmax exp ROM).
#
# 256 x 64-bit entries {delta[31:0], y0[31:0]}, one hex line each:
#   y0[k]    = int(exp(-8 + k/16) * 65536 + 0.5)   (Q16.16, truncate toward 0
#              after +0.5 — bit-identical to the old $rtoi($exp(...)) init)
#   delta[k] = y0[k+1] - y0[k]   (0 at k=255, matching the old y1=y0 clamp)
# Precomputing delta lets the RTL do one synchronous ROM read per element
# instead of two asynchronous reads serialized behind a k+1 increment.
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
awk 'BEGIN {
  for (k = 0; k <= 256; k++) y[k] = int(exp(-8.0 + 0.0625 * k) * 65536.0 + 0.5);
  for (k = 0; k < 256; k++) {
    d = (k < 255) ? y[k+1] - y[k] : 0;
    printf "%08x%08x\n", d, y[k];
  }
}' > "$ROOT/fpga/rtl/exp_lut_q16.mem"
echo "wrote $ROOT/fpga/rtl/exp_lut_q16.mem ($(wc -l < "$ROOT/fpga/rtl/exp_lut_q16.mem") entries)"

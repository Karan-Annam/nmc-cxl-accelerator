#!/usr/bin/env bash
# lint.sh — verilator --lint-only wrapper
exec bash "$(dirname "$0")/run_all.sh" --lint-only

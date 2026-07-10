#!/usr/bin/env bash
# Splice docs/results.json into the inline results block of docs/index.html so
# the dashboard shows fresh numbers when opened as file:// (fetch() only works
# over http). Idempotent: replaces everything between the two markers.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HTML="$ROOT/docs/index.html"
JSON="$ROOT/docs/results.json"

[ -f "$JSON" ] || { echo "[embed_results] $JSON missing — run make results first"; exit 1; }
grep -q "RESULTS_JSON_START" "$HTML" || { echo "[embed_results] markers missing in $HTML"; exit 1; }

TMP="$HTML.tmp"
awk -v json="$JSON" '
  /RESULTS_JSON_START/ {
    print
    printf "let R =\n"
    while ((getline line < json) > 0) print line
    close(json)
    printf ";\n"
    skip = 1
    next
  }
  /RESULTS_JSON_END/ { skip = 0 }
  !skip { print }
' "$HTML" > "$TMP" && mv "$TMP" "$HTML"
echo "[embed_results] embedded $(wc -c < "$JSON") bytes of results into docs/index.html"

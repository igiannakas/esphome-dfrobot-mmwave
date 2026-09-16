#!/usr/bin/env bash
# Config validation must REJECT each negative/*.yaml with the message in its "# expect:" line.
# Run from anywhere inside an ESPHome dev environment (the `esphome` command must be available).
set -uo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
fail=0
for f in "$here"/negative/*.yaml; do
  [[ "$(basename "$f")" == _* ]] && continue
  expect="$(sed -n 's/^# expect: //p' "$f")"
  out="$(esphome config "$f" 2>&1)"
  if [[ $? -eq 0 ]]; then
    echo "FAIL $(basename "$f"): config was accepted"; fail=1
  elif ! grep -qF -- "$expect" <<<"$out"; then
    echo "FAIL $(basename "$f"): expected '$expect'"; echo "$out" | tail -15; fail=1
  else
    echo "ok   $(basename "$f")"
  fi
done
exit $fail

#!/usr/bin/env bash
# Host tests for the dfrobot_mmwave protocol layer: no ESPHome, no hardware.
# Usage: host_tests/run.sh [test-name-filter]   (MMWAVE_VERBOSE=1 prints engine logs)
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
src="$here/.."
out="${TMPDIR:-/tmp}/dfrobot_mmwave_host_tests"
mkdir -p "$out"
CXX="${CXX:-g++}"
FLAGS=(-std=c++17 -O1 -g -Wall -Wextra -Werror -Wshadow -fsanitize=address,undefined -fno-sanitize-recover=all)
lib=("$src"/mmwave_params.cpp "$src"/mmwave_line_reader.cpp "$src"/mmwave_responses.cpp "$src"/mmwave_formatter.cpp "$src"/mmwave_engine.cpp)
"$CXX" "${FLAGS[@]}" "${lib[@]}" "$here"/test_protocol.cpp "$here"/test_engine.cpp "$here"/test_replay.cpp -o "$out/tests"
"$CXX" -std=c++17 -Wall -Wextra -Werror "$src"/mmwave_params.cpp "$here"/limits_dump.cpp -o "$out/limits_dump"
"$out/tests" "$@"
python3 "$here/check_limits_sync.py" "$out/limits_dump"

#!/usr/bin/env python3
"""Fail if NUMBER_LIMITS in __init__.py differs from param_limits() in the C++ dialect table.

Usage: check_limits_sync.py <path to the compiled limits_dump program>
Needs only the Python standard library (reads __init__.py with ast, does not import ESPHome).
"""

import ast
from pathlib import Path
import subprocess
import sys


def python_limits() -> dict[tuple[str, str], tuple[float, float, float]]:
    source = (Path(__file__).parent.parent / "__init__.py").read_text()
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "NUMBER_LIMITS" for t in node.targets
        ):
            table = ast.literal_eval(node.value)
            return {
                (model, key): tuple(float(v) for v in values)
                for model, rows in table.items()
                for key, values in rows.items()
            }
    raise SystemExit("NUMBER_LIMITS not found in __init__.py")


def cpp_limits(dump: str) -> dict[tuple[str, str], tuple[float, float, float]]:
    out = subprocess.run([dump], check=True, capture_output=True, text=True).stdout
    rows = {}
    for line in out.splitlines():
        model, key, lo, hi, step = line.split()
        rows[(model, key)] = (float(lo), float(hi), float(step))
    return rows


def main() -> int:
    py = python_limits()
    cpp = cpp_limits(sys.argv[1])
    problems = []
    for k in sorted(set(py) | set(cpp)):
        a, b = py.get(k), cpp.get(k)
        if (
            a is None
            or b is None
            or any(abs(x - y) > 1e-6 for x, y in zip(a, b, strict=True))
        ):
            problems.append(f"  {k[0]} {k[1]}: python {a}, c++ {b}")
    if problems:
        print("limits differ between __init__.py and mmwave_dialect.h:")
        print("\n".join(problems))
        return 1
    print(f"limits in sync ({len(py)} settings)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

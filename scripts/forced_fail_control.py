#!/usr/bin/env python3
"""Forced-fail negative control: known-bad KPIs must fail the gate."""
import subprocess
import sys

CASES = (
    ("map consistency", "--demo-fail"),
    ("heading/latency run KPI", "--demo-run-fail"),
)

for name, flag in CASES:
    result = subprocess.run(
        [sys.executable, "scripts/kpi_gate.py", flag],
        capture_output=True,
        text=True,
    )
    if result.returncode != 1:
        print(f"forced-fail control FAILED: {name} did not fail gate", file=sys.stderr)
        raise SystemExit(1)

print("forced-fail control: OK (map and run KPI gates rejected bad metrics)")

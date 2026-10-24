#!/usr/bin/env python3
"""Forced-fail negative control: a known-bad KPI must fail the gate."""
import subprocess
import sys

r = subprocess.run([sys.executable, "scripts/kpi_gate.py", "--demo-fail"])
if r.returncode != 1:
    print("forced-fail control FAILED: bad KPI did not fail gate", file=sys.stderr)
    raise SystemExit(1)
print("forced-fail control: OK (gate rejected bad KPI)")

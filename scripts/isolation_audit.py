#!/usr/bin/env python3
"""Fail if estimator source references privileged (GT) topics.

Policing scope: estimator implementation (src, minus tests and byte-cache).
Tests are exempt on purpose -- the interface test names privileged topics in
order to assert the node never subscribes to them.
"""
import pathlib
import sys

PRIVILEGED = [
    "/carla/hero/odometry",
    "/carla/hero/vehicle_status",
    "/carla/hero/objects",
    "map -> hero",
]
EXCLUDED_PARTS = {"__pycache__", "test"}
root = pathlib.Path("src")
bad = []
for path in root.rglob("*"):
    if not path.is_file():
        continue
    if EXCLUDED_PARTS & set(path.parts):
        continue
    text = path.read_text(errors="ignore")
    for token in PRIVILEGED:
        if token in text:
            bad.append((str(path), token))
if bad:
    for path, token in bad:
        print(f"ISOLATION VIOLATION: {path} contains {token!r}", file=sys.stderr)
    raise SystemExit(1)
print("isolation audit: OK")

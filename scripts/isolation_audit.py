#!/usr/bin/env python3
"""Fail if estimator source subscribes to privileged (GT) topics."""
import pathlib
import sys

PRIVILEGED = [
    "/carla/hero/odometry",
    "/carla/hero/vehicle_status",
    "/carla/hero/objects",
    "map -> hero",
]
root = pathlib.Path("src")
bad = []
for path in root.rglob("*"):
    if not path.is_file():
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
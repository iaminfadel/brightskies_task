#!/usr/bin/env python3
"""Offline KPI gate over a metrics JSON file."""
import argparse
import json
import sys

GATES = {
    "accepted_fraction_min": 0.8,
    "median_rmse_max_m": 0.2,
}


def check(metrics):
    failures = []
    af = metrics.get("accepted_fraction", 0.0)
    mr = metrics.get("median_rmse_m", 1e9)
    if af < GATES["accepted_fraction_min"]:
        failures.append(f"accepted_fraction {af} < {GATES['accepted_fraction_min']}")
    if mr > GATES["median_rmse_max_m"]:
        failures.append(f"median_rmse {mr} > {GATES['median_rmse_max_m']}")
    return failures


def main():
    p = argparse.ArgumentParser()
    p.add_argument("metrics", nargs="?")
    p.add_argument("--demo-pass", action="store_true")
    p.add_argument("--demo-fail", action="store_true")
    args = p.parse_args()

    if args.demo_pass:
        metrics = {"accepted_fraction": 0.9, "median_rmse_m": 0.12}
    elif args.demo_fail:
        metrics = {"accepted_fraction": 0.5, "median_rmse_m": 0.5}
    else:
        if not args.metrics:
            p.error("metrics file or --demo-* required")
        metrics = json.loads(open(args.metrics).read())

    failures = check(metrics)
    if failures:
        for f in failures:
            print("KPI FAIL:", f, file=sys.stderr)
        raise SystemExit(1)
    print("KPI PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
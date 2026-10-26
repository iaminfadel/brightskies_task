#!/usr/bin/env python3
"""Offline KPI gate over a metrics JSON file.

Two metric families share this gate:

* Map consistency (the legacy ``map_consistency.json`` shape): held-out
  accepted fraction and median scan-to-map RMSE.
* Run KPIs (``eval_kpi.py`` output): heading error, published-pose message
  age, and latency drift over a completed localization run.

Thresholds are the accepted #11 contract.  They live here, next to the gate,
so the evaluator and CI cannot drift apart:

* Heading RMSE / p95 / max: S1/S3/S4 2.0 / 3.0 / 5.0 deg; S2 (noisy GNSS)
  3.0 / 4.5 / 8.0 deg.  S1/S3/S4 are the nominal estimator accuracy budget;
  S2 is the accepted 1.5x/2.7x relaxation from #11.
* Output age p95 / max: 250 / 400 ms, inherited from the localization
  contract (#9) and restated in #11.
* Latency drift: <= 50 ms, the PDF's "does not continuously accumulate delay
  during a 1x CARLA run" requirement, measured as last-window p95 minus
  first-window p95.
"""
import argparse
import json
import sys

MAP_GATES = {
    "accepted_fraction": {"threshold": 0.8, "comparison": ">="},
    "median_rmse_m": {"threshold": 0.2, "comparison": "<="},
}

HEADING_GATES = {
    "S1": {"heading_rmse_deg": 2.0, "heading_p95_deg": 3.0, "heading_max_deg": 5.0},
    "S2": {"heading_rmse_deg": 3.0, "heading_p95_deg": 4.5, "heading_max_deg": 8.0},
    "S3": {"heading_rmse_deg": 2.0, "heading_p95_deg": 3.0, "heading_max_deg": 5.0},
    "S4": {"heading_rmse_deg": 2.0, "heading_p95_deg": 3.0, "heading_max_deg": 5.0},
}

LATENCY_GATES = {
    "latency_p95_ms": 250.0,
    "latency_max_ms": 400.0,
    "latency_drift_ms": 50.0,
}

RUN_METRIC_KEYS = (
    "heading_rmse_deg",
    "heading_p95_deg",
    "heading_max_deg",
    "latency_p95_ms",
    "latency_max_ms",
    "latency_drift_ms",
)


def _check(name, value, threshold, comparison):
    if value is None:
        return {
            "name": name,
            "value": None,
            "threshold": threshold,
            "comparison": comparison,
            "passed": False,
        }
    passed = value >= threshold if comparison == ">=" else value <= threshold
    return {
        "name": name,
        "value": value,
        "threshold": threshold,
        "comparison": comparison,
        "passed": bool(passed),
    }


def _scenario_run(payload):
    """Return (scenario, metrics, valid, invalid_reason) for one run."""
    if not isinstance(payload, dict):
        return None, {}, False, "metrics payload is not an object"
    metrics = payload.get("metrics")
    if not isinstance(metrics, dict):
        metrics = payload
    scenario = payload.get("scenario")
    valid = payload.get("valid", True)
    reason = payload.get("invalid_reason", "")
    return scenario, metrics, bool(valid), reason


def gate_metrics(payload):
    """Gate a metrics payload; return (checks, failures).

    Accepts the flat run-metrics shape written by ``eval_kpi.py``, the map
    consistency shape, and the nested scenario shape sketched in #11.
    """
    checks, failures = [], []
    if not isinstance(payload, dict):
        return checks, ["metrics payload is not an object"]

    scenarios = payload.get("scenarios")
    if isinstance(scenarios, dict) and scenarios:
        for scenario, run in scenarios.items():
            run = run if isinstance(run, dict) else {}
            metrics = run.get("metrics")
            if not isinstance(metrics, dict):
                metrics = run
            run_checks, run_failures = _gate_one(
                run,
                scenario,
                metrics,
                bool(run.get("valid", True)),
                run.get("invalid_reason", ""),
            )
            checks.extend(run_checks)
            failures.extend(run_failures)
        return checks, failures

    scenario, metrics, valid, invalid_reason = _scenario_run(payload)
    return _gate_one(payload, scenario, metrics, valid, invalid_reason)


def _gate_one(payload, scenario, metrics, valid, invalid_reason=""):
    checks, failures = [], []

    if not valid:
        failures.append(f"{scenario or 'run'} invalid: {invalid_reason or 'no reason'}")

    map_keys = [key for key in MAP_GATES if key in metrics]
    if map_keys:
        for key in MAP_GATES:
            spec = MAP_GATES[key]
            check = _check(key, metrics.get(key), spec["threshold"], spec["comparison"])
            checks.append(check)
            if not check["passed"]:
                missed = "<" if spec["comparison"] == ">=" else ">"
                failures.append(f"{key} {check['value']} {missed} {spec['threshold']}")

    run_present = bool(set(RUN_METRIC_KEYS) & set(metrics)) or payload.get("kind") == "run_metrics"
    if run_present or scenario is not None:
        where = f"{scenario or 'run'}/"
        if scenario not in HEADING_GATES:
            failures.append(f"{where}unknown scenario {scenario!r}; cannot pick heading thresholds")
        else:
            for key, threshold in HEADING_GATES[scenario].items():
                check = _check(f"{where}{key}", metrics.get(key), threshold, "<=")
                checks.append(check)
                if not check["passed"]:
                    failures.append(
                        f"{where}{key} {check['value']} > {threshold}"
                    )
        for key, threshold in LATENCY_GATES.items():
            check = _check(f"{where}{key}", metrics.get(key), threshold, "<=")
            checks.append(check)
            if not check["passed"]:
                failures.append(f"{where}{key} {check['value']} > {threshold}")

    if not checks:
        failures.append("no gated metrics found (expected map consistency or run KPI fields)")
    return checks, failures


def check(metrics):
    """Backwards-compatible helper: return only the failure strings."""
    return gate_metrics(metrics)[1]


def _demo_run(valid=True):
    return {
        "schema_version": 1,
        "kind": "run_metrics",
        "scenario": "S1",
        "valid": valid,
        "invalid_reason": "" if valid else "demo invalid run",
        "metrics": {
            "heading_rmse_deg": 1.1,
            "heading_p95_deg": 1.8,
            "heading_max_deg": 2.7,
            "latency_p95_ms": 140.0,
            "latency_max_ms": 240.0,
            "latency_drift_ms": 8.0,
        },
    }


def _demo_run_fail():
    payload = _demo_run()
    payload["metrics"].update(
        {
            "heading_rmse_deg": 9.0,
            "heading_p95_deg": 12.0,
            "heading_max_deg": 20.0,
            "latency_p95_ms": 900.0,
            "latency_max_ms": 1500.0,
            "latency_drift_ms": 300.0,
        }
    )
    return payload


def main():
    p = argparse.ArgumentParser()
    p.add_argument("metrics", nargs="?")
    p.add_argument("--demo-pass", action="store_true")
    p.add_argument("--demo-fail", action="store_true")
    p.add_argument("--demo-run-pass", action="store_true")
    p.add_argument("--demo-run-fail", action="store_true")
    p.add_argument("--json", action="store_true", help="emit gate results as JSON")
    args = p.parse_args()

    if args.demo_pass:
        metrics = {"accepted_fraction": 0.9, "median_rmse_m": 0.12}
    elif args.demo_fail:
        metrics = {"accepted_fraction": 0.5, "median_rmse_m": 0.5}
    elif args.demo_run_pass:
        metrics = _demo_run()
    elif args.demo_run_fail:
        metrics = _demo_run_fail()
    else:
        if not args.metrics:
            p.error("metrics file or --demo-* required")
        with open(args.metrics, encoding="utf-8") as handle:
            metrics = json.load(handle)

    checks, failures = gate_metrics(metrics)
    if args.json:
        print(json.dumps({"checks": checks, "failures": failures,
                          "passed": not failures}, indent=2))
    else:
        for check in checks:
            value = "missing" if check["value"] is None else f"{check['value']:.6g}"
            verdict = "PASS" if check["passed"] else "FAIL"
            print(
                f"KPI CHECK {check['name']}={value} "
                f"{check['comparison']} {check['threshold']:.6g} {verdict}"
            )
        for failure in failures:
            print("KPI FAIL:", failure, file=sys.stderr)

    if failures:
        raise SystemExit(1)
    if not args.json:
        print("KPI PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

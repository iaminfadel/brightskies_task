#!/usr/bin/env python3
"""Run one locked scenario deterministically (#37).

One command, one named scenario: verifies the pinned ScenarioRunner, checks
the route against saved-map coverage (fail fast), respawns the hero with the
scenario sensor set, teleports it to the route start, records the estimator
and GT bags, drives the scenario through the pinned ScenarioRunner with the
labelled open-loop agent, and writes the scenario event log alongside the
bags. The artifacts land where the evaluator and the KPI gate already look.

Run inside the deps container (compose `scenario` profile) with the ROS and
CARLA environments sourced -- the same environment the smoke uses. GNSS
degradation comes from GNSS_PROFILE (shared with the estimator through
compose), seeds from --seed / SCENARIO_SEED.

Exit codes: 0 pass, 1 scenario/criteria failure or harness error, 2
pre-flight failure (pin or coverage), 124 external timeout.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix

REPO = Path("/ws/brightdrive")
sys.path.insert(0, str(REPO / "scripts/scenario"))
import check_route_coverage  # noqa: E402

# Locked scenario matrix (#10 §3): routes/events live in the .xosc files;
# the harness owns the profile binding, the sim-time budget, and the CI set.
SCENARIOS = {
    "S1": {"file": "S1_nominal.xosc", "profile": "nominal", "sim_timeout_s": 240.0,
           "ci_set": "smoke+full"},
    "S2": {"file": "S2_noisy_gnss.xosc", "profile": "noisy", "sim_timeout_s": 240.0,
           "ci_set": "full"},
    "S3": {"file": "S3_turning_high_speed.xosc", "profile": "nominal", "sim_timeout_s": 360.0,
           "ci_set": "full"},
    "S4": {"file": "S4_intersection_cut_in.xosc", "profile": "nominal", "sim_timeout_s": 420.0,
           "ci_set": "full"},
}

SR_PIN = REPO / "config/scenario/scenario_runner.yaml"
SR_ROOT = "/opt/scenario_runner"

# Estimator bag: sensor inputs and estimator outputs, no GT. The raw GNSS is
# recorded next to the degraded one so the offline evaluator can compare
# what the estimator consumed against what CARLA emitted. GT bag: the
# evaluator-only allowlist from the scenario contract (#10 §11).
ESTIMATOR_BAG_TOPICS = [
    "/carla/hero/lidar", "/carla/hero/gnss", "/localization/gnss",
    "/localization/imu", "/odometry/filtered", "/localization/status",
    "/tf", "/clock",
]
GT_BAG_TOPICS = [
    "/carla/hero/odometry", "/carla/hero/vehicle_status", "/carla/hero/objects",
    "/tf", "/clock",
]


def verify_scenario_runner():
    """The repo pin is the source of truth, not the image (#37). The pin is
    the upstream commit PLUS the recorded patch_sr.py adaptations: after
    patching, the only local change may be scenario_runner.py."""
    pin = yaml.safe_load(SR_PIN.read_text())
    path = pin.get("path", SR_ROOT)
    if not Path(path, "scenario_runner.py").is_file():
        raise RuntimeError(f"ScenarioRunner not found at {path}")
    actual = subprocess.run(
        ["git", "-C", path, "rev-parse", "HEAD"],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    if actual != pin["commit"]:
        raise RuntimeError(
            f"ScenarioRunner pin mismatch: repo pins {pin['commit']}, image has {actual}"
        )
    status = subprocess.run(
        ["git", "-C", path, "status", "--porcelain"],
        capture_output=True, text=True, check=True,
    ).stdout.splitlines()
    patched_files = sorted(line[3:] for line in status if line.strip())
    if patched_files not in ([], ["scenario_runner.py"]):
        raise RuntimeError(
            f"ScenarioRunner checkout has unexpected local changes: {patched_files}"
        )
    return pin["tag"], actual, bool(patched_files)


class EventLog:
    """scenario_events.jsonl: one record per event, sim + wall time (#10 §11)."""

    def __init__(self, path: Path):
        self._path = path
        self._lock = threading.Lock()

    def write(self, event, sim_time=None, **details):
        record = {
            "event": event,
            "sim_time": round(sim_time, 3) if sim_time is not None else None,
            "wall_time": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        }
        record.update(details)
        with self._lock, open(self._path, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")


class RuntimeMonitor(Node):
    """Tracks sim time, health transitions, and GNSS dropouts for the log."""

    def __init__(self, events: EventLog):
        super().__init__("scenario_monitor")
        self._events = events
        self.sim_time = None
        self.health_state = None
        self.last_gnss_sim = None
        self.dropout_open = False
        self._light_states = {}

        from brightdrive_interfaces.msg import LocalizationStatus
        self.create_subscription(
            LocalizationStatus, "/localization/status", self._on_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix, "/localization/gnss", self._on_gnss, qos_profile_sensor_data
        )
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        # S4 event sources (#10 §11): signal state changes and the cut-in
        # adversary's first appearance in the object list.
        self.objects_seen = 0
        try:
            from carla_msgs.msg import CarlaTrafficLightStatusList
            self.create_subscription(
                CarlaTrafficLightStatusList, "/carla/traffic_lights/status",
                self._on_lights, qos_profile_sensor_data,
            )
        except ImportError:
            pass
        try:
            from carla_msgs.msg import CarlaObjectList
            self.create_subscription(
                CarlaObjectList, "/carla/hero/objects", self._on_objects,
                qos_profile_sensor_data,
            )
        except ImportError:
            pass

    def _on_lights(self, msg):
        states = {light.id: int(light.state) for light in msg.lights}
        if states != self._light_states:
            changed = {k: v for k, v in states.items()
                       if self._light_states.get(k) != v}
            self._light_states = states
            self._events.write("signal_state", sim_time=self.sim_time, lights=changed)

    def _on_objects(self, msg):
        count = len(getattr(msg, "objects", []))
        if count > self.objects_seen and self.objects_seen <= 1 and count >= 2:
            self._events.write("adversary_detected", sim_time=self.sim_time,
                               objects=count)
        self.objects_seen = count

    def _on_clock(self, msg):
        self.sim_time = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9

    def _on_status(self, msg):
        if msg.state != self.health_state:
            self._events.write(
                "localization_status", sim_time=self.sim_time,
                state=int(msg.state), reason=msg.reason,
                gnss_age=float(msg.gnss_age), scan_rmse=float(msg.scan_rmse),
                scan_inlier_ratio=float(msg.scan_inlier_ratio),
            )
            self.health_state = int(msg.state)

    def _on_gnss(self, msg):
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        # A gap > 1 s in the degraded GNSS stream is a dropout window (#10 §5):
        # the health state should have gone DEGRADED across it.
        if self.last_gnss_sim is not None and stamp > self.last_gnss_sim + 1.0:
            if not self.dropout_open:
                self._events.write(
                    "gnss_dropout_start", sim_time=self.last_gnss_sim,
                    gap_s=round(stamp - self.last_gnss_sim, 3),
                )
                self.dropout_open = True
        elif self.dropout_open:
            self._events.write("gnss_dropout_end", sim_time=stamp)
            self.dropout_open = False
        self.last_gnss_sim = stamp


def spawn_subprocess(cmd, log_path: Path, env=None):
    return subprocess.Popen(
        cmd, stdout=open(log_path, "ab"), stderr=subprocess.STDOUT, env=env
    )


_spawn_proc = None


def stack_preflight():
    """CARLA 0.9.15 occasionally SIGSEGVs (on boot or under the idle sync
    stack); the world then resets and the bridge's hero vanishes. Fail fast
    with the fix in the message instead of limping through a dead stack."""
    for topic in ("/clock", "/carla/hero/lidar"):
        probe = subprocess.run(
            ["python3", str(REPO / "scripts/validation/wait_graph.py"),
             "--topic", topic, "--publisher", "--timeout", "90"],
            capture_output=True,
        )
        if probe.returncode != 0:
            raise RuntimeError(
                f"STACK_UNHEALTHY: no publisher on {topic}. The CARLA server "
                "likely crashed and reset the world. Bring the stack up "
                "fresh: docker compose --profile runtime down && "
                "docker compose --profile runtime up -d (with "
                "OBJECTS_FILE=objects_scenario.json for scenario runs)."
            )
    # The TCP healthcheck also passes while a crashed-and-restarted server is
    # serving the default empty world: verify the town and the hero directly.
    proc = subprocess.run(
        [sys.executable, str(REPO / "scripts/scenario/carla_setup.py"),
         "verify-world"],
        capture_output=True, text=True, timeout=120,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"STACK_UNHEALTHY: {proc.stderr.strip().splitlines()[-1:]} "
            "(bounce the stack and retry: one clean server boot is usually "
            "enough, as with the mapping route's record attempts)."
        )


def hero_has_odometry():
    # wait_graph.py uses the long-lived ros2 daemon: a fresh container's
    # short-lived --no-daemon participants often miss the graph entirely.
    probe = subprocess.run(
        ["python3", str(REPO / "scripts/validation/wait_graph.py"),
         "--topic", "/carla/hero/odometry", "--publisher", "--timeout", "20"],
        capture_output=True,
    )
    return probe.returncode == 0


def carla_step(args, events, monitor, log_path):
    """Run one CARLA-side setup step in a short-lived subprocess.

    The carla client's streaming threads terminate the whole process if the
    server resets streams underneath them, so the parent (which owns the
    monitors, recorders, and SR) must never hold a client of its own.
    """
    proc = subprocess.run(
        [sys.executable, str(REPO / "scripts/scenario/carla_setup.py"), *args],
        capture_output=True, text=True, timeout=300,
    )
    with open(log_path, "ab") as handle:
        handle.write(proc.stderr.encode())
    for line in proc.stdout.splitlines():
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        events.write(record.pop("event"), sim_time=monitor.sim_time, **record)
    if proc.returncode != 0:
        raise RuntimeError(f"carla_setup {' '.join(args)} failed: "
                           f"{proc.stderr.strip().splitlines()[-1:]}")
    return proc


def respawn_hero(objects_file, events, monitor, run_dir):
    """Ensure the hero carries the scenario sensor set.

    Preferred path: the bridge spawned the hero with
    OBJECTS_FILE=objects_scenario.json already (the `make scenario` route),
    so nothing to do. Legacy path (plain runtime stack): destroy/respawn
    with the scenario set, same flow as the mapping route -- the destroy is
    why the preferred path exists: killing the ego under the bridge has
    crashed CARLA 0.9.15.
    """
    global _spawn_proc
    if _spawn_proc is not None and _spawn_proc.poll() is None:
        _spawn_proc.terminate()
        try:
            _spawn_proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            _spawn_proc.kill()

    if hero_has_odometry():
        events.write("hero_scenario_sensors_present", sim_time=monitor.sim_time)
        return

    carla_step(["destroy-hero"], events, monitor, run_dir / "logs/carla_setup.log")
    events.write("hero_respawn_started", sim_time=monitor.sim_time)

    # carla_spawn_objects.launch.py stays up after spawning; run it in the
    # background like map_run.sh does and wait for the GT odometry topic.
    _spawn_proc = spawn_subprocess(
        ["ros2", "launch", "carla_spawn_objects", "carla_spawn_objects.launch.py",
         f"objects_definition_file:={objects_file}"],
        run_dir / "logs/spawn.log",
    )
    deadline = time.time() + 120
    while time.time() < deadline:
        probe = subprocess.run(
            ["python3", str(REPO / "scripts/validation/wait_graph.py"),
             "--topic", "/carla/hero/odometry", "--timeout", "20"],
            capture_output=True,
        )
        if probe.returncode == 0:
            events.write("hero_respawned", sim_time=monitor.sim_time,
                         objects_file=objects_file)
            return
    raise RuntimeError("hero respawn produced no /carla/hero/odometry")


def teleport_hero_to_route_start(scenario_file, events, monitor, run_dir):
    """Place the hero at the first route waypoint, facing along the route."""
    carla_step(["teleport", str(scenario_file)], events, monitor,
               run_dir / "logs/carla_setup.log")


def await_server_quiescent(events, monitor, run_dir):
    """SR launched into a busy server wedges on its first client call."""
    carla_step(["await-quiescent"], events, monitor, run_dir / "logs/carla_setup.log")


def start_recorders(run_dir: Path, events, monitor):
    estimator = spawn_subprocess(
        ["ros2", "bag", "record", "-o", str(run_dir / "bags/estimator"),
         *ESTIMATOR_BAG_TOPICS],
        run_dir / "logs/bag_estimator.log",
    )
    gt = spawn_subprocess(
        ["ros2", "bag", "record", "-o", str(run_dir / "bags/gt"), *GT_BAG_TOPICS],
        run_dir / "logs/bag_gt.log",
    )
    events.write("recorders_started", sim_time=monitor.sim_time,
                 estimator_bag=str(run_dir / "bags/estimator"),
                 gt_bag=str(run_dir / "bags/gt"))
    return estimator, gt


def stop_recorder(proc, timeout=30):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def scenario_environment():
    """Env for the SR child: ROS + CARLA + ScenarioRunner on the path."""
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        p for p in ["/opt/carla/PythonAPI/carla", SR_ROOT, env.get("PYTHONPATH", "")] if p
    )
    env["SCENARIO_RUNNER_ROOT"] = SR_ROOT
    return env


def run_once(meta, scenario_file, seed, run_dir: Path, profile, events, monitor,
             relay_seed=0):
    events.write("run_start", sim_time=monitor.sim_time, seed=seed, profile=profile,
                 scenario_file=str(scenario_file), relay_seed=relay_seed)
    if relay_seed != seed:
        events.write(
            "seed_env_mismatch", sim_time=monitor.sim_time, requested=seed,
            note="the GNSS relay samples with the stack's SCENARIO_SEED; "
                 "multi-seed averaging requires one stack per seed",
        )
    estimator_bag, gt_bag = start_recorders(run_dir, events, monitor)
    try:
        respawn_hero(str(REPO / "config/scenario/objects_scenario.json"), events, monitor,
                     run_dir)
        teleport_hero_to_route_start(scenario_file, events, monitor, run_dir)
        await_server_quiescent(events, monitor, run_dir)

        # The bridge owns the tick: patch SR's one unguarded world.tick()
        # before it can double-tick the server (see patch_sr.py).
        patch = subprocess.run([sys.executable, str(REPO / "scripts/scenario/patch_sr.py")],
                               capture_output=True, text=True)
        events.write("sr_tick_patch", sim_time=monitor.sim_time,
                     output=patch.stdout.strip() or patch.stderr.strip())
        if patch.returncode != 0:
            raise RuntimeError(f"sr patch failed: {patch.stderr.strip()}")

        output_dir = run_dir / "scenario_runner"
        output_dir.mkdir(parents=True, exist_ok=True)
        proc = subprocess.Popen(
            [sys.executable, f"{SR_ROOT}/scenario_runner.py",
             "--openscenario", str(scenario_file),
             "--json", "--junit", "--outputDir", str(output_dir),
             # No SR --record (server-side CARLA recorder): repeated runs
             # kill the server's sensor streams; the estimator/GT bags and
             # the SR JSON/JUnit carry the evidence contract instead.
             # SR's default client timeout is 10 s; the server is busy with
             # the fresh sensor set at scenario start (the ROS wrapper passes
             # 1000000 for the same reason).
             "--timeout", "120",
             "--waitForEgo", "--host", "127.0.0.1", "--port", "2000"],
            stdout=open(run_dir / "logs/scenario_runner.log", "wb"),
            stderr=subprocess.STDOUT,
            env=scenario_environment(),
            cwd=SR_ROOT,
        )
        events.write("scenario_runner_started", sim_time=monitor.sim_time,
                     pid=proc.pid, sim_timeout_s=meta["sim_timeout_s"])

        start_sim = monitor.sim_time
        verdict = "completed"
        while True:
            rc = proc.poll()
            if rc is not None:
                events.write("scenario_runner_exit", sim_time=monitor.sim_time,
                             returncode=rc)
                if rc != 0:
                    verdict = "sr_error"
                break
            if (monitor.sim_time is not None and start_sim is not None
                    and monitor.sim_time - start_sim > meta["sim_timeout_s"]):
                events.write("external_timeout", sim_time=monitor.sim_time,
                             limit_s=meta["sim_timeout_s"])
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                verdict = "timeout"
                break
            time.sleep(1.0)

        sr_json = parse_sr_results(output_dir, events, monitor)
        return verdict, sr_json
    finally:
        stop_recorder(estimator_bag)
        stop_recorder(gt_bag)
        events.write("recorders_stopped", sim_time=monitor.sim_time)


def parse_sr_results(output_dir: Path, events, monitor):
    """SR writes one <name>_<timestamp>.json + .junit pair per run."""
    files = sorted(output_dir.glob("*.json"), key=lambda p: p.stat().st_mtime)
    if not files:
        events.write("sr_results_missing", sim_time=monitor.sim_time)
        return None
    sr_json = files[-1]
    try:
        doc = json.loads(sr_json.read_text())
        events.write("sr_results", sim_time=monitor.sim_time, file=str(sr_json))
        for criterion in doc.get("criteria", []):
            events.write(
                "criterion", sim_time=monitor.sim_time,
                name=criterion.get("name"),
                verdict="success" if criterion.get("success") else "failure",
                actual_value=criterion.get("actual_value"),
                expected_value=criterion.get("expected_value"),
            )
    except (json.JSONDecodeError, KeyError) as error:
        events.write("sr_results_unparsable", sim_time=monitor.sim_time,
                     file=str(sr_json), error=str(error))
        return None
    return sr_json


def sr_criteria_pass(sr_json):
    """All SR criteria succeeded (missing results fail the run)."""
    if sr_json is None:
        return False
    try:
        doc = json.loads(Path(sr_json).read_text())
        criteria = doc.get("criteria", [])
        return bool(criteria) and all(c.get("success") for c in criteria)
    except (json.JSONDecodeError, OSError):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scenario", help="scenario id (S1..S4) or a path to a .xosc")
    parser.add_argument("--profile", default=None, help="GNSS profile override")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--seed-runs", type=int, default=1,
                        help="N seeds: seed, seed+10, ... (#10 §10)")
    parser.add_argument("--evidence", default="/ws/evidence/scenarios")
    parser.add_argument("--map-dir", default="/ws/maps/current/town05-v14-pg")
    parser.add_argument("--map-eval", default=None,
                        help="map_eval.yaml override (defaults to the committed one)")
    args = parser.parse_args()

    scenario_id = args.scenario
    if scenario_id in SCENARIOS:
        meta = SCENARIOS[scenario_id]
        scenario_file = REPO / "scenarios" / meta["file"]
    else:
        meta = {"profile": "nominal", "sim_timeout_s": 420.0, "ci_set": "adhoc"}
        scenario_file = Path(scenario_id)
    profile = args.profile or meta["profile"]

    # ---- pre-flight: nothing expensive starts before these pass ----
    stack_preflight()
    tag, commit, patched = verify_scenario_runner()
    print(f"scenario_runner pinned: {tag} ({commit})", flush=True)

    map_eval = args.map_eval or str(
        check_route_coverage.REPO / "docs/results/map/town05-v14-pg/map_eval.yaml"
    )
    trajectory = str(Path(args.map_dir) / "trajectory.tum")
    if not Path(trajectory).is_file():
        trajectory = str(check_route_coverage.REPO / "maps/town05-v14-pg/coverage.tum")
    coverage = check_route_coverage.run_preflight(str(scenario_file), map_eval, trajectory)
    if not coverage["inside_coverage"]:
        print("ROUTE_OUTSIDE_COVERAGE " + json.dumps(coverage), file=sys.stderr)
        return 2
    print(f"route coverage OK: max {coverage['max_distance_m']:.2f} m", flush=True)

    # ---- runtime ----
    evidence_root = Path(args.evidence) / scenario_id
    results = []
    overall = 0

    rclpy.init()
    relay_seed = int(os.environ.get("SCENARIO_SEED", "0") or 0)
    for run_index in range(args.seed_runs):
        seed = args.seed + 10 * run_index
        os.environ["SCENARIO_SEED"] = str(seed)
        run_dir = evidence_root / f"seed_{seed}"
        (run_dir / "logs").mkdir(parents=True, exist_ok=True)
        events = EventLog(run_dir / "scenario_events.jsonl")

        monitor = RuntimeMonitor(events)
        spin = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
        spin.start()
        # DDS discovery on a fresh container is slow; the sim-time budget and
        # event stamps need /clock, so wait for the first tick before the run.
        deadline = time.time() + 30
        while monitor.sim_time is None and time.time() < deadline:
            time.sleep(0.5)
        if monitor.sim_time is None:
            events.write("clock_stale")

        try:
            verdict, sr_json = run_once(
                meta, scenario_file, seed, run_dir, profile, events, monitor,
                relay_seed,
            )
        except Exception as error:  # noqa: BLE001 - log visibly, fail the run
            events.write("harness_error", sim_time=monitor.sim_time, error=str(error))
            verdict, sr_json = "harness_error", None
        finally:
            monitor.destroy_node()

        passed = verdict == "completed" and sr_criteria_pass(sr_json)
        run_summary = {
            "scenario": scenario_id, "seed": seed, "profile": profile,
            "ci_set": meta["ci_set"], "verdict": verdict, "passed": passed,
            "scenario_file": str(scenario_file),
            "map_dir": args.map_dir, "coverage": coverage,
            "scenario_runner": {"tag": tag, "commit": commit, "patched": patched},
            "sr_results": str(sr_json) if sr_json else None,
            "bags": {"estimator": str(run_dir / "bags/estimator"),
                     "gt": str(run_dir / "bags/gt")},
            "event_log": str(run_dir / "scenario_events.jsonl"),
        }
        (run_dir / "run_summary.json").write_text(json.dumps(run_summary, indent=2) + "\n")
        events.write("run_end", sim_time=monitor.sim_time, verdict=verdict, passed=passed)
        results.append(run_summary)
        if not passed:
            overall = max(overall, 1)

    (evidence_root / "scenario_runs.json").write_text(
        json.dumps({"scenario": scenario_id, "runs": results}, indent=2) + "\n"
    )
    rclpy.try_shutdown()
    return overall


if __name__ == "__main__":
    sys.exit(main())

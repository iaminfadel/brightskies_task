# Handoff: mapping pipeline validation (2026-10-25)

Context: verify + validate the whole mapping pipeline, then refactor, simplify,
document, and make it human-operable. Validation is done and the mapper is
dramatically improved but not yet passing all gates. This doc hands off the
state, findings, and open threads.

## Where things stand

### The gates exist and work (main deliverable)

Three independent gates now score a map bundle; the old "map is perfect"
circular validation is gone.

| gate | script / binary | what it catches | current result on best map (town05-v7/v8) |
|---|---|---|---|
| Held-out GICP | `ros2 run brightdrive_localization map_check <dir>` | per-scan local misalignment | PASS (0.82-0.86 accepted, median RMSE ~0.12 m) |
| ATE vs ground truth | `scripts/eval_ate.py <bundle> <est_bag> <eval_bag>` | global drift, anchor/yaw/frame errors | FAIL (~2.0 m RMSE, p95 ~3 m, yaw ~10-14 deg) |
| Loop consistency | `scripts/eval_loops.py <bag> <bundle>` | inter-visit misalignment (ghost walls) | FAIL (median correction ~0.8-1.0 m, p95 ~2.5-3.2 m) |

Ground truth for ATE comes from `sensor.pseudo.odom` recorded in a *separate
eval bag* (`<bag>_eval`) by `scripts/map_run.sh` — evaluation path only, never
an estimator input. On the first honest run, the old production mapper scored
ATE 1.93 m RMSE where the old validation said "perfect". The gates agree with
what your eyes saw in RViz.

Artifacts: every gate writes JSON + PNG into `<bundle>/validation/`
(`ate.json/png`, `loop_consistency.json/png`, `map_consistency.json`).
`docs/results/map/town05-v2/` still holds the old (failing, now-known-bad) map
evidence for the before/after story.

### The pose-graph mapper (main fix, nearly there)

`scripts/build_map.py --pose-graph` builds: KISS-ICP odometry edges + GNSS
antenna factors (lever arm modeled in-graph) + course-over-ground yaw factors +
ICP loop edges, solved by a sparse Levenberg-Marquardt Gauss-Newton
(`scripts/pose_graph.py`, scipy-only, SE(2)+z, Huber-robust, gauge anchored at
node 0). Self-tests: `pose_graph.py` (drift 48 m → 0.18 m, yaw gauge recovered
from a 25 deg-rotated init), `eval_ate.py --self-test`,
`eval_loops.py --self-test`. All committed and pushed.

Workflow tools: `--pg-dump <npz>` dumps the graph inputs after the expensive
KISS+loop-detection stage; `--pg-load` resumes from a dump; `--pg-optimized`
loads pre-solved states and goes straight to map accumulation. This exists so
VM interruptions (preemptions, idle-stop) cost one stage, not the whole run.

Best solve so far: `/tmp/pg_solved5.npz` on the laptop (from bag
`/tmp/bags4/map_full4` fetched from the VM): antenna-position residual vs GNSS
median 7 mm; trajectory 3,731 m vs GNSS 3,591 m; but ~5% of the run is pulled
2-3 m off by bad loop edges (diagnosis below). **A solve6 with vetted loop
edges was still running when this doc was written** (pass 1 kept 60/111 loop
edges, dropped 51 with >0.5 m or >10 deg residual; solve pass 2 in
`/tmp/pg_solve6.log` on the laptop, output `/tmp/pg_solved6.npz`). If it
finished clean: build with
`python3 scripts/build_map.py /tmp/bags4/map_full4 /tmp/maps/town05-v9-pg --map-id town05-v9-pg --spawn=-164.732162,-95.141876,0.300000,-179.913696 --pg-optimized /tmp/pg_solved6.npz --heldout 100`,
scp to VM `brightdrive-vm:/ws/maps/builds/`, and run the three gates (see
"run the gates" below).

### What is definitely fixed (root causes found and closed)

1. **KISS-ICP never worked in this repo.** CARLA publishes LiDAR per tick
   (partial arcs, overlapping). KISS diverged on them (150 km phantom
   trajectories) — which is why the original author quietly switched the
   offline build to raw GNSS poses and hid the failure. Fixed at the source:
   run the bridge in **synchronous mode with `fixed_delta_seconds=0.1`** — one
   tick = exactly one 10 Hz lidar rotation, so every message is a COMPLETE
   sweep (verified: 359.6 deg azimuth coverage, 10.9k pts/msg). KISS then
   works: path ratio 0.986 vs GNSS (3,543 vs 3,591 m), loop-closure error
   25.8 m over 3.6 km (0.7%) — exactly what the pose graph closes.
   `--sweep-merge` (azimuth-coverage merge in `read_bag`) exists as a fallback
   for async bags but the sync runtime is the real fix.
2. **GNSS→ENU projection had a bogus 0.9933056 scale** (0.67% north
   compression) in `build_map.py` — removed. Verified against ground truth:
   CARLA GNSS is plain equirectangular about the georeference origin; after
   lever-arm removal, world→ENU is identity+translation to 2 mm.
3. **GNSS lever arm was corrected with convention-broken IMU yaw.** Now the
   antenna→lidar lever (1.0 m) is modeled *inside* the graph: the factor
   residual rotates with the state's yaw (`"lever"` key on gnss edges).
4. **Yaw gauge was unobservable and the anchor convention wrong.** Fixed
   principled-ly: course-over-ground yaw factors (only where the course is
   actually moving) make the frame observable; the optimizer recovers it from
   factors alone — no baked constants, works for any town/spawn/IMU convention.
   Odometry yaw trust raised to KISS's real per-scan accuracy (sigma ~0.8 deg)
   so yaw no longer random-walks through stops (this was causing 65 deg yaw
   swings while stationary).
5. **Solver robustness**: plain GN diverged (oscillation); now
   Levenberg-Marquardt with cost-based step acceptance. Trust-region clamp
   removed in favor of LM.
6. **Infra**: deploy uses `sudo rm` (docker writes /ws as root);
   `deps.Dockerfile` now pip-installs rosbags, scipy, pydantic-settings,
   kiss-icp (previously ad-hoc container installs that silently vanished);
   `small_gicp` python bindings were never built (the offline
   `validate_map.py` could never have run — it reads a `poses.json` nothing
   writes; it is dead code, delete in the refactor).
7. **VM**: the runtime is now `brightdrive-vm` (europe-west4-a, Spot, cloned
   from the old `brightskies-vm` via disk snapshot; infra/config.env points at
   it). Disk grown 100→200 GB after a full-disk boot deadlock (cloud-init
   ENOSPC before it could resize — fixed via a temporary rescue VM, since
   deleted). Old bags and stale builds deleted; ~99 GB free.

### Runtime pacing decision (recorded, was contentious)

`-benchmark -fps=10` on the server breaks sensor streaming (wedged server).
The mapping runtime is: **sync bridge, uncapped server, fast-forward**. Every
lidar message is a full sweep and all stamps are sim-time-consistent; bag
recorders keep up. The fast-forward (RR~2.3) is fine for mapping because
nothing live consumes the topics. Live localization runs are a separate
concern: they will need a rate-limited ticker (bridge ticks as fast as it can;
the old "sim-time starvation" README note comes from this).

## How to work the cloud (what actually worked, after much pain)

Laptop `gcloud` is NOT on the agent shell PATH. The working invocation:

```bash
export PATH="$HOME/google-cloud-sdk/bin:$PATH" CLOUDSDK_PYTHON=/usr/bin/python3.11 PYTHONNOUSERSITE=1
gcloud compute ssh|scp|instances ... --zone europe-west4-a --project project-e038f785-ff95-4654-ba6 --tunnel-through-iap
```

- `--tunnel-through-iap` on every ssh/scp (there is no usable external SSH;
  plain IP connect times out).
- `infra/vm-start.sh` / `scripts/vm-run.sh` still work and use
  `infra/config.env` (VM_NAME=brightdrive-vm, ZONE=europe-west4-a).
- FIREWALL: only instances tagged `brightskies-vm` accept port 22 from IAP.
  A fresh helper VM must `gcloud compute instances add-tags` before SSH works.
- VM jobs that outlive your attention must heartbeat, or the idle-stop guard
  kills the VM after ~30 min: `while :; do sudo touch /var/lib/brightskies/heartbeat; sleep 120; done &`
  (map_run.sh and record_mapping.sh do this; ad-hoc jobs must too).
- Spot preemptions happen roughly hourly in the evening. Boot to SSH takes
  1-25 min. Long stages must be checkpointed (the --pg-dump/--pg-load/--pg-optimized
  chain) or run detached under `/ws/jobs/` via `scripts/vm-run.sh '<cmd>'`.
- Provisioning model CANNOT be changed after creation (spot→standard is
  create-time only). The clone was made by: disk snapshot → new disk from
  snapshot → new instance `--provisioning-model=STANDARD` (on-demand T4
  capacity in europe-west4 was exhausted that evening; the VM is Spot again
  per user decision).
- Rescue pattern for a full-disk VM that cannot boot: stop it, detach the
  disk, attach to a tiny e2 helper VM, growpart/resize2fs/rm junk there,
  detach, delete helper, reattach, start.
- CARLA bring-up is one command on the VM host:
  `bash /ws/brightdrive/scripts/validation/up_carla.sh Town05`
  (containers, fresh server on the right map, bridge, health-checked).
  Recording: `bash /ws/brightdrive/scripts/validation/record_mapping.sh
  <bag> <seconds> <eval_bag> [attempts]` (auto-retries CARLA crashes).

## Run the gates (the 5-minute check anyone can do)

```bash
# on laptop, VM up:
./scripts/vm-run.sh "docker exec brightdrive-deps bash -lc 'source /opt/brightdrive/setup.bash; source /ws/install/setup.bash; cd /ws/brightdrive; \
  ros2 run brightdrive_localization map_check /ws/maps/builds/<ID>; \
  python3 scripts/eval_ate.py /ws/maps/builds/<ID> /ws/bags/map_full4 /ws/bags/map_full4_eval; \
  python3 scripts/eval_loops.py /ws/bags/map_full4 /ws/maps/builds/<ID>'"
```

## Current issues / open threads (priority order)

1. **Close the loop-edge quality gap** (the last ATE/loop blocker). 51/111
   ICP loop edges were mis-locks (up to 2.5 m, 45 deg) — submaps are only
   ~1.5 s of street; ICP locks onto similar geometry. solve6 (two-pass:
   drop edges with >0.5 m / >10 deg residual vs pass-1 solution, re-solve)
   was running. If pass 2 lands under ~0.5 m antenna residual everywhere,
   rebuild + gate; expect held-out PASS, loop gate much closer, ATE near
   GNSS accuracy. If loop gate still fails, tighten `--loop-radius`
   (currently 6.0), require higher ICP inlier ratio, or enlarge submaps
   (`--submap-half` default 45) before solving.
2. **Loop-consistency gate vs solver circularity**: `eval_loops.py` measures
   corrections with the same ICP machinery that produced the edges. Fine for
   now (it measures the *built map*, not the solver), but keep independent.
3. **`validate_map.py` is dead code** (needs small_gicp python bindings +
   a poses.json nothing writes). Delete in refactor; map_check (C++) is the
   real held-out gate.
4. **Refactor + simplify** (the /goal): scripts/ has grown
   (build_map 900+ lines, eval_*, pose_graph, pg flags). Consolidate the
   pose-graph paths, single config file for bags/spawn/extrinsics/gates,
   remove dead flags (`--gnss-pose/--gnss-yaw/--imu-delta/--gnss-delta`
   legacy pose sources can go once the pose-graph path is canonical).
5. **Human-operable docs + entry points**: README's map section still
   documents the old GNSS-posed flow; rewrite around: `make map-record`,
   `make map` (pose-graph), `make map-eval`, plus raw `ros2 launch` usage.
   Add the gates and their meaning. Document the sync-runtime decision and
   the rate-limiting need for live runs.
6. **Foxglove**: deps image has foxglove_bridge; `publish_map_cloud.py`
   publishes the map latched on `/localization/map_cloud`. Nothing wired for
   a "review the map visually" one-command flow yet.
7. **uncommitted in repo**: `docs/diagrams/` (yours) and the Makefile CI
   section were from a parallel session — last seen committed by them; check
   `git status` before pushing.

## Key numbers to beat (bag map_full4, Town05, 3.6 km loop)

- GNSS path: 3,591 m; GT (pseudo.odom) path: 3,688 m (includes reverse wiggle
  at the final stop); KISS-only path: 3,543 m (ratio 0.986).
- KISS-only loop closure error: 25.8 m. After pose graph: sub-metre.
- Gate thresholds: held-out ≥80% accepted & median RMSE ≤0.20 m (note: the
  0.25 m voxel map has a ~0.12 m floor — the gate has little headroom);
  ATE abs RMSE ≤0.30 m, p95 ≤0.60 m, yaw RMSE ≤2.0 deg; loops: ≥10 pairs,
  median correction ≤0.15 m, p95 ≤0.40 m.
- Best map so far (v8, solved w/o loop vetting): held-out PASS 0.82, ATE
  2.02 m RMSE / yaw 14.3 deg, loops median 1.04 m — FAIL, but the trajectory
  overlays GT almost perfectly at map scale; remaining error is the bad-loop
  pull in ~5% of the run (see ate.png in any bundle's validation/).

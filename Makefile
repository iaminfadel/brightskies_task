SHELL := /bin/bash

.PHONY: build up down ps logs test scenario eval compose-config \
        map map-build map-eval map-eval-ate map-install map-upload map-pull eval-kpi map-check \
        vm-provision vm-up vm-down vm-ssh vm-tunnel vm-sync vm-shell vm-job vm-status vm-snapshot \
        ci-lint ci-bootstrap ci-vm-build-test

# --- layout ---------------------------------------------------------------
# The VM workspace is ${WS_DIR}:/ws (brightdrive/, install/, maps/, bags/,
# evidence/). The laptop never builds ROS locally: every `make` target that
# needs ROS runs on the VM through infra/vm-run.sh, which deploys committed
# HEAD and runs the command over IAP.
WS_DIR ?= /ws
COMPOSE ?= docker compose
ROOT ?= /ws/brightdrive

# One-off ROS command in a throwaway deps container (no stack needed).
run-one = ./infra/vm-run.sh --fg "bash infra/ros_exec.sh '$(1)'"

# The pinned, gate-passing bundle (ADR 0002); runtime and install agree (#35).
MAP_ID ?= town05-v14-pg
MAP_BAG ?= /ws/bags/map_full5
MAP_EVAL_BAG ?= $(MAP_BAG)_eval
MAP_DRIVE_SECONDS ?= 900
MAP_BUILDS ?= /ws/maps/builds
MAP_INSTALL ?= $(WS_DIR)/maps/current
MAP_SPAWN ?= -164.732162,-95.141876,0.300000,-179.913696
MAP_BUCKET ?= gs://brightskies-artifacts-330586358676/maps

KPI_EST_BAG ?= /ws/evidence/bags/estimator
KPI_GT_BAG ?= /ws/evidence/bags/gt
KPI_MAP_DIR ?= $(MAP_BUILDS)/$(MAP_ID)
KPI_SCENARIO ?= S1
KPI_EVIDENCE ?= /ws/evidence

# --- compose overrides ----------------------------------------------------
# Every tunable is an env override (see .env.example). Exporting the names
# lets `make up FOO=bar` reach compose interpolation; empty means "unset" for
# the ${VAR:-default} form.
export WS_DIR MAP_DIR CARLA_TOWN OBJECTS_FILE BRIDGE_FIXED_DELTA_SECONDS
export KISS_PARAMS EKF_PARAMS LOCALIZATION_PARAMS EVIDENCE METRICS_FILE
export MAP_DRIVE_SECONDS MAP_SPAWN MAP_BUILDER_FLAGS
export GNSS_PROFILE SCENARIO

# --- build ----------------------------------------------------------------

# Build the ROS workspace into $(WS_DIR)/install with the pinned deps image.
build:
	. ./docker/deps.lock && docker run --rm -v $(WS_DIR):/ws \
	  "$$DEPS_IMAGE" bash -lc 'source /opt/brightdrive/setup.bash && \
	  colcon build --base-paths /ws/brightdrive/src \
	    --build-base /ws/build --install-base /ws/install'

# --- runtime route (make up) ---------------------------------------------

up:
	$(COMPOSE) --profile runtime up -d

down:
	$(COMPOSE) --profile runtime --profile test --profile eval \
	  --profile mapping --profile scenario down --remove-orphans

ps:
	$(COMPOSE) --profile runtime --profile test --profile eval \
	  --profile mapping --profile scenario ps

logs:
	$(COMPOSE) logs -f $(SVC)

compose-config:
	$(COMPOSE) --profile runtime --profile test --profile eval \
	  --profile mapping --profile scenario config

# Runtime + the one-shot smoke that drives it and writes evidence to
# $(EVIDENCE). GPU host only; run it on the VM (infra/vm-run.sh).
test:
	-$(COMPOSE) --profile runtime --profile test down --remove-orphans --timeout 10
	$(COMPOSE) --profile runtime --profile test run --rm smoke

# --- scenario route -------------------------------------------------------
# One locked scenario, deterministic by seed (#37). GNSS_PROFILE is bound to
# the scenario by default so the sensor model and the filter cannot drift.
SCENARIO ?= S1
SCENARIO_SEED ?= 7
SCENARIO_SEED_RUNS ?= 1
SCENARIO_EVIDENCE ?= /ws/evidence/scenarios
export SCENARIO_SEED SCENARIO_SEED_RUNS SCENARIO_EVIDENCE

ifeq ($(SCENARIO),S2)
  GNSS_PROFILE ?= noisy
endif

# The hero must carry the scenario sensor set (GT odometry for the labelled
# open-loop agent + the traffic-lights pseudo actor) from bridge boot --
# respawning under the bridge has crashed CARLA 0.9.15.
export OBJECTS_FILE ?= /ws/brightdrive/config/scenario/objects_scenario.json

scenario:
	$(COMPOSE) --profile runtime --profile scenario run --rm scenario

# --- eval route -----------------------------------------------------------
# Post-drive over artifacts only: no CARLA, never co-scheduled with the
# estimator (GT isolation).
eval:
	$(COMPOSE) --profile eval run --rm evaluator

# --- mapping route --------------------------------------------------------
# Recorder (one lap, sensor-only bag) -> map-builder, sequenced by compose
# (service_completed_successfully). Long job on the VM: wrap in vm-run.sh
# with a heartbeat when running detached.
map:
	MAP_BAG='$(MAP_BAG)' MAP_EVAL_BAG='$(MAP_EVAL_BAG)' MAP_ID='$(MAP_ID)' \
	  $(COMPOSE) --profile mapping run --rm map-builder

# Builder only, from an existing bag: --no-deps skips the recorder.
map-build:
	MAP_BAG='$(MAP_BAG)' MAP_ID='$(MAP_ID)' \
	  $(COMPOSE) --profile mapping run --rm --no-deps map-builder

# ATE gate over the pinned bundle (ground truth vs the recorded trajectory).
map-eval-ate:
	$(call run-one,python3 scripts/eval_ate.py $(MAP_BUILDS)/$(MAP_ID) $(MAP_BAG) $(MAP_EVAL_BAG))

map-eval: map-eval-ate

# Held-out GICP consistency gate (tools/map_check): a standalone development
# tool, built here in the pinned deps image, not a ROS node and never installed.
map-check:
	$(call run-one,cmake -S tools/map_check -B /tmp/map_check -DCMAKE_BUILD_TYPE=Release >/dev/null && \
	  cmake --build /tmp/map_check -j4 >/dev/null && \
	  /tmp/map_check/map_check $(MAP_BUILDS)/$(MAP_ID))

# Runtime install: the three runtime files plus the optional tile grid; the
# evaluator files never reach the estimator.
map-install:
	./infra/vm-run.sh --fg "set -e; \
	  rm -rf $(MAP_INSTALL)/$(MAP_ID) && mkdir -p $(MAP_INSTALL)/$(MAP_ID) && \
	  cp $(MAP_BUILDS)/$(MAP_ID)/map.pcd $(MAP_BUILDS)/$(MAP_ID)/map.yaml \
	     $(MAP_BUILDS)/$(MAP_ID)/trajectory.tum $(MAP_INSTALL)/$(MAP_ID)/ && \
	  for extra in tiles map_index.yaml; do \
	    if [ -e $(MAP_BUILDS)/$(MAP_ID)/\$$extra ]; then \
	      cp -r $(MAP_BUILDS)/$(MAP_ID)/\$$extra $(MAP_INSTALL)/$(MAP_ID)/; \
	    fi; \
	  done; \
	  test ! -e $(MAP_INSTALL)/$(MAP_ID)/map_eval.yaml && \
	  test ! -e $(MAP_INSTALL)/$(MAP_ID)/validation"

map-upload:
	./infra/vm-run.sh --fg "gsutil -m cp -r $(MAP_BUILDS)/$(MAP_ID) $(MAP_BUCKET)/"

map-pull:
	./infra/vm-run.sh --fg "rm -rf $(MAP_BUILDS)/$(MAP_ID) && \
	  mkdir -p $(MAP_BUILDS)/$(MAP_ID) && \
	  gsutil -m cp -r '$(MAP_BUCKET)/$(MAP_ID)/*' $(MAP_BUILDS)/$(MAP_ID)/"

# Heading, published-pose age and latency drift over a recorded scenario
# run; writes metrics.json/CSV/Markdown/plot then gates them.
eval-kpi:
	$(call run-one,python3 scripts/eval_kpi.py $(KPI_EST_BAG) $(KPI_GT_BAG) \
	  $(KPI_MAP_DIR) --scenario $(KPI_SCENARIO) --out-dir $(KPI_EVIDENCE) && \
	  python3 scripts/kpi_gate.py $(KPI_EVIDENCE)/metrics.json)

# --- VM lifecycle ---------------------------------------------------------

vm-provision:
	./infra/provision.sh

vm-up:
	./infra/vm-start.sh

vm-down:
	./infra/vm-stop.sh

vm-ssh:
	./infra/vm-ssh.sh $(ARGS)

vm-tunnel:
	./infra/vm-ssh.sh -N -L 8765:127.0.0.1:8765

vm-sync:
	./infra/vm-run.sh --sync-only

vm-shell:
	./infra/vm-ssh.sh bash

vm-job:
	python3 infra/vm_job.py $(ARGS)

vm-status:
	./infra/vm-status.sh

vm-snapshot:
	./infra/snapshot.sh

# --- CI (see docs/ci.md) --------------------------------------------------

ci-lint:
	pre-commit run --all-files

ci-bootstrap:
	./infra/ci_bootstrap.sh

# Exactly what ci/build-test runs, but on the VM (the laptop never builds ROS).
ci-vm-build-test:
	./infra/vm-run.sh --fg "bash infra/ros_exec.sh 'colcon build \
	  --base-paths /ws/brightdrive/src --build-base /ws/build --install-base /ws/install && \
	  colcon test --packages-select brightdrive_localization brightdrive_map_server && \
	  colcon test-result --verbose'"

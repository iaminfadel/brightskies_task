SHELL := /bin/bash

.PHONY: vm-provision vm-up vm-down vm-ssh vm-status vm-snapshot \
        map map-gnss-imu map-record map-check map-eval map-eval-ate map-eval-loops map-verify \
        map-install map-upload map-pull \
        ci-lint ci-bootstrap ci-vm-build-test

PY_RUN = source /opt/brightdrive/setup.bash; source /ws/install/setup.bash 2>/dev/null || true; cd /ws/brightdrive

# --- CI (see docs/ci.md) ----------------------------------------------------

# Exactly what the ci/lint job runs (pip install pre-commit once, then:
#   pre-commit install).
ci-lint:
	pre-commit run --all-files

# One-time: staging branch + branch protection (idempotent, uses gh).
ci-bootstrap:
	./scripts/ci_bootstrap.sh

# Exactly what the ci/build-test job runs, but on the VM deps container
# (the laptop never builds ROS locally).
ci-vm-build-test:
	./scripts/vm-run.sh --fg "docker exec brightdrive-deps bash -lc '$(PY_RUN) && colcon build --packages-select brightdrive_interfaces brightdrive_localization brightdrive_bringup && colcon test --packages-select brightdrive_localization && colcon test-result --verbose'"

MAP_ID ?= town05-pg
MAP_BAG ?= /ws/bags/map_full4
MAP_EVAL_BAG ?= $(MAP_BAG)_eval
MAP_DRIVE_SECONDS ?= 900
MAP_RECORD_ATTEMPTS ?= 3
MAP_BUILDS ?= /ws/maps/builds
MAP_INSTALL ?= install/maps
MAP_SPAWN ?= -164.732162,-95.141876,0.300000,-179.913696
MAP_BUCKET ?= gs://brightskies-artifacts-330586358676/maps

PY_ENV = source /opt/brightdrive/setup.bash; source /ws/install/setup.bash; cd /ws/brightdrive

# Lidar SLAM: KISS-ICP odometry + GNSS/level-arm/course-yaw factors + GNSS-vetoed
# ICP loop closures, solved as one pose graph. Long job: runs detached on the
# VM with a heartbeat so the idle-stop guard leaves it alone.
map:
	./scripts/vm-run.sh "bash -c 'while :; do sudo touch /var/lib/brightskies/heartbeat 2>/dev/null || true; sleep 120; done & HP=$$!; docker exec brightdrive-deps bash -lc \"$(PY_ENV) && python3 scripts/build_map.py $(MAP_BAG) $(MAP_BUILDS)/$(MAP_ID) --map-id $(MAP_ID) --spawn=$(MAP_SPAWN) --pose-graph\"; rc=$$?; kill $$HP 2>/dev/null; exit $$rc'"

# Deterministic diagnostic baseline (no lidar): GNSS antenna positions + IMU
# yaw. Passes gates only because the CARLA rig's GNSS/IMU are noise-free --
# never the shipping map path.
map-gnss-imu:
	./scripts/vm-run.sh --fg "docker exec brightdrive-deps bash -lc '$(PY_ENV) && python3 scripts/build_map.py $(MAP_BAG) $(MAP_BUILDS)/$(MAP_ID) --map-id $(MAP_ID) --spawn=$(MAP_SPAWN) --gnss-imu'"

map-record:
	./scripts/vm-run.sh "bash /ws/brightdrive/scripts/validation/record_mapping.sh $(MAP_BAG) $(MAP_DRIVE_SECONDS) $(MAP_EVAL_BAG) $(MAP_RECORD_ATTEMPTS)"

map-check:
	./scripts/vm-run.sh --fg "docker exec brightdrive-deps bash -lc 'source /opt/brightdrive/setup.bash; source /ws/install/setup.bash; ros2 run brightdrive_localization map_check $(MAP_BUILDS)/$(MAP_ID)'"

map-eval-ate:
	./scripts/vm-run.sh --fg "docker exec brightdrive-deps bash -lc '$(PY_ENV) && python3 scripts/eval_ate.py $(MAP_BUILDS)/$(MAP_ID) $(MAP_BAG) $(MAP_EVAL_BAG)'"

map-eval-loops:
	./scripts/vm-run.sh --fg "docker exec brightdrive-deps bash -lc '$(PY_ENV) && python3 scripts/eval_loops.py $(MAP_BAG) $(MAP_BUILDS)/$(MAP_ID)'"

map-eval: map-check map-eval-ate map-eval-loops

map-install:
	./scripts/vm-run.sh --fg "docker exec brightdrive-deps bash -lc 'rm -rf /ws/maps/current/$(MAP_ID) && mkdir -p /ws/maps/current/$(MAP_ID) && cp $(MAP_BUILDS)/$(MAP_ID)/map.pcd $(MAP_BUILDS)/$(MAP_ID)/map.yaml $(MAP_BUILDS)/$(MAP_ID)/trajectory.tum /ws/maps/current/$(MAP_ID)/ && test ! -e /ws/maps/current/$(MAP_ID)/map_eval.yaml && test ! -e /ws/maps/current/$(MAP_ID)/validation'"

map-verify:
	./scripts/vm-run.sh --fg "python3 /ws/brightdrive/scripts/map_verify.py $(MAP_BUILDS)/$(MAP_ID)"

map-upload:
	./scripts/vm-run.sh --fg "gsutil -m cp -r $(MAP_BUILDS)/$(MAP_ID) $(MAP_BUCKET)/"

map-pull:
	./scripts/vm-run.sh --fg "rm -rf $(MAP_BUILDS)/$(MAP_ID) && mkdir -p $(MAP_BUILDS)/$(MAP_ID) && gsutil -m cp -r '$(MAP_BUCKET)/$(MAP_ID)/*' $(MAP_BUILDS)/$(MAP_ID)/ && python3 /ws/brightdrive/scripts/map_verify.py $(MAP_BUILDS)/$(MAP_ID)"

vm-provision:
	./infra/provision.sh

vm-up:
	./infra/vm-start.sh

vm-down:
	./infra/vm-stop.sh

vm-ssh:
	./infra/vm-ssh.sh $(ARGS)

vm-sync:
	./scripts/vm-run.sh --sync-only

vm-shell:
	./infra/vm-ssh.sh bash

vm-job:
	python3 scripts/vm_job.py $(ARGS)

vm-status:
	./infra/vm-status.sh

vm-snapshot:
	./infra/snapshot.sh

# --- compose routes (#26) ---------------------------------------------------
# One docker-compose.yml, routes as profiles: runtime (make up), test
# (make test), eval (make eval), mapping (make map-compose / map-build).
# Additive alongside the legacy vm-run.sh path above; both share host ports
# 2000/8765 and must not run at once (stop_stack.sh stops the legacy stack).
# Every tunable is an env override -- see .env.example; never edit config
# files in place.

COMPOSE ?= docker compose
WS_DIR ?= /ws

# Flow through to compose interpolation when set on the make command line;
# unset values fall back to the compose defaults (empty env == unset for
# the ${VAR:-default} interpolation form). WS_DIR is also used directly in
# the build recipe, hence the default above.
export WS_DIR MAP_DIR CARLA_TOWN OBJECTS_FILE BRIDGE_FIXED_DELTA_SECONDS
export KISS_PARAMS EKF_PARAMS LOCALIZATION_PARAMS EVIDENCE METRICS_FILE
export MAP_DRIVE_SECONDS MAP_SPAWN MAP_BUILDER_FLAGS

# Recorder/builder defaults are deliberately NEUTRAL: the recorder deletes
# MAP_BAG before recording, so the legacy map targets' golden bags are only
# ever touched when named explicitly here.
COMPOSE_MAP_BAG ?= /ws/bags/map_full5
COMPOSE_MAP_ID ?= town05-compose

.PHONY: up down ps logs compose-config build test eval map-compose map-build

up:
	$(COMPOSE) --profile runtime up -d

down:
	$(COMPOSE) --profile runtime --profile test --profile eval --profile mapping down --remove-orphans

ps:
	$(COMPOSE) --profile runtime --profile test --profile eval --profile mapping ps

logs:
	$(COMPOSE) logs -f $(SVC)

compose-config:
	$(COMPOSE) --profile runtime --profile test --profile eval --profile mapping config

# Build the ROS workspace into $(WS_DIR)/install with the pinned deps image
# (digest from docker/deps.lock). Run on the VM or any Linux+Docker host;
# the laptop never builds ROS locally.
build:
	. ./docker/deps.lock && docker run --rm -v $(WS_DIR):/ws \
	  "$$DEPS_IMAGE" bash -lc 'source /opt/brightdrive/setup.bash && colcon build --base-paths /ws/brightdrive/src --build-base /ws/build --install-base /ws/install'

# The compose smoke: brings the runtime route up through dependencies, then
# the one-shot smoke service drives it and writes evidence to $(EVIDENCE).
# GPU host only; on the VM run it through vm-run.sh (as with the legacy
# targets) so the SSH-held session keeps the idle-stop guard calm.
test:
	-$(COMPOSE) --profile runtime --profile test down --remove-orphans --timeout 10
	$(COMPOSE) --profile runtime --profile test run --rm smoke

# Post-drive evaluator over artifacts only -- no carla, and never
# co-scheduled with the estimator (GT isolation).
eval:
	$(COMPOSE) --profile eval run --rm evaluator

# Mapping route: recorder -> map-builder with compose-native sequencing
# (the builder waits for the recorder via service_completed_successfully).
# Long job on the VM: wrap in vm-run.sh with a heartbeat (see the legacy
# map target) when running detached.
map-compose:
	MAP_BAG='$(COMPOSE_MAP_BAG)' MAP_EVAL_BAG='$(COMPOSE_MAP_BAG)_eval' \
	MAP_ID='$(COMPOSE_MAP_ID)' \
	  $(COMPOSE) --profile mapping run --rm map-builder

# Builder only, from an existing bag: --no-deps skips the recorder.
map-build:
	MAP_BAG='$(COMPOSE_MAP_BAG)' MAP_ID='$(COMPOSE_MAP_ID)' \
	  $(COMPOSE) --profile mapping run --rm --no-deps map-builder

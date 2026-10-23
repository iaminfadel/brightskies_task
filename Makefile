SHELL := /bin/bash

.PHONY: vm-provision vm-up vm-down vm-ssh vm-status vm-snapshot \
        map map-check map-verify map-install map-upload map-pull

MAP_ID ?= town05-v2
MAP_BAG ?= /ws/bags/map_full2
MAP_BUILDS ?= /ws/maps/builds
MAP_INSTALL ?= install/maps
MAP_SPAWN ?= -164.732162,-95.141876,0.300000,-179.913696
MAP_BUCKET ?= gs://brightskies-artifacts-330586358676/maps

map:
	./scripts/vm-run.sh --fg "docker exec brightdrive-deps bash -lc 'export PYTHONPATH=/opt/ros_ws/src/kiss-icp/python:\${PYTHONPATH:-}; source /opt/brightdrive/setup.bash; source /ws/install/setup.bash; cd /ws/brightdrive && python3 scripts/build_map.py $(MAP_BAG) $(MAP_BUILDS)/$(MAP_ID) --map-id $(MAP_ID) --spawn=$(MAP_SPAWN) --gnss-pose --gnss-yaw --imu-delta'"

map-check:
	./scripts/vm-run.sh --fg "docker exec brightdrive-deps bash -lc 'source /opt/brightdrive/setup.bash; source /ws/install/setup.bash; ros2 run brightdrive_localization map_check $(MAP_BUILDS)/$(MAP_ID)'"

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
	./infra/vm-ssh.sh

vm-status:
	./infra/vm-status.sh

vm-snapshot:
	./infra/snapshot.sh

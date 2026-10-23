SHELL := /bin/bash

.PHONY: vm-provision vm-up vm-down vm-ssh vm-status vm-snapshot \
        map map-check map-install map-pull

MAP_ID ?= town05-v2
MAP_BAG ?= /ws/bags/map_full2
MAP_BUILDS ?= /ws/maps/builds
MAP_INSTALL ?= install/maps
MAP_SPAWN ?= -164.732162,-95.141876,0.300000,-179.913696
MAP_BUCKET ?= gs://brightskies-artifacts-330586358676/maps

map:
	./scripts/vm-run.sh --fg "docker exec brightdrive-deps bash -lc 'source /opt/brightdrive/setup.bash; source /ws/install/setup.bash; cd /ws/brightdrive && python3 scripts/build_map.py $(MAP_BAG) $(MAP_BUILDS)/$(MAP_ID) --map-id $(MAP_ID) --spawn=$(MAP_SPAWN) --gnss-pose --gnss-yaw --imu-delta'"

map-check:
	./scripts/vm-run.sh --fg "docker exec brightdrive-deps bash -lc 'source /opt/brightdrive/setup.bash; source /ws/install/setup.bash; ros2 run brightdrive_localization map_check $(MAP_BUILDS)/$(MAP_ID)'"

map-install:
	rm -rf $(MAP_INSTALL)/$(MAP_ID)
	mkdir -p $(MAP_INSTALL)/$(MAP_ID)
	cp $(MAP_BUILDS)/$(MAP_ID)/map.pcd $(MAP_BUILDS)/$(MAP_ID)/map.yaml \
	   $(MAP_BUILDS)/$(MAP_ID)/trajectory.tum $(MAP_INSTALL)/$(MAP_ID)/
	@test ! -e $(MAP_INSTALL)/$(MAP_ID)/map_eval.yaml
	@test ! -e $(MAP_INSTALL)/$(MAP_ID)/validation

map-pull:
	rm -rf $(MAP_BUILDS)/$(MAP_ID)
	mkdir -p $(MAP_BUILDS)/$(MAP_ID)
	gsutil -m cp -r "$(MAP_BUCKET)/$(MAP_ID)/*" $(MAP_BUILDS)/$(MAP_ID)/
	cd $(MAP_BUILDS)/$(MAP_ID) && echo "$$(python3 -c 'import json;print(json.load(open("map.yaml"))["pcd_sha256"])')  map.pcd" | sha256sum -c

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

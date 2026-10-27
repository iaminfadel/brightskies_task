#!/usr/bin/env python3
"""Patch the pinned ScenarioRunner for the bridge-owned tick (#37, #10 §2).

Two minimal adaptations for this runtime, both in scenario_runner.py's
_prepare_ego_vehicles (the pin in config/scenario/scenario_runner.yaml still
enforces the exact upstream commit -- these are recorded execution-
environment adaptations, not a fork):

1. SR ticks the world whenever the WORLD is in synchronous mode, even when
   SR itself runs without --sync. Under the BrightDrive runtime the bridge
   owns the tick (compose bridge, sync mode), so that call double-ticks the
   server and corrupts its sensor streams. SR must wait for ticks, never
   issue them.
2. SR then teleports the matched ego to the OSC entity transform -- which
   is the origin (0, 0, 0) for a --waitForEgo entity with no Init
   TeleportAction. In Town05 the origin is inside static geometry, and the
   teleport SIGSEGVs the server. The harness has already placed the ego at
   the route start; SR must leave it there.
"""
import sys
from pathlib import Path

SR_ROOT = Path("/opt/scenario_runner")
TARGET = SR_ROOT / "scenario_runner.py"

ORIGINAL_TICK = """        # sync state
        if CarlaDataProvider.is_sync_mode():
            self.world.tick()
        else:
            self.world.wait_for_tick()"""

PATCHED_TICK = """        # sync state
        # BrightDrive harness patch (#37): the ROS bridge owns the tick in
        # this runtime; without --sync SR must wait for ticks, never issue
        # them -- a double tick corrupts the server's sensor streams.
        self.world.wait_for_tick()"""

ORIGINAL_TELEPORT = """            for i, _ in enumerate(self.ego_vehicles):
                self.ego_vehicles[i].set_transform(ego_vehicles[i].transform)
                self.ego_vehicles[i].set_target_velocity(carla.Vector3D())
                self.ego_vehicles[i].set_target_angular_velocity(carla.Vector3D())
                self.ego_vehicles[i].apply_control(carla.VehicleControl())
                CarlaDataProvider.register_actor(self.ego_vehicles[i], ego_vehicles[i].transform)"""

PATCHED_TELEPORT = """            for i, _ in enumerate(self.ego_vehicles):
                # BrightDrive harness patch (#37): the harness teleports the
                # ego to the route start before SR attaches; SR's default
                # entity transform (the origin) is inside geometry and the
                # teleport crashes the server. Leave the ego where it is.
                CarlaDataProvider.register_actor(self.ego_vehicles[i])"""


def main():
    source = TARGET.read_text()
    if PATCHED_TICK in source and PATCHED_TELEPORT in source:
        print("sr patch: already applied")
        return 0
    if ORIGINAL_TICK not in source or ORIGINAL_TELEPORT not in source:
        print("sr patch: expected blocks not found; upstream changed?", file=sys.stderr)
        return 1
    source = source.replace(ORIGINAL_TICK, PATCHED_TICK, 1)
    source = source.replace(ORIGINAL_TELEPORT, PATCHED_TELEPORT, 1)
    TARGET.write_text(source)
    print("sr patch: applied to", TARGET)
    return 0


if __name__ == "__main__":
    sys.exit(main())

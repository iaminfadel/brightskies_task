import carla
import csv
import json
import math
import os
import sys
import time
import traceback

HOST = "127.0.0.1"
PORT = 2000
TM_PORT = 8000
DURATION_S = 180.0
NPC_TARGET = 8
SPAWN_INDEX = 12
OUT_CSV = "/content/gt.csv"
OUT_META = "/content/drive_meta.json"
OUT_LOG = "/content/drive.log"

log_f = open(OUT_LOG, "a", buffering=1)


def log(*args):
    msg = " ".join(str(a) for a in args)
    print(msg, flush=True)
    log_f.write(msg + "\n")


def main():
    log("drive start", time.strftime("%Y-%m-%d %H:%M:%S"))
    client = carla.Client(HOST, PORT)
    client.set_timeout(60.0)
    world = client.get_world()
    log("map", world.get_map().name)

    hero = None
    deadline = time.time() + 180
    while time.time() < deadline:
        for a in world.get_actors().filter("vehicle.*"):
            if a.attributes.get("role_name", "") == "hero":
                hero = a
                break
        if hero is not None:
            break
        time.sleep(1.0)
    if hero is None:
        raise RuntimeError("hero actor not found")

    m = world.get_map()
    sps = list(m.get_spawn_points())
    sp = sps[SPAWN_INDEX]
    log("teleport hero to spawn", SPAWN_INDEX, sp.location, "yaw", sp.rotation.yaw)
    try:
        hero.set_autopilot(False, TM_PORT)
    except Exception:
        pass
    try:
        hero.apply_control(carla.VehicleControl())
    except Exception:
        pass
    hero.set_transform(sp)
    time.sleep(1.0)
    log("hero", hero.id, hero.type_id, "at", hero.get_transform().location)

    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(1234)
    tm.ignore_lights_percentage(hero, 100)
    tm.ignore_signs_percentage(hero, 100)

    vehicles = list(world.get_blueprint_library().filter("vehicle.*"))
    spawn_points = list(m.get_spawn_points())
    hero_loc = hero.get_transform().location
    npcs = []
    for sp_npc in spawn_points:
        if len(npcs) >= NPC_TARGET:
            break
        if sp_npc.location.distance(hero_loc) < 35.0:
            continue
        bp = vehicles[len(npcs) % len(vehicles)]
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", f"npc{len(npcs)}")
        try:
            v = world.try_spawn_actor(bp, sp_npc)
        except Exception:
            v = None
        if v is None:
            continue
        try:
            v.set_autopilot(True, TM_PORT)
        except Exception:
            pass
        npcs.append(v)
    log("npcs", len(npcs))

    hero.set_autopilot(True, TM_PORT)
    tm.vehicle_percentage_speed_difference(hero, 40.0)

    rows = []
    last_frame = -1
    start_t = None
    phase = 0
    error = None
    try:
        while True:
            snap = world.get_snapshot()
            frame = snap.frame
            if frame == last_frame:
                time.sleep(0.02)
                continue
            last_frame = frame
            t = snap.timestamp.elapsed_seconds
            if start_t is None:
                start_t = t
            tr = hero.get_transform()
            vel = hero.get_velocity()
            speed_kmh = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z) * 3.6
            rows.append([
                frame, round(t, 4), round(t - start_t, 4),
                round(tr.location.x, 4), round(tr.location.y, 4), round(tr.location.z, 4),
                round(tr.rotation.roll, 4), round(tr.rotation.pitch, 4), round(tr.rotation.yaw, 4),
                round(speed_kmh, 4),
            ])
            elapsed = t - start_t
            if phase == 0 and elapsed >= 60.0:
                tm.vehicle_percentage_speed_difference(hero, 20.0)
                phase = 1
                log("phase 2: speed difference +20%% at t=%.1f" % elapsed)
            elif phase == 1 and elapsed >= 120.0:
                tm.vehicle_percentage_speed_difference(hero, 40.0)
                phase = 2
                log("phase 3: speed difference +40%% at t=%.1f" % elapsed)
            if elapsed >= DURATION_S:
                break
    except KeyboardInterrupt:
        log("interrupted")
    except Exception as exc:
        error = traceback.format_exc()
        log("drive error", exc)
        log(error)
    finally:
        for v in npcs:
            try:
                v.destroy()
            except Exception:
                pass
        try:
            hero.set_autopilot(False, TM_PORT)
        except Exception:
            pass
        with open(OUT_CSV, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["frame", "t_abs", "t", "x", "y", "z", "roll", "pitch", "yaw", "speed_kmh"])
            w.writerows(rows)
        speeds = [r[9] for r in rows]
        meta = {
            "map": world.get_map().name,
            "frames": len(rows),
            "start_frame": rows[0][0] if rows else None,
            "end_frame": rows[-1][0] if rows else None,
            "start_abs": start_t,
            "duration_s": rows[-1][2] if rows else None,
            "max_speed_kmh": max(speeds) if speeds else None,
            "mean_speed_kmh": (sum(speeds) / len(speeds)) if speeds else None,
            "npcs": len(npcs),
            "hero_id": hero.id,
            "hero_type": hero.type_id,
            "tm_port": TM_PORT,
            "spawn_index": SPAWN_INDEX,
            "error": error,
        }
        with open(OUT_META, "w") as f:
            json.dump(meta, f, indent=2)
        log("drive done", json.dumps(meta))
        log_f.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        log_f.write(traceback.format_exc())
        log_f.close()
        sys.exit(1)

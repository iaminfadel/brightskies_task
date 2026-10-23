#!/usr/bin/env python3
"""Load a CARLA town and wait until it is ready (human/CI helper).

Run inside the brightdrive-deps container next to the carla-server:

  python3 scripts/validation/load_carla_town.py --town Town05 --timeout 900
"""
import argparse
import time

import carla


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--town", default="Town05")
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--period", type=float, default=5.0)
    args = parser.parse_args()

    deadline = time.time() + args.timeout
    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)
    while time.time() < deadline:
        try:
            name = client.get_world().get_map().name
            break
        except RuntimeError:
            print("waiting for CARLA RPC ...", flush=True)
            time.sleep(15)
    else:
        raise SystemExit("CARLA RPC never became ready")
    print(f"rpc ready, current map: {name}", flush=True)

    if name.endswith(args.town):
        print("TOWN_READY", flush=True)
        return

    print(f"loading {args.town} ...", flush=True)
    try:
        client.load_world(f"/Game/Carla/Maps/{args.town}")
    except RuntimeError as error:
        print(f"load_world call returned: {error}", flush=True)
    while time.time() < deadline:
        time.sleep(args.period)
        try:
            name = carla.Client(args.host, args.port).get_world().get_map().name
        except RuntimeError:
            continue
        if name.endswith(args.town):
            print("TOWN_READY", flush=True)
            return
    raise SystemExit(f"{args.town} did not load within {args.timeout:.0f}s")


if __name__ == "__main__":
    main()

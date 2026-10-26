#!/usr/bin/env python3
"""Throwaway prototype for #27 tile lookup + sliding window. Not production."""
import math, random


def tile_index(px, py, origin, tile):
    return (math.floor((px - origin[0]) / tile), math.floor((py - origin[1]) / tile))


def window(px, py, index, r_load, r_evict, loaded):
    # O(1) lookup for seed tile; expand by Manhattan-ish Chebyshev ring until r.
    ix, iy = tile_index(px, py, index["origin"], index["tile"])
    want = set()
    maxr = int(math.ceil(r_load / index["tile"])) + 1
    for dx in range(-maxr, maxr + 1):
        for dy in range(-maxr, maxr + 1):
            cx = (ix + dx + 0.5) * index["tile"] + index["origin"][0]
            cy = (iy + dy + 0.5) * index["tile"] + index["origin"][1]
            if math.hypot(cx - px, cy - py) <= r_load:
                want.add((ix + dx, iy + dy))
    # evict
    for t in list(loaded):
        cx = (t[0] + 0.5) * index["tile"] + index["origin"][0]
        cy = (t[1] + 0.5) * index["tile"] + index["origin"][1]
        if math.hypot(cx - px, cy - py) > r_evict:
            loaded.discard(t)
    loaded |= want
    return sorted(loaded)


if __name__ == "__main__":
    idx = {"origin": (0.0, 0.0), "tile": 20.0}
    loaded = set()
    # drive a line; window should grow then evict
    for x in range(0, 120, 5):
        tiles = window(x, 0, idx, 30, 45, loaded)
        assert tile_index(x, 0, idx["origin"], idx["tile"]) in [t for t in tiles]
    assert len(loaded) < 40, len(loaded)
    print("ok", len(loaded), "tiles loaded at end")

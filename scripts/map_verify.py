#!/usr/bin/env python3
"""Verify a saved-map bundle against the #15 contract."""
import hashlib
import json
import sys
from pathlib import Path

REQUIRED_MAP = {
    "schema_version",
    "map_id",
    "map_frame",
    "created_at",
    "anchor",
    "T_map_slam",
    "pcd_sha256",
    "voxel_size_m",
    "point_count",
    "bounds",
    "coverage_radius_m",
    "source",
    "generation_command",
}
REQUIRED_SOURCE = {
    "run_id",
    "bag_sha256",
    "git_sha",
    "kiss_icp_version",
    "config_sha256",
}
REQUIRED_EVAL = {
    "schema_version",
    "map_id",
    "T_map_carla",
    "T_map_carla_validation",
    "heldout",
    "source_run_id",
}
REQUIRED_MANIFEST = {
    "schema_version",
    "map_id",
    "created_at",
    "pcd_sha256",
    "bundle_sha256",
    "files",
    "source_bag_sha256",
    "git_sha",
    "tool_versions",
    "generation_command",
}
REQUIRED_TILE_INDEX = {
    "schema_version",
    "map_id",
    "tile_size_m",
    "origin",
    "tile_count",
    "point_count",
    "files",
}
PLACEHOLDERS = {"...", "TBD", "TODO", "", "unknown"}
EVALUATOR_FILES = {
    "validation/map_consistency.json",
    "validation/ate.json",
    "validation/ate.png",
    "validation/loop_consistency.json",
    "validation/loop_check.png",
}


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def fail(errors, message):
    errors.append(message)


def check_matrix(errors, name, values):
    if not isinstance(values, list) or len(values) != 16:
        fail(errors, f"{name} must be 16 row-major floats")
        return
    if any(not isinstance(value, (int, float)) for value in values):
        fail(errors, f"{name} contains non-numeric values")


def pcd_point_count(path):
    """POINTS from a binary PCD header, without reading the payload."""
    header = b""
    with open(path, "rb") as handle:
        while b"DATA " not in header:
            chunk = handle.read(4096)
            if not chunk:
                break
            header += chunk
    for line in header.decode("ascii", errors="replace").splitlines():
        if line.startswith("POINTS "):
            return int(line.split()[1])
    raise ValueError(f"{path}: no POINTS in the PCD header")


def check_tiles(errors, root, map_yaml, tile_index):
    """Tiling is additive (#32): the index is optional, but when it is present
    it must describe the tiles that are on disk and partition the full map."""
    missing = REQUIRED_TILE_INDEX - set(tile_index)
    if missing:
        fail(errors, f"map_index.yaml missing {sorted(missing)}")
        return
    if tile_index.get("map_id") != map_yaml.get("map_id"):
        fail(errors, "map_index.yaml map_id does not match map.yaml")
    if not float(tile_index.get("tile_size_m", 0.0)) > 0.0:
        fail(errors, "map_index.yaml tile_size_m must be positive")
    origin = tile_index.get("origin", {})
    if not all(isinstance(origin.get(key), (int, float)) for key in ("x_m", "y_m")):
        fail(errors, "map_index.yaml origin must be {x_m, y_m}")

    entries = tile_index.get("files")
    if not isinstance(entries, list):
        fail(errors, "map_index.yaml files must be a list")
        return
    total = 0
    for entry in entries:
        path = root / entry.get("path", "")
        if not path.is_file():
            fail(errors, f"map_index.yaml lists missing tile {entry.get('path')}")
            continue
        try:
            count = pcd_point_count(path)
        except (OSError, ValueError) as error:
            fail(errors, f"{entry.get('path')}: {error}")
            continue
        total += count
        if count != entry.get("point_count"):
            fail(errors, f"{entry.get('path')} point_count does not match the tile")
        if entry.get("ix") is None or entry.get("iy") is None:
            fail(errors, f"{entry.get('path')} must carry its ix/iy grid index")
    if total != map_yaml.get("point_count"):
        fail(errors, "tiles do not partition the map: point counts do not sum to point_count")
    if tile_index.get("tile_count") != len(entries):
        fail(errors, "map_index.yaml tile_count does not match its file list")
    if tile_index.get("point_count") != total:
        fail(errors, "map_index.yaml point_count does not match its tiles")


def main():
    if len(sys.argv) != 2:
        print("usage: map_verify.py <bundle_dir>", file=sys.stderr)
        return 2
    root = Path(sys.argv[1])
    errors = []
    yaml_path = root / "map.yaml"
    eval_path = root / "map_eval.yaml"
    manifest_path = root / "manifest.json"
    for path in (yaml_path, eval_path, manifest_path, root / "map.pcd", root / "trajectory.tum"):
        if not path.is_file():
            fail(errors, f"missing {path.relative_to(root)}")
    if errors:
        print_results(errors)
        return 1

    map_yaml = json.loads(yaml_path.read_text(encoding="utf-8"))
    map_eval = json.loads(eval_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    missing = REQUIRED_MAP - set(map_yaml)
    if missing:
        fail(errors, f"map.yaml missing {sorted(missing)}")
    missing = REQUIRED_SOURCE - set(map_yaml.get("source", {}))
    if missing:
        fail(errors, f"map.yaml source missing {sorted(missing)}")
    missing = REQUIRED_EVAL - set(map_eval)
    if missing:
        fail(errors, f"map_eval.yaml missing {sorted(missing)}")
    missing = REQUIRED_MANIFEST - set(manifest)
    if missing:
        fail(errors, f"manifest.json missing {sorted(missing)}")

    if map_yaml.get("map_frame") != "map":
        fail(errors, "map_frame must be map")
    if map_yaml.get("anchor", {}).get("source") != "first_gnss_fix":
        fail(errors, "anchor.source must be first_gnss_fix")
    check_matrix(errors, "T_map_slam", map_yaml.get("T_map_slam"))
    check_matrix(errors, "T_map_carla", map_eval.get("T_map_carla"))

    pcd_sha = sha256_file(root / "map.pcd")
    if map_yaml.get("pcd_sha256") != pcd_sha:
        fail(errors, "map.yaml pcd_sha256 does not match map.pcd")
    if manifest.get("pcd_sha256") != pcd_sha:
        fail(errors, "manifest pcd_sha256 does not match map.pcd")

    validation = map_eval.get("T_map_carla_validation", {})
    if not validation.get("samples", 0) >= 3:
        fail(errors, "T_map_carla_validation.samples must be >= 3")
    if not 0.0 <= float(validation.get("residual_m", 1e9)) < float(validation.get("threshold_m", 0.5)):
        fail(errors, "T_map_carla_validation.residual_m must be below threshold_m")
    heldout = map_eval.get("heldout", {})
    if not heldout.get("count", 0) >= 20:
        fail(errors, "heldout.count must be >= 20 (map_check gate needs the sample size)")
    if not (root / heldout.get("directory", "validation/heldout")).is_dir():
        fail(errors, "heldout directory missing")
    gicp_sha = heldout.get("gicp_settings_sha256")
    if not isinstance(gicp_sha, str) or len(gicp_sha) != 64:
        fail(errors, "gicp_settings_sha256 must be a sha256 hex digest")

    for key in ("source_bag_sha256", "git_sha"):
        value = manifest.get(key)
        if not isinstance(value, str) or value in PLACEHOLDERS:
            fail(errors, f"manifest {key} is a placeholder")
    for key in ("bag_sha256", "config_sha256"):
        value = map_yaml.get("source", {}).get(key)
        if not isinstance(value, str) or value in PLACEHOLDERS:
            fail(errors, f"map.yaml source.{key} is a placeholder")

    listed = {entry["path"]: entry for entry in manifest.get("files", [])}
    if not listed:
        fail(errors, "manifest files cannot be empty")
    for relative, entry in listed.items():
        path = root / relative
        if not path.is_file():
            fail(errors, f"manifest lists missing file {relative}")
            continue
        if path.stat().st_size != entry.get("bytes"):
            fail(errors, f"size mismatch for {relative}")
        if sha256_file(path) != entry.get("sha256"):
            fail(errors, f"sha256 mismatch for {relative}")
    on_disk = {
        str(path.relative_to(root))
        for path in root.rglob("*")
        if path.is_file() and path.name != "manifest.json"
    } - EVALUATOR_FILES
    if on_disk != set(listed):
        fail(errors, f"manifest file set mismatch: {sorted(on_disk ^ set(listed))}")
    bundle = hashlib.sha256(
        "".join(f"{name}:{listed[name]['sha256']}" for name in sorted(listed)).encode()
    ).hexdigest()
    if manifest.get("bundle_sha256") != bundle:
        fail(errors, "manifest bundle_sha256 does not match the bundle")

    index_path = root / "map_index.yaml"
    if index_path.is_file():
        try:
            check_tiles(errors, root, map_yaml, json.loads(index_path.read_text(encoding="utf-8")))
        except json.JSONDecodeError as error:
            fail(errors, f"map_index.yaml is not valid JSON: {error}")

    print_results(errors)
    return 0 if not errors else 1


def print_results(errors):
    if errors:
        for error in errors:
            print(f"MAP_VERIFY_ERROR {error}")
        print(f"MAP_VERIFY ok=false errors={len(errors)}")
    else:
        print("MAP_VERIFY ok=true")


if __name__ == "__main__":
    sys.exit(main())

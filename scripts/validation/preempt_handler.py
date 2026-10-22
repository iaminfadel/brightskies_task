#!/usr/bin/env python3
"""Preemption handler prototype for the #12/HITL boundary.

Modes:
  --simulate-notice N   write the notice file after N seconds (HITL test)
  --notice-file PATH    watch a file that signals a 120 s preemption notice
  --metadata            poll the GCP metadata server preempted endpoint
  --retry               verify the stack on the fresh VM and exit 0

On notice: package partial evidence, upload it to GCS, record the result,
and exit 44 (PREEMPTED) so the caller can retry on a fresh VM.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import tarfile
import threading
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

PREEMPTED_EXIT = 44


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def metadata_preempted() -> bool:
    url = (
        "http://metadata.google.internal/computeMetadata/v1/instance/"
        "preempted?wait=true"
    )
    request = urllib.request.Request(url, headers={"Metadata-Flavor": "Google"})
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            return response.read().decode().strip().lower() == "true"
    except Exception:
        return False


def boot_id() -> str:
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError:
        return "unknown"


def package_evidence(evidence: Path, out: Path) -> None:
    def keep(path: str) -> bool:
        top = path.split("/", 1)[0]
        return top not in ("bags", ".cache", "smoke")

    with tarfile.open(out, "w:gz") as archive:
        for path in evidence.rglob("*"):
            rel = str(path.relative_to(evidence))
            if keep(rel):
                archive.add(path, arcname=rel)


def upload(local: Path, bucket: str, key: str) -> str:
    if shutil.which("gcloud"):
        uri = f"gs://{bucket}/{key}"
        subprocess.run(
            ["gcloud", "storage", "cp", str(local), uri],
            check=True,
            timeout=180,
        )
        return uri
    return "local-only"


def on_preempt(args: argparse.Namespace) -> int:
    evidence = Path(args.evidence_dir)
    evidence.mkdir(parents=True, exist_ok=True)
    run_dir = evidence / "preempt"
    run_dir.mkdir(exist_ok=True)
    partial = run_dir / "partial-evidence.tar.gz"
    package_evidence(evidence, partial)
    uri = upload(
        partial,
        args.bucket,
        f"{args.prefix}/{args.run_id}/partial-evidence.tar.gz",
    )
    result = {
        "status": "PREEMPTED",
        "exit_code": PREEMPTED_EXIT,
        "notice_at": utc_now(),
        "boot_id": boot_id(),
        "partial_evidence": str(partial),
        "partial_upload_uri": uri,
        "retry_required": True,
    }
    (run_dir / "preempt_result.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2))
    return PREEMPTED_EXIT


def watch(args: argparse.Namespace) -> int:
    if args.simulate_notice is not None:
        path = Path(args.notice_file)

        def write_notice() -> None:
            time.sleep(args.simulate_notice)
            path.write_text(utc_now() + "\n", encoding="utf-8")

        threading.Thread(target=write_notice, daemon=True).start()
        deadline = time.monotonic() + args.simulate_notice + 30
        while time.monotonic() < deadline:
            if path.exists():
                return on_preempt(args)
            time.sleep(1)
        print("simulated notice was not observed", file=sys.stderr)
        return 2

    path = Path(args.notice_file)
    while True:
        if path.exists():
            return on_preempt(args)
        if args.metadata and metadata_preempted():
            return on_preempt(args)
        time.sleep(1)


def retry(args: argparse.Namespace) -> int:
    evidence = Path(args.evidence_dir)
    evidence.mkdir(parents=True, exist_ok=True)
    marker = evidence / "retry_observation.json"
    first_boot = "unknown"
    first = evidence / "preempt" / "preempt_result.json"
    if first.exists():
        first_boot = json.loads(first.read_text(encoding="utf-8")).get(
            "boot_id", "unknown"
        )
    current = boot_id()
    checks = {
        "carla_image": subprocess.run(
            ["docker", "image", "inspect", args.carla_image],
            capture_output=True,
            text=True,
        ).returncode
        == 0,
        "deps_image": subprocess.run(
            ["docker", "image", "inspect", args.deps_image],
            capture_output=True,
            text=True,
        ).returncode
        == 0,
        "fresh_boot": first_boot != "unknown" and current != first_boot,
    }
    result = {
        "status": "RETRY_OK" if all(checks.values()) else "RETRY_FAILED",
        "boot_id": current,
        "previous_boot_id": first_boot,
        "checks": checks,
        "retry_at": utc_now(),
    }
    marker.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if all(checks.values()) else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-dir", default="/ws/evidence")
    parser.add_argument("--bucket", default=os.environ.get("BUCKET", ""))
    parser.add_argument("--prefix", default="bags")
    parser.add_argument("--run-id", default=time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))
    parser.add_argument("--notice-file", default="/ws/evidence/preemption_notice")
    parser.add_argument("--simulate-notice", type=int, default=None)
    parser.add_argument("--metadata", action="store_true")
    parser.add_argument("--retry", action="store_true")
    parser.add_argument(
        "--carla-image",
        default=os.environ.get("CARLA_IMAGE", "carlasim/carla:0.9.15"),
    )
    parser.add_argument(
        "--deps-image",
        default=os.environ.get(
            "DEPS_IMAGE", "ghcr.io/iaminfadel/brightdrive-deps:0.9.15-r1"
        ),
    )
    args = parser.parse_args()

    if not args.bucket:
        parser.error("--bucket or BUCKET is required")
    if args.retry:
        return retry(args)
    return watch(args)


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""GCP VM job orchestrator for BrightDrive CI (replaces colab_job.py).

Starts the Spot VM, waits for IAP SSH, syncs the committed repo, runs a
remote command, collects artifacts, and stops the VM unless --keep-vm.
A remote exit code of 44 (PREEMPTED) triggers one retry on a fresh VM.
"""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

# Match infra/common.sh so gcloud's protobuf works on Fedora's Python 3.14.
os.environ.setdefault("PYTHONNOUSERSITE", "1")
if os.path.exists("/usr/bin/python3.11"):
    os.environ.setdefault("CLOUDSDK_PYTHON", "/usr/bin/python3.11")
# Ensure gcloud is on PATH.
for candidate in (os.path.expanduser("~/google-cloud-sdk/bin"), "/usr/lib/google-cloud-sdk/bin"):
    if os.path.isdir(candidate) and candidate not in os.environ.get("PATH", ""):
        os.environ["PATH"] = candidate + os.pathsep + os.environ.get("PATH", "")

PREEMPTED_EXIT = 44
PROJECT = os.environ.get("PROJECT_ID", "project-e038f785-ff95-4654-ba6")
ZONE = os.environ.get("ZONE", "europe-west4-a")
VM = os.environ.get("VM_NAME", "brightdrive-vm")
REPO = Path(__file__).resolve().parents[1]
INFRA = REPO / "infra"


def run(cmd, check=True, capture=False, **kw):
    print("+ " + " ".join(cmd), flush=True)
    return subprocess.run(cmd, check=check, capture_output=capture, text=True, **kw)


def gcloud(*args, check=True, capture=False):
    return run(["gcloud", *args], check=check, capture=capture)


def vm_state():
    try:
        out = gcloud(
            "compute", "instances", "describe", VM, "--zone", ZONE,
            "--format=value(status)", check=False, capture=True,
        ).stdout.strip()
        return out or "MISSING"
    except Exception:
        return "MISSING"


def start_vm():
    st = vm_state()
    if st not in ("RUNNING",):
        if st in ("MISSING",):
            raise SystemExit(f"VM {VM} is MISSING; provision first")
        gcloud("compute", "instances", "start", VM, "--zone", ZONE, "--quiet")
    # wait IAP SSH
    deadline = time.time() + 600
    while time.time() < deadline:
        r = gcloud(
            "compute", "ssh", VM, "--zone", ZONE, "--tunnel-through-iap",
            "--strict-host-key-checking=no", "--quiet",
            "--command=echo ready", check=False, capture=True,
        )
        if r.returncode == 0:
            print("IAP SSH ready", flush=True)
            return
        time.sleep(10)
    raise SystemExit("timed out waiting for IAP SSH")


def stop_vm():
    st = vm_state()
    if st in ("TERMINATED", "STOPPED", "MISSING"):
        return
    gcloud("compute", "instances", "stop", VM, "--zone", ZONE, "--quiet", check=False)


def sync_repo():
    # deploy committed HEAD via git archive over IAP SSH
    sha = subprocess.check_output(
        ["git", "-C", str(REPO), "rev-parse", "HEAD"], text=True
    ).strip()
    print(f"deploying {sha}", flush=True)
    # deploy via tar + scp (gcloud compute scp over IAP)
    tar_path = Path("/tmp/brightdrive.tar")
    with tar_path.open("wb") as f:
        subprocess.run(
            ["git", "-C", str(REPO), "archive", "--format=tar", sha],
            stdout=f, check=True,
        )
    gcloud(
        "compute", "scp", "--tunnel-through-iap", "--zone", ZONE,
        str(tar_path), f"{VM}:/tmp/brightdrive.tar", check=True,
    )
    run([
        "gcloud", "compute", "ssh", VM, "--zone", ZONE, "--tunnel-through-iap",
        "--strict-host-key-checking=no", "--quiet", "--command",
        "sudo rm -rf /ws/brightdrive && sudo mkdir -p /ws/brightdrive && "
        "sudo tar -xf /tmp/brightdrive.tar -C /ws/brightdrive && "
        f"echo {sha} | sudo tee /ws/brightdrive/.deployed-sha >/dev/null",
    ])
    tar_path.unlink(missing_ok=True)


def remote_exec(command):
    r = run([
        "gcloud", "compute", "ssh", VM, "--zone", ZONE, "--tunnel-through-iap",
        "--strict-host-key-checking=no", "--quiet", "--command",
        f"cd /ws/brightdrive && {command}",
    ], check=False)
    return r.returncode


def collect(remote_path, local_dir):
    local_dir = Path(local_dir)
    local_dir.mkdir(parents=True, exist_ok=True)
    run([
        "gcloud", "compute", "scp", "--tunnel-through-iap", "--zone", ZONE,
        "--recurse", f"{VM}:{remote_path}", str(local_dir),
    ], check=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="action", required=True)

    s = sub.add_parser("start")
    s.add_argument("--keep-vm", action="store_true")

    sub.add_parser("stop")

    r = sub.add_parser("run")
    r.add_argument("--command", required=True)
    r.add_argument("--artifact-dir", default="artifacts")
    r.add_argument("--collect", default="/ws/evidence")
    r.add_argument("--keep-vm", action="store_true")
    r.add_argument("--max-retries", type=int, default=1)

    args = p.parse_args()

    if args.action == "start":
        start_vm()
        sync_repo()
        print("VM ready")
        return 0

    if args.action == "stop":
        stop_vm()
        print("VM stopped")
        return 0

    # run
    attempts = 0
    while attempts <= args.max_retries:
        attempts += 1
        try:
            start_vm()
        except SystemExit as e:
            print(e, file=sys.stderr)
            return 1
        sync_repo()
        rc = remote_exec(args.command)
        collect(args.collect, args.artifact_dir)
        if rc != PREEMPTED_EXIT or attempts > args.max_retries:
            if not args.keep_vm:
                stop_vm()
            print(f"remote exit code: {rc}")
            return rc
        print("preempted; retrying on a fresh VM", flush=True)
        stop_vm()
        time.sleep(15)
    if not args.keep_vm:
        stop_vm()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

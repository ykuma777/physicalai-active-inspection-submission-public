#!/usr/bin/env python3
"""Change only the task-owned gauge entity; truth metadata is outside the runtime AI."""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml
from target_model import build_sdf


ROOT = Path(__file__).resolve().parents[1]


def run(argv: list[str], timeout: int = 15) -> str:
    completed = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
    if completed.returncode:
        raise RuntimeError(f"{argv[0]} failed ({completed.returncode}): {completed.stdout} {completed.stderr}")
    return completed.stdout


def entity_poses(world: str) -> dict[str, list[float]]:
    raw = run(["ign", "topic", "-e", "-t", f"/world/{world}/pose/info", "-n", "1", "--json-output"], 12)
    poses = {}
    # Gazebo can emit several adjacent JSON messages for one topic read.
    first_message, _ = json.JSONDecoder().raw_decode(raw.lstrip())
    for item in first_message.get("pose", []):
        pos = item.get("position", {})
        poses[item.get("name", "")] = [float(pos.get(k, 0.0)) for k in ("x", "y", "z")]
    return poses


def main() -> None:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--state", choices=["normal", "abnormal"])
    group.add_argument("--angle-rad", type=float)
    parser.add_argument("--truth-label", choices=["normal", "abnormal"])
    args = parser.parse_args()
    config = yaml.safe_load((ROOT / "configs/runtime.yaml").read_text(encoding="utf-8"))
    if os.environ.get("ROS_DOMAIN_ID") != str(config["ros_domain_id"]):
        raise RuntimeError(f"refusing to change target outside ROS_DOMAIN_ID={config['ros_domain_id']}")
    dataset = yaml.safe_load((ROOT / "configs/dataset.yaml").read_text(encoding="utf-8"))
    world = str(config["world"])
    entity = str(config["target_entity"])
    if args.state:
        angle = float(dataset["runtime_states"][args.state])
        truth = args.state
    else:
        angle = float(args.angle_rad)
        truth = args.truth_label
    target_file = ROOT / "generated/target.sdf"
    target_file.parent.mkdir(parents=True, exist_ok=True)
    target_file.write_text(build_sdf(config, angle), encoding="utf-8")
    run(["ign", "sdf", "-k", str(target_file)])
    services = run(["ign", "service", "-l"], 8).splitlines()
    for service in (f"/world/{world}/remove", f"/world/{world}/create"):
        if service not in services:
            raise RuntimeError(f"Gazebo service absent: {service}")
    if entity in entity_poses(world):
        output = run([
            "ign", "service", "-s", f"/world/{world}/remove",
            "--reqtype", "ignition.msgs.Entity", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "3000", "--req", f'name: "{entity}", type: MODEL',
        ], 8)
        if "true" not in output.lower():
            raise RuntimeError(f"target removal was not confirmed: {output}")
        for _ in range(20):
            if entity not in entity_poses(world):
                break
            time.sleep(0.2)
        else:
            raise RuntimeError("target still exists after removal; no respawn attempted")
    xyz = [float(x) for x in config["target_bottom_world"]]
    run([
        "ros2", "run", "ros_gz_sim", "create", "-world", world,
        "-file", str(target_file), "-name", entity,
        "-x", str(xyz[0]), "-y", str(xyz[1]), "-z", str(xyz[2]),
    ], 15)
    poses = entity_poses(world)
    if entity not in poses:
        raise RuntimeError("target create returned but entity is absent")
    if max(abs(a-b) for a,b in zip(poses[entity], xyz)) > 0.002:
        raise RuntimeError(f"target world pose mismatch: actual={poses[entity]}, expected={xyz}")
    truth_record = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "entity": entity,
        "needle_angle_rad": angle,
        "world_bottom_xyz": poses[entity],
        "truth_label": truth,
        "sdf_path": str(target_file),
    }
    truth_path = ROOT / "results/ground_truth_current.json"
    truth_path.parent.mkdir(parents=True, exist_ok=True)
    truth_path.write_text(json.dumps(truth_record, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(truth_record, sort_keys=True))


if __name__ == "__main__":
    main()

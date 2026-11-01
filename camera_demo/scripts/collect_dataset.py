#!/usr/bin/env python3
"""Capture labeled Gazebo camera frames; split by needle-angle groups."""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    runtime_path = ROOT / "configs/runtime.yaml"
    dataset_path = ROOT / "configs/dataset.yaml"
    runtime = yaml.safe_load(runtime_path.read_text(encoding="utf-8"))
    if os.environ.get("ROS_DOMAIN_ID") != str(runtime["ros_domain_id"]):
        raise RuntimeError(f"dataset collector must run in ROS_DOMAIN_ID={runtime['ros_domain_id']}")
    dataset = yaml.safe_load(dataset_path.read_text(encoding="utf-8"))
    preflight = subprocess.run(
        ["bash", str(ROOT / "scripts/preflight_camera.sh"), "--expected-pose", "observe"],
        capture_output=True, text=True, timeout=35,
    )
    if preflight.returncode:
        raise RuntimeError(f"preflight failed; no dataset collected: {preflight.stdout} {preflight.stderr}")
    label_names = ("normal", "abnormal")
    train_indices = set(dataset["train_angle_indices"])
    validation_indices = set(dataset["validation_angle_indices"])
    if train_indices & validation_indices or train_indices | validation_indices != set(range(len(dataset["normal_angles_rad"]))):
        raise ValueError("train/validation angle groups must be disjoint and complete")
    if len(dataset["normal_angles_rad"]) != len(dataset["abnormal_angles_rad"]):
        raise ValueError("normal and abnormal angle counts differ")
    output_dir = ROOT / "data/images"
    if output_dir.exists() and any(output_dir.rglob("*.png")):
        raise FileExistsError(f"dataset images already exist; refusing to overwrite: {output_dir}")
    rclpy.init(args=[])
    node = rclpy.create_node("camera_dataset_collector")
    bridge = CvBridge()
    newest = {"frame": None, "stamp": None, "arrival": 0.0}

    def on_image(msg):
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        newest["frame"] = frame.copy()
        newest["stamp"] = {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)}
        newest["arrival"] = time.monotonic()

    node.create_subscription(Image, runtime["camera_topic"], on_image, qos_profile_sensor_data)
    rows = []
    try:
        for label in label_names:
            for angle_index, angle in enumerate(dataset[f"{label}_angles_rad"]):
                split = "train" if angle_index in train_indices else "validation"
                switch = subprocess.run(
                    [sys.executable, str(ROOT / "scripts/set_target_state.py"),
                     "--angle-rad", str(angle), "--truth-label", label],
                    capture_output=True, text=True, timeout=30,
                )
                if switch.returncode:
                    raise RuntimeError(f"target switch failed at {label}/{angle_index}: {switch.stdout} {switch.stderr}")
                switch_finished = time.monotonic()
                captured = 0
                deadline = time.monotonic() + 15
                last_saved_stamp = None
                while captured < int(dataset["frames_per_angle"]):
                    if time.monotonic() > deadline:
                        raise TimeoutError(f"camera frames missing at {label}/{angle_index}")
                    rclpy.spin_once(node, timeout_sec=0.1)
                    stamp = newest["stamp"]
                    if newest["frame"] is None or newest["arrival"] < switch_finished + 0.35 or stamp == last_saved_stamp:
                        continue
                    frame = newest["frame"]
                    if frame.shape != (runtime["image_height"], runtime["image_width"], 3) or frame.std() < runtime["minimum_image_std"]:
                        raise ValueError(f"camera image invalid at {label}/{angle_index}")
                    image_path = output_dir / split / label / f"{label}_{angle_index:02d}_{captured:02d}.png"
                    image_path.parent.mkdir(parents=True, exist_ok=True)
                    if not cv2.imwrite(str(image_path), frame):
                        raise IOError(f"could not save {image_path}")
                    rows.append({
                        "relative_path": str(image_path.relative_to(ROOT)).replace("\\", "/"),
                        "label": label, "angle_rad": float(angle), "angle_index": angle_index,
                        "split": split, "camera_stamp": stamp,
                        "sha256": sha256(image_path),
                    })
                    last_saved_stamp = stamp
                    captured += 1
                print(f"captured {split}/{label} angle={angle:.3f}: {captured} images", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    manifest = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": "Gazebo arm-mounted /inspection_camera/image after observe-pose preflight",
        "seed": int(dataset["seed"]),
        "dataset_config_sha256": sha256(dataset_path),
        "runtime_config_sha256": sha256(runtime_path),
        "split_unit": "needle angle group; no angle appears in both train and validation",
        "rows": rows,
    }
    manifest_path = ROOT / "data/dataset_manifest.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"dataset_manifest={manifest_path} rows={len(rows)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

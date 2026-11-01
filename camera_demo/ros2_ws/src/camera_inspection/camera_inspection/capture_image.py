"""Save a new ROS 2 camera frame and its acquisition timestamp."""
from __future__ import annotations

import argparse
import hashlib
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class FrameReceiver(Node):
    def __init__(self, topic: str):
        super().__init__("inspection_camera_capture")
        self.bridge = CvBridge()
        self.frame: np.ndarray | None = None
        self.header_stamp: dict[str, int] | None = None
        self.encoding: str | None = None
        self.received_at_utc: str | None = None
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, message: Image) -> None:
        if self.frame is not None:
            return
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if frame.ndim != 3 or frame.shape[2] != 3 or not frame.size:
            raise ValueError("camera frame is empty or not BGR")
        self.frame = frame.copy()
        self.encoding = message.encoding
        self.header_stamp = {"sec": int(message.header.stamp.sec),
                             "nanosec": int(message.header.stamp.nanosec)}
        self.received_at_utc = utc_now()


def capture(topic: str, path: Path, timeout_sec: float, min_std: float) -> dict:
    rclpy.init(args=[])
    node = FrameReceiver(topic)
    started_at_utc = utc_now()
    start = time.monotonic()
    try:
        while node.frame is None and time.monotonic() - start < timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.frame is None:
            raise TimeoutError(f"no new ROS 2 camera frame on {topic} within {timeout_sec}s")
        frame = node.frame
        deviation = float(frame.std())
        if not np.isfinite(frame).all() or deviation < min_std:
            raise ValueError(f"invalid or nearly blank camera frame: std={deviation:.3f}")
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), frame):
            raise IOError(f"could not save camera image: {path}")
        metadata = {
            "topic": topic,
            "subscription_started_at_utc": started_at_utc,
            "received_at_utc": node.received_at_utc,
            "header_stamp": node.header_stamp,
            "source_encoding": node.encoding,
            "saved_encoding": "BGR8 PNG",
            "shape_hwc": list(frame.shape),
            "pixel_std": deviation,
            "image_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "image_path": str(path),
        }
        path.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        return metadata
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/inspection_camera/image")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--minimum-image-std", type=float, default=4.0)
    args = parser.parse_args()
    print(json.dumps(capture(args.topic, args.output, args.timeout_sec, args.minimum_image_std), sort_keys=True))


if __name__ == "__main__":
    main()

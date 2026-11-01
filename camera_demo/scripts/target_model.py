#!/usr/bin/env python3
"""Generate one monochrome needle-gauge SDF. State labels are kept outside the model."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
from xml.sax.saxutils import escape

import yaml


ROOT = Path(__file__).resolve().parents[1]


def build_sdf(config: dict, angle_rad: float) -> str:
    if not math.isfinite(angle_rad) or abs(angle_rad) > 1.25:
        raise ValueError("needle angle must be finite and within +/-1.25 rad")
    length = 0.032
    needle_y = 0.5 * length * math.sin(angle_rad)
    needle_z = 0.077 + 0.5 * length * math.cos(angle_rad)
    entity = escape(str(config["target_entity"]))
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{entity}">
    <static>true</static>
    <link name="body">
      <collision name="housing_collision">
        <pose>0 0 0.06 0 0 0</pose>
        <geometry><box><size>0.07 0.07 0.12</size></box></geometry>
      </collision>
      <visual name="housing">
        <pose>0 0 0.06 0 0 0</pose>
        <geometry><box><size>0.07 0.07 0.12</size></box></geometry>
        <material><ambient>0.22 0.24 0.27 1</ambient><diffuse>0.22 0.24 0.27 1</diffuse></material>
      </visual>
      <visual name="dial_rim">
        <pose>-0.0362 0 0.077 0 1.5707963 0</pose>
        <geometry><cylinder><radius>0.033</radius><length>0.002</length></cylinder></geometry>
        <material><ambient>0.04 0.04 0.04 1</ambient><diffuse>0.04 0.04 0.04 1</diffuse></material>
      </visual>
      <visual name="dial_face">
        <pose>-0.0380 0 0.077 0 1.5707963 0</pose>
        <geometry><cylinder><radius>0.029</radius><length>0.001</length></cylinder></geometry>
        <material><ambient>0.95 0.95 0.93 1</ambient><diffuse>0.95 0.95 0.93 1</diffuse></material>
      </visual>
      <visual name="needle">
        <pose>-0.0400 {needle_y:.7f} {needle_z:.7f} {-angle_rad:.7f} 0 0</pose>
        <geometry><box><size>0.003 0.004 {length:.5f}</size></box></geometry>
        <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
      </visual>
      <visual name="pivot">
        <pose>-0.0415 0 0.077 0 1.5707963 0</pose>
        <geometry><cylinder><radius>0.004</radius><length>0.002</length></cylinder></geometry>
        <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle-rad", type=float, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "generated/target.sdf")
    args = parser.parse_args()
    config = yaml.safe_load((ROOT / "configs/runtime.yaml").read_text(encoding="utf-8"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(build_sdf(config, args.angle_rad), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Measure, never assume, local image response to two small joint probes."""
from __future__ import annotations
import hashlib
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import cv2
import yaml
import rclpy
from detection import detect
from motion import Motion, ROOT, now, write


def capture(motion, rd, name, arrival, limits):
    path = rd / (name + '.png')
    command = ['ros2', 'run', 'camera_inspection', 'capture_image', '--topic',
               motion.cfg['camera_topic'], '--output', str(path), '--timeout-sec',
               str(motion.cfg['image_timeout_sec']), '--minimum-image-std', str(motion.cfg['minimum_image_std'])]
    proc = subprocess.run(command, capture_output=True, text=True, timeout=30)
    if proc.returncode:
        raise RuntimeError('camera capture failed: ' + proc.stderr[-250:])
    info = json.loads(path.with_suffix('.json').read_text())
    stamp, image_stamp = arrival['joint_state_header']['stamp'], info['header_stamp']
    if info['received_at_utc'] <= arrival['at_utc'] or (image_stamp['sec'], image_stamp['nanosec']) <= (stamp['sec'], stamp['nanosec']):
        raise RuntimeError('stale ROS image after movement')
    if hashlib.sha256(path.read_bytes()).hexdigest() != info['image_sha256']:
        raise RuntimeError('image changed after capture')
    found = detect(cv2.imread(str(path)), limits)
    if not found['detected']:
        raise RuntimeError('dial undetected after calibration probe: ' + found['reason'])
    return {'image': info, 'detection': found, 'path': str(path)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--baseline-run-id', required=True)
    ap.add_argument('--run-id', required=True)
    args = ap.parse_args()
    active = yaml.safe_load((ROOT / 'configs/control.yaml').read_text())
    controls = active['control']
    baseline_path = ROOT / 'results/runs' / args.baseline_run_id / 'run_manifest.json'
    baseline = json.loads(baseline_path.read_text())
    if baseline['result'] != 'PASS' or baseline['detection']['detected'] is not True:
        raise RuntimeError('baseline pilot not valid')
    rd = ROOT / 'results/runs' / args.run_id
    rd.mkdir(parents=True, exist_ok=False)
    m = {'run_id': rd.name, 'started_at_utc': now(), 'result': 'RUNNING', 'motions': [],
         'probes': {}, 'derivative_pixels_per_rad': None, 'error': None}
    def persist(): write(rd / 'run_manifest.json', m)
    motion = None
    try:
        rclpy.init()
        motion = Motion()
        motion.register_scene(rd / 'collision_scene.json')
        center = list(motion.cfg['poses']['scan_center'])
        if max(abs(a-b) for a, b in zip(motion.wait_sample(), center)) > .02:
            raise RuntimeError('robot not at calibrated scan_center')
        for axis in (0, 1):
            goal = center.copy()
            goal[axis] += float(controls['joint_probe_rad'])
            key = f'probe_joint{axis+1}'
            motion.cfg['poses'][key] = goal
            entry = motion.move('scan_center', key, rd, m, persist)
            time.sleep(.4)
            m['probes'][key] = capture(motion, rd, key, entry['arrival'], active['capture_quality'])
            persist()
            motion.move(key, 'scan_center', rd, m, persist)
        base = baseline['detection']
        features = lambda d: d['center_error_xy_px'] + [d['dial_width_px']]
        m['derivative_pixels_per_rad'] = [[(a-b)/float(controls['joint_probe_rad']) for a,b in zip(features(m['probes'][f'probe_joint{axis+1}']['detection']),features(base))] for axis in (0,1)]
        motion.move('scan_center', 'home', rd, m, persist)
        m['result'] = 'PASS'
        persist()
        artifact = {'schema_version': 1, 'method': 'measured +0.035 rad joint probes from scan_center',
                    'source_run_id': args.run_id, 'baseline_run_id': args.baseline_run_id,
                    'baseline_manifest_sha256': hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
                    'derivative_pixels_per_rad': m['derivative_pixels_per_rad']}
        (ROOT / 'artifacts').mkdir(exist_ok=True)
        write(ROOT / 'artifacts/image_jacobian.json', artifact)
        print(json.dumps({'result': m['result'], 'derivative_pixels_per_rad': m['derivative_pixels_per_rad']}))
        return 0
    except Exception as exc:
        m['result'] = 'FAIL'; m['error'] = f'{type(exc).__name__}: {exc}'
        persist(); print(m['error'], file=sys.stderr)
        return 1
    finally:
        m['finished_at_utc'] = now(); persist()
        if motion: motion.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

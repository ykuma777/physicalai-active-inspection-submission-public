#!/usr/bin/env python3
"""Prepare known Gazebo gauges. Truth lives only in this setup/evaluation process.

No inference, target selection, trajectory publication, or automatic mutation
retries occur here. The runtime coordinator must never import this module.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time
from datetime import datetime, timezone
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / 'camera_demo/scripts/target_model.py'
sys.path.insert(0, str(SOURCE.parent))
from target_model import build_sdf


def utc():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    os.replace(tmp, path)


def command(argv, log, timeout=12):
    started = utc()
    try:
        result = subprocess.run(argv, text=True, capture_output=True, timeout=timeout)
        row = {'started_at_utc': started, 'finished_at_utc': utc(), 'argv': argv,
               'returncode': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr}
    except subprocess.TimeoutExpired as exc:
        row = {'started_at_utc': started, 'finished_at_utc': utc(), 'argv': argv,
               'returncode': None, 'error': 'TIMEOUT_NO_RESEND',
               'stdout': str(exc.stdout), 'stderr': str(exc.stderr)}
        with log.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, ensure_ascii=False) + '\n')
        raise TimeoutError(f'command timed out; no resend: {argv[0]}') from exc
    with log.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(row, ensure_ascii=False) + '\n')
    if result.returncode:
        raise RuntimeError(f'command failed; no resend: {argv[0]}: {result.stderr[-500:]}')
    return result.stdout


def entity_poses(world, log):
    raw = command(['ign', 'topic', '-e', '-t', f'/world/{world}/pose/info',
                   '-n', '1', '--json-output'], log)
    # Adjacent messages are possible. Parse each actual JSON document, never infer
    # an entity pose from the requested preparation values.
    decoder = json.JSONDecoder()
    poses = {}
    text = raw.lstrip()
    while text:
        message, end = decoder.raw_decode(text)
        for entity in message.get('pose', []):
            p, q = entity.get('position', {}), entity.get('orientation', {})
            poses[entity.get('name', '')] = {
                'xyz': [float(p.get(k, 0.0)) for k in ('x', 'y', 'z')],
                'quaternion_xyzw': [float(q.get(k, 1.0 if k == 'w' else 0.0)) for k in ('x', 'y', 'z', 'w')],
            }
        text = text[end:].lstrip()
    return poses


def mutation(world, suffix, reqtype, request, log):
    output = command(['ign', 'service', '-s', f'/world/{world}/{suffix}',
                      '--reqtype', reqtype, '--reptype', 'ignition.msgs.Boolean',
                      '--timeout', '3000', '--req', request], log, timeout=8)
    if not re.search(r'\bdata\s*:\s*true\b', output):
        raise RuntimeError(f'{suffix} response was not true; no resend: {output}')


def make_sdf(entity, angle, target):
    # Reuse the historical appearance/needle geometry and extend its physical
    # collision envelope to exactly match the configured MoveIt box.
    model = ET.fromstring(build_sdf({'target_entity': entity}, angle))
    collision = model.find('./model/link/collision[@name="housing_collision"]')
    center = [float(v) for v in target['collision_center_local']]
    size = [float(v) for v in target['collision_dimensions']]
    if len(center) != 3 or len(size) != 3 or any(not math.isfinite(v) for v in center + size) or any(v <= 0 for v in size):
        raise ValueError('invalid shared gauge collision box')
    collision.find('pose').text = ' '.join(map(str, center)) + ' 0 0 0'
    collision.find('./geometry/box/size').text = ' '.join(map(str, size))
    ET.indent(model, space='  ')
    return ET.tostring(model, encoding='unicode', xml_declaration=True)



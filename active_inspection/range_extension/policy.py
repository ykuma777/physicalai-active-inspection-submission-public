"""Joint-1-only horizontal alignment policies; no ROS or simulator imports.

Only an image detection and measured joints are runtime inputs. World poses,
placement IDs, labels and teacher outputs never enter the learned prediction.
This module does not publish commands or perform fallback to another actor.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

FEATURE_ORDER = ('center_error_x_px', 'joint1_rad')
OUTPUT_ORDER = ('joint1_delta_rad',)
MAX_STEP_RAD = 0.055
SCHEMA_VERSION = 1


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_max_step(max_step):
    limit = float(max_step)
    if not math.isfinite(limit) or not 0 < limit <= MAX_STEP_RAD:
        raise ValueError('joint1 safety step must be positive and <= 0.055 rad')
    return limit


def observation_features(detection, joints):
    if detection.get('detected') is not True or detection.get('fully_in_frame') is not True:
        raise ValueError('one complete detected dial is required')
    if len(joints) != 4:
        raise ValueError('four measured joint positions are required')
    ex, ey = map(float, detection['center_error_xy_px'])
    width = float(detection['dial_width_px'])
    q = list(map(float, joints))
    if not all(math.isfinite(v) for v in [ex, ey, width, *q]):
        raise ValueError('nonfinite image/joint feature')
    if abs(ey) > 35 or not 145 <= width <= 255:
        raise ValueError('horizontal-only policy cannot correct vertical/size quality failure')
    return {'detected': True, 'center_error_x_px': ex, 'center_error_y_px': ey,
            'dial_width_px': width}, q


def features(detection, joints):
    observed, q = observation_features(detection, joints)
    return [observed['center_error_x_px'], q[0]]


def audit_output(actor, raw, detection, joints, max_step):
    limit = validate_max_step(max_step)
    observed, q = observation_features(detection, joints)
    raw = float(raw)
    if not math.isfinite(raw):
        raise ValueError('nonfinite policy output')
    raw4 = [raw, 0.0, 0.0, 0.0]
    excessive = abs(raw) > limit * 2.0
    # Retain the previous model's excessive-output rejection as well as the
    # actual per-operation cap. Rejection never invokes a rule teacher.
    bounded = None if excessive else [max(-limit, min(limit, raw)), 0.0, 0.0, 0.0]
    return {
        'result': 'REJECTED_EXCESSIVE_OUTPUT' if excessive else 'PASS',
        'actor': actor,
        'image_features': observed,
        'model_feature_order': list(FEATURE_ORDER),
        'model_input_features': [observed['center_error_x_px'], q[0]],
        'current_joint_positions_rad': q,
        'raw_output_rad': raw4,
        'raw_model_output_rad': raw4 if actor == 'clone' else None,
        'raw_teacher_output_rad': raw4 if actor == 'rule' else None,
        'safety_limited_output_rad': bounded,
        'limited_joints': [1] if abs(raw) > limit else [],
        'model_call_count_increment': 1 if actor == 'clone' else 0,
        # A teacher DECISION is not yet a valid teacher action record. Arrival
        # and the subsequent fresh image must be verified by the controller.
        'teacher_decision_count_increment': 1 if actor == 'rule' else 0,
        'max_step_rad': limit,
        'fallback_actor': None,
    }


class Joint1RuleTeacher:
    """Fixed image-Jacobian teacher with half-gain toward the image center.

    At the retained measured slope 685.714 px/rad, x errors 42, 64 and115 px
    produce different limited magnitudes (~.0306, .0467, .055 rad). New images
    may yield new decisions after each verified move; this is not a resend.
    """
    def __init__(self, calibration_path=None, *, pixels_per_rad=None, gain=0.5):
        if calibration_path is not None and pixels_per_rad is not None:
            raise ValueError('supply a calibration path or scalar, not both')
        self.calibration_sha256 = None
        if calibration_path is not None:
            data = json.loads(Path(calibration_path).read_text(encoding='utf-8'))
            pixels_per_rad = data['derivative_pixels_per_rad'][0][0]
            self.calibration_sha256 = sha256(calibration_path)
        if pixels_per_rad is None:
            raise ValueError('explicit measured calibration is required')
        self.pixels_per_rad = float(pixels_per_rad)
        self.gain = float(gain)
        if not math.isfinite(self.pixels_per_rad) or abs(self.pixels_per_rad) < 20:
            raise ValueError('unusable measured joint1 image Jacobian')
        if not math.isfinite(self.gain) or not 0 < self.gain <= 1:
            raise ValueError('teacher gain must lie in (0,1]')

    def predict_with_audit(self, detection, joints, max_step=MAX_STEP_RAD):
        observed, _ = observation_features(detection, joints)
        raw = -self.gain * observed['center_error_x_px'] / self.pixels_per_rad
        result = audit_output('rule', raw, detection, joints, max_step)
        result['teacher'] = {'method': 'fixed proportional horizontal image feedback',
                             'gain': self.gain, 'desired_error_x_px': 0.0,
                             'measured_pixels_per_rad': self.pixels_per_rad,
                             'calibration_sha256': self.calibration_sha256}
        return result


class Joint1ClonePolicy:
    def __init__(self, model_path, *, allow_offline_synthetic=False):
        self.path = Path(model_path)
        metadata_path = self.path.with_suffix('.json')
        if not metadata_path.exists():
            metadata_path = self.path.with_name(self.path.stem + '_metadata.json')
        self.metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        if self.metadata.get('offline_synthetic') and not allow_offline_synthetic:
            raise ValueError('synthetic offline-check artifact is not a deployment model')
        if self.metadata.get('model_sha256') != sha256(self.path):
            raise ValueError('model SHA256 differs from metadata')
        if self.metadata.get('feature_order') != list(FEATURE_ORDER):
            raise ValueError('unexpected feature order')
        if self.metadata.get('output_order') != list(OUTPUT_ORDER):
            raise ValueError('unexpected output order; joint2 is not permitted')
        if float(self.metadata.get('safety_max_step_rad', MAX_STEP_RAD)) != MAX_STEP_RAD:
            raise ValueError('model metadata safety limit differs from retained limit')
        with np.load(self.path, allow_pickle=False) as values:
            self.mean = np.asarray(values['mean'], dtype=np.float64)
            self.scale = np.asarray(values['scale'], dtype=np.float64)
            self.weights = np.asarray(values['weights'], dtype=np.float64)
        if self.mean.shape != (2,) or self.scale.shape != (2,) or self.weights.shape != (3,):
            raise ValueError('unexpected horizontal model dimensions')
        if not (np.isfinite(self.mean).all() and np.isfinite(self.scale).all()
                and np.isfinite(self.weights).all() and (self.scale > 0).all()):
            raise ValueError('invalid weights/normalization')
        self.model_sha256 = sha256(self.path)
        self.metadata_sha256 = sha256(metadata_path)

    def predict_with_audit(self, detection, joints, max_step=MAX_STEP_RAD):
        x = np.asarray(features(detection, joints), dtype=np.float64)
        z = (x - self.mean) / self.scale
        raw = float(np.r_[1.0, z] @ self.weights)
        result = audit_output('clone', raw, detection, joints, max_step)
        result['normalized_model_features'] = z.tolist()
        result['model_sha256'] = self.model_sha256
        result['model_metadata_sha256'] = self.metadata_sha256
        return result


class ConstantActionBaseline:
    """Training-label mean only; diagnostic baseline, never a clone fallback."""
    def __init__(self, training_mean_delta_rad):
        self.mean = float(training_mean_delta_rad)
        if not math.isfinite(self.mean):
            raise ValueError('invalid constant baseline')

    def predict_with_audit(self, detection, joints, max_step=MAX_STEP_RAD):
        result = audit_output('constant_baseline', self.mean, detection, joints, max_step)
        result['baseline_definition'] = 'constant mean of training teacher joint1 actions'
        return result

"""Retained HOG/linear-SVM model with a separate, pinned preprocessing contract.

This module accepts image pixels only. It never imports preparation, gauge angle,
truth, target IDs or motion configuration. Original assets remain unmodified.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
CAMERA = REPO / 'camera_demo'
RUNTIME = CAMERA / 'configs/runtime.yaml'
MODEL = CAMERA / 'artifacts/gauge_hog_svm.yml'
METADATA = CAMERA / 'artifacts/gauge_model_metadata.json'
LABELS = {0: 'normal', 1: 'abnormal'}
SETTING_KEYS = ('image_width', 'image_height', 'roi_center_xy', 'roi_size_px',
                'resize_px', 'hog_win_size_px', 'minimum_model_margin')


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical_hash(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    ensure_ascii=True, allow_nan=False).encode()).hexdigest()


def contract(settings: dict) -> dict:
    return {'schema_version': 1,
            'settings': {key: settings[key] for key in SETTING_KEYS},
            'decoder': 'OpenCV IMREAD_COLOR BGR8', 'grayscale': 'COLOR_BGR2GRAY',
            'resize_interpolation': 'INTER_LINEAR',
            'hog': {'block': [16, 16], 'block_stride': [8, 8], 'cell': [8, 8],
                    'bins': 9, 'gamma_correction': False},
            'quality': {'minimum_roi_std': 8.0, 'canny_thresholds': [40, 120],
                        'minimum_canny_edges': 80}}


def verified_contract(preprocessing_path: Path | None = None) -> tuple[dict, dict, dict]:
    metadata = json.loads(METADATA.read_text(encoding='utf-8'))
    if sha256(MODEL) != metadata['model_sha256']:
        raise ValueError('trained model SHA-256 mismatch')
    if sha256(RUNTIME) != metadata['runtime_config_sha256']:
        raise ValueError('original training runtime SHA-256 mismatch')
    settings = yaml.safe_load(RUNTIME.read_text(encoding='utf-8'))
    descriptor = contract(settings)
    if preprocessing_path is not None:
        supplied = json.loads(Path(preprocessing_path).read_text(encoding='utf-8'))
        if canonical_hash(supplied) != canonical_hash(descriptor):
            raise ValueError('preprocessing contract changed; requires a separately trained model')
    return settings, metadata, descriptor


def preprocess(image: np.ndarray, settings: dict) -> tuple[np.ndarray, dict]:
    if image is None or image.dtype != np.uint8 or image.shape != (
            int(settings['image_height']), int(settings['image_width']), 3):
        raise ValueError('expected full-size 640x480 BGR8 ROS image')
    x, y = map(int, settings['roi_center_xy'])
    side = int(settings['roi_size_px'])
    if side % 2:
        raise ValueError('ROI must have an even side length')
    half = side // 2
    area = image[y-half:y+half, x-half:x+half]
    if area.shape != (side, side, 3):
        raise ValueError('ROI lies outside image')
    gray = cv2.cvtColor(area, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (int(settings['resize_px']), int(settings['resize_px'])),
                      interpolation=cv2.INTER_LINEAR)
    roi_std = float(gray.std())
    edges = int(np.count_nonzero(cv2.Canny(gray, 40, 120)))
    if roi_std < 8.0:
        raise ValueError('indeterminate: ROI has insufficient texture')
    if edges < 80:
        raise ValueError('indeterminate: ROI has insufficient gauge edges')
    return gray, {'result': 'PASS', 'roi_std': roi_std, 'roi_canny_edges': edges,
                  'roi_bounds_xyxy': [x-half, y-half, x+half, y+half],
                  'resized_shape': list(gray.shape)}


def classify(image_path: Path, preprocessing_path: Path | None = None) -> dict:
    """Predict from one saved ROS image, checking exact input/model/config hashes.

    SVM raw output is diagnostic only. Label comes from SVM class prediction.
    There is no angle/ground-truth/target-ID input and no automatic observation retry.
    """
    image_path = Path(image_path)
    settings, metadata, descriptor = verified_contract(preprocessing_path)
    image_digest_before = sha256(image_path)
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    gray, quality = preprocess(image, settings)
    side = int(settings['hog_win_size_px'])
    if gray.shape != (side, side):
        raise ValueError('resized ROI and HOG input sizes differ')
    hog = cv2.HOGDescriptor((side, side), (16, 16), (8, 8), (8, 8), 9)
    vector = hog.compute(gray).reshape(1, -1).astype(np.float32)
    if not np.isfinite(vector).all() or vector.shape[1] != int(metadata['feature_length']):
        raise ValueError('invalid HOG feature width or non-finite values')
    svm = cv2.ml.SVM_load(str(MODEL))
    if svm.empty() or svm.getKernelType() != cv2.ml.SVM_LINEAR:
        raise ValueError('expected trained linear SVM')
    _, output = svm.predict(vector)
    _, margin_output = svm.predict(vector, flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)
    raw_id = float(output[0, 0])
    predicted_id = int(raw_id)
    raw_margin = float(margin_output[0, 0])
    if raw_id != predicted_id or predicted_id not in LABELS or not math.isfinite(raw_margin):
        raise ValueError('unexpected SVM class or non-finite decision value')
    if abs(raw_margin) < float(settings['minimum_model_margin']):
        raise ValueError('indeterminate: SVM decision value below configured minimum')
    if sha256(image_path) != image_digest_before:
        raise ValueError('input image changed during inference')
    return {'label': LABELS[predicted_id], 'image_sha256': image_digest_before,
            'input_image': str(image_path), 'model_sha256': metadata['model_sha256'],
            'model_version': metadata['model_version'],
            'runtime_config_sha256': metadata['runtime_config_sha256'],
            'preprocessing_sha256': canonical_hash(descriptor),
            'preprocessing_contract': descriptor, 'quality': quality,
            'svm_raw_margin': raw_margin,
            'svm_margin_semantics': 'uncalibrated signed SVM decision value; not probability or anomaly score',
            'opencv_version': cv2.__version__, 'numpy_version': np.__version__}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path)
    parser.add_argument('--preprocessing', type=Path)
    parser.add_argument('--show-contract', action='store_true')
    args = parser.parse_args()
    if args.show_contract:
        _, _, descriptor = verified_contract(args.preprocessing)
        print(json.dumps({'contract': descriptor, 'sha256': canonical_hash(descriptor)}, indent=2))
    elif args.image:
        print(json.dumps(classify(args.image, args.preprocessing), indent=2))
    else:
        parser.error('--image or --show-contract required')


if __name__ == '__main__':
    main()
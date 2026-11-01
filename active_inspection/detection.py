"""Image-only dial detection and registration for the retained HOG/SVM.

No world position, needle angle or ground-truth label is an input here.
The dark rim is a deliberately designed visual feature of the existing gauge,
not an added fiducial. A detection is never inferred from the background.
"""
from __future__ import annotations

import cv2
import numpy as np


CANONICAL_DIAL = (202.0, 245.0, 236.0, 118.0)


def detect(image: np.ndarray, limits: dict) -> dict:
    if image is None or image.shape != (480, 640, 3) or image.dtype != np.uint8:
        raise ValueError("expected one 640x480 BGR8 ROS camera image")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = (gray < int(limits["dark_threshold"])).astype(np.uint8)
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    candidates = []
    for x, y, w, h, area in stats[1:]:
        if not (int(limits["min_dial_width_px"]) <= w <= int(limits["max_dial_width_px"])):
            continue
        if h < 25 or not (1.45 <= w / h <= 2.6):
            continue
        if not (0.12 <= area / (w * h) <= 0.65):
            continue
        # Partial rim and image-border fragments are not acceptable detections.
        if x < 8 or y < 8 or x + w > 632 or y + h > 472:
            continue
        candidates.append((int(x), int(y), int(w), int(h), int(area)))
    if len(candidates) != 1:
        return {"detected": False, "reason": f"expected one complete dark dial rim, found {len(candidates)}", "candidates": candidates}
    x, y, w, h, area = candidates[0]
    cx, cy = x + w / 2.0, y + h / 2.0
    desired = limits["desired_center_xy"]
    return {"detected": True, "dial_bbox_xywh": [x, y, w, h],
            "center_xy": [cx, cy], "center_error_xy_px": [cx - desired[0], cy - desired[1]],
            "dial_width_px": w, "dark_rim_pixels": area,
            "fully_in_frame": True}


def quality(detection: dict, limits: dict) -> dict:
    reasons = []
    if not detection["detected"]:
        reasons.append(detection["reason"])
    else:
        x, y = detection["center_error_xy_px"]
        if abs(x) > limits["max_center_error_px"] or abs(y) > limits["max_center_error_px"]:
            reasons.append("dial center outside tolerance")
        w = detection["dial_width_px"]
        if not limits["quality_width_px"][0] <= w <= limits["quality_width_px"][1]:
            reasons.append("dial size outside tolerance")
    return {"result": "PASS" if not reasons else "FAIL", "reasons": reasons}


def register_for_legacy_model(image: np.ndarray, detection: dict) -> np.ndarray:
    """Warp the detected rim into the retained model's canonical view.

    The source image remains unchanged and must be saved with its ROS stamp.
    Classification uses this registered image, whose SHA-256 is also recorded.
    """
    if not detection["detected"]:
        raise ValueError("cannot register an undetected dial")
    x, y, w, h = detection["dial_bbox_xywh"]
    dx, dy, dw, dh = CANONICAL_DIAL
    sx, sy = dw / w, dh / h
    transform = np.array([[sx, 0.0, dx - sx * x], [0.0, sy, dy - sy * y]], dtype=np.float32)
    return cv2.warpAffine(image, transform, (640, 480), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=(185, 183, 178))

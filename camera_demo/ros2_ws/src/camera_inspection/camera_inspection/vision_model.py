"""Grayscale HOG + linear SVM trained on labeled Gazebo camera images."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


ROOT = Path("/home/ubuntu/camera_demo")
LABEL_TO_ID = {"normal": 0, "abnormal": 1}
ID_TO_LABEL = {value: key for key, value in LABEL_TO_ID.items()}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def config() -> dict:
    return yaml.safe_load((ROOT / "configs/runtime.yaml").read_text(encoding="utf-8"))


def crop(image: np.ndarray, settings: dict) -> np.ndarray:
    if image is None or image.shape != (settings["image_height"], settings["image_width"], 3):
        raise ValueError("image must be a full-size BGR frame from the configured ROS camera")
    x, y = map(int, settings["roi_center_xy"])
    half = int(settings["roi_size_px"]) // 2
    area = image[y-half:y+half, x-half:x+half]
    if area.shape != (2*half, 2*half, 3):
        raise ValueError("ROI lies outside camera image")
    gray = cv2.cvtColor(area, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (settings["resize_px"], settings["resize_px"]))
    if float(gray.std()) < 8:
        raise ValueError("ROI has almost no visual texture; target may be out of view")
    if int(np.count_nonzero(cv2.Canny(gray, 40, 120))) < 80:
        raise ValueError("ROI has too few edges for a visible needle gauge")
    return gray


def feature(image: np.ndarray, settings: dict) -> np.ndarray:
    gray = crop(image, settings)
    side = int(settings["hog_win_size_px"])
    if gray.shape != (side, side):
        raise ValueError("HOG input size and resized ROI must agree")
    hog = cv2.HOGDescriptor((side,side), (16,16), (8,8), (8,8), 9)
    values = hog.compute(gray).reshape(-1).astype(np.float32)
    if not np.isfinite(values).all():
        raise ValueError("non-finite HOG feature")
    return values


def read_image(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"cannot read camera image: {path}")
    return image


def augment(image: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    h,w = image.shape[:2]
    angle = float(rng.uniform(-2.0, 2.0))
    move_x = float(rng.uniform(-3.0, 3.0))
    move_y = float(rng.uniform(-3.0, 3.0))
    matrix = cv2.getRotationMatrix2D((w/2,h/2), angle, 1.0)
    matrix[0,2] += move_x
    matrix[1,2] += move_y
    warped = cv2.warpAffine(image, matrix, (w,h), borderMode=cv2.BORDER_REFLECT)
    alpha = float(rng.uniform(0.92, 1.08))
    beta = float(rng.uniform(-7, 7))
    return np.clip(warped.astype(np.float32)*alpha+beta, 0, 255).astype(np.uint8)


def train(manifest_path: Path) -> dict:
    settings = config()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    dataset_cfg = yaml.safe_load((ROOT / "configs/dataset.yaml").read_text(encoding="utf-8"))
    rng = np.random.default_rng(int(dataset_cfg["seed"]))
    train_features, train_labels, validation = [], [], []
    train_groups, validation_groups = set(), set()
    for row in manifest["rows"]:
        path = ROOT / row["relative_path"]
        if sha256(path) != row["sha256"]:
            raise ValueError(f"dataset image hash changed: {path}")
        label = row["label"]
        group = (label, row["angle_index"])
        image = read_image(path)
        if row["split"] == "train":
            train_groups.add(group)
            for variant in (image, *(augment(image, rng) for _ in range(3))):
                train_features.append(feature(variant, settings))
                train_labels.append(LABEL_TO_ID[label])
        elif row["split"] == "validation":
            validation_groups.add(group)
            validation.append((row, feature(image, settings)))
        else:
            raise ValueError(f"unknown split: {row['split']}")
    if train_groups & validation_groups or not train_features or not validation:
        raise ValueError("train/validation angle groups overlap or are empty")
    matrix = np.stack(train_features).astype(np.float32)
    labels = np.asarray(train_labels, dtype=np.int32)
    svm = cv2.ml.SVM_create()
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setKernel(cv2.ml.SVM_LINEAR)
    svm.setC(float(dataset_cfg["svm_c"]))
    svm.setTermCriteria((cv2.TERM_CRITERIA_MAX_ITER, 1000, 1e-6))
    if not svm.train(matrix, cv2.ml.ROW_SAMPLE, labels):
        raise RuntimeError("OpenCV SVM training failed")
    predictions = []
    for row, vector in validation:
        _, output = svm.predict(vector.reshape(1,-1))
        predicted = ID_TO_LABEL[int(output[0,0])]
        predictions.append({
            "image": row["relative_path"], "angle_index": row["angle_index"],
            "truth": row["label"], "predicted": predicted,
            "pass": predicted == row["label"],
        })
    accuracy = sum(item["pass"] for item in predictions)/len(predictions)
    if accuracy < float(dataset_cfg["min_validation_accuracy"]):
        raise RuntimeError(f"held-out-angle accuracy {accuracy:.3f} below required threshold")
    model_path = ROOT / "artifacts/gauge_hog_svm.yml"
    model_path.parent.mkdir(parents=True, exist_ok=True)
    svm.save(str(model_path))
    metadata = {
        "model_version": "grayscale_hog_linear_svm_v1",
        "algorithm": "OpenCV HOG descriptor + supervised linear C-SVC",
        "classes": ID_TO_LABEL,
        "opencv_version": cv2.__version__,
        "numpy_version": np.__version__,
        "seed": int(dataset_cfg["seed"]),
        "train_rows_after_augmentation": int(len(train_labels)),
        "validation_rows": len(validation),
        "held_out_angle_accuracy": accuracy,
        "dataset_manifest_sha256": sha256(manifest_path),
        "model_sha256": sha256(model_path),
        "runtime_config_sha256": sha256(ROOT / "configs/runtime.yaml"),
        "feature_length": int(matrix.shape[1]),
    }
    metadata_path = ROOT / "artifacts/gauge_model_metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    evaluation = {
        "split_unit": "needle angle group",
        "train_angle_groups": sorted([list(x) for x in train_groups]),
        "validation_angle_groups": sorted([list(x) for x in validation_groups]),
        "validation_accuracy": accuracy,
        "per_image": predictions,
    }
    evaluation_path = ROOT / "results/model_evaluation.json"
    evaluation_path.parent.mkdir(parents=True, exist_ok=True)
    evaluation_path.write_text(json.dumps(evaluation, indent=2) + "\n", encoding="utf-8")
    return {"metadata": metadata, "evaluation": evaluation, "model_path": str(model_path)}


def classify(image_path: Path) -> dict:
    settings = config()
    model_path = Path(settings["model_path"])
    metadata_path = Path(settings["model_metadata_path"])
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if sha256(model_path) != metadata["model_sha256"]:
        raise ValueError("trained image model hash mismatch")
    if sha256(ROOT / "configs/runtime.yaml") != metadata["runtime_config_sha256"]:
        raise ValueError("runtime preprocessing config changed after training")
    vector = feature(read_image(image_path), settings)
    if len(vector) != metadata["feature_length"]:
        raise ValueError("HOG feature width differs from trained model")
    svm = cv2.ml.SVM_load(str(model_path))
    _, output = svm.predict(vector.reshape(1,-1))
    _, margin_output = svm.predict(vector.reshape(1,-1), flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)
    predicted_id = int(output[0,0])
    if predicted_id not in ID_TO_LABEL:
        raise ValueError(f"unexpected SVM class: {predicted_id}")
    margin = abs(float(margin_output[0,0]))
    if margin < float(settings["minimum_model_margin"]):
        raise ValueError(f"SVM decision margin too small: {margin}")
    return {
        "label": ID_TO_LABEL[predicted_id],
        "svm_raw_margin": float(margin_output[0,0]),
        "svm_abs_margin": margin,
        "image_sha256": sha256(image_path),
        "model_sha256": metadata["model_sha256"],
        "model_version": metadata["model_version"],
        "runtime_config_sha256": metadata["runtime_config_sha256"],
        "input_image": str(image_path),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    action = parser.add_subparsers(dest="action", required=True)
    train_parser = action.add_parser("train")
    train_parser.add_argument("--manifest", type=Path, default=ROOT / "data/dataset_manifest.json")
    predict_parser = action.add_parser("predict")
    predict_parser.add_argument("--image", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(train(args.manifest) if args.action == "train" else classify(args.image), indent=2))


if __name__ == "__main__":
    main()

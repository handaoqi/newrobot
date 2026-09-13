"""Licensed-model adapter for offline static LiDAR scene semantics.

The runner intentionally keeps model execution separate from mapping.  PTv3
exports may differ in input/output names, so the adapter accepts the common
``[N, C]`` and ``[1, N, C]`` ONNX forms and leaves the trained checkpoint out
of the repository.
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Callable

import numpy as np

LOGGER = logging.getLogger(__name__)

SCHEMA = "roamerx.scene-semantics.v1"
LABELS = ("road", "tree", "building", "wall", "natural_ground", "low_vegetation", "other")
AUTOWARE_LABELS = (
    "car", "truck", "bus", "bicycle", "pedestrian", "traffic_cone",
    "barrier", "debris", "drivable_flat", "non_drivable_flat", "vegetation",
    "building", "vertical_thin", "static_clutter", "noise",
)
ASSET_CATEGORIES = {
    "road", "tree", "building", "wall", "barrier", "traffic_cone",
    "vegetation", "debris", "drivable_flat", "non_drivable_flat", "vertical_thin",
}


class SceneSemanticError(RuntimeError):
    pass


def _pcd_header(path: Path) -> tuple[int, dict[str, str], int]:
    values: dict[str, str] = {}
    header_size = 0
    with path.open("rb") as stream:
        while header_size < 64 * 1024:
            line = stream.readline()
            if not line:
                break
            header_size += len(line)
            text = line.decode("ascii", errors="ignore").strip()
            if text and not text.startswith("#"):
                key, _, value = text.partition(" ")
                values[key.upper()] = value.strip()
            if text.upper().startswith("DATA "):
                return header_size, values, int(values.get("POINTS") or values.get("WIDTH") or 0)
    raise SceneSemanticError(f"PCD header is invalid: {path}")


def read_pcd_xyz_intensity(path: Path, max_points: int) -> np.ndarray:
    """Read a bounded PCD sample as float32 ``x,y,z,intensity``."""
    header_size, header, point_count = _pcd_header(path)
    if point_count <= 0:
        raise SceneSemanticError("PCD contains no points")
    fields = header.get("FIELDS", "").split()
    sizes = [int(value) for value in header.get("SIZE", "").split()]
    types = header.get("TYPE", "").split()
    counts = [int(value) for value in header.get("COUNT", "").split()] or [1] * len(fields)
    if len(fields) != len(sizes) or len(fields) != len(types) or len(fields) != len(counts):
        raise SceneSemanticError("PCD fields are inconsistent")
    wanted = [fields.index(name) for name in ("x", "y", "z") if name in fields]
    if len(wanted) != 3:
        raise SceneSemanticError("PCD must contain x, y and z")
    intensity_index = fields.index("intensity") if "intensity" in fields else None
    data_kind = header.get("DATA", "").lower()
    stride = max(1, math.ceil(point_count / max(1, max_points)))
    if data_kind == "ascii":
        raw = np.loadtxt(path, dtype=np.float32, skiprows=header_size and sum(1 for _ in _header_lines(path)))
        raw = np.atleast_2d(raw)[::stride]
        values = raw[:, wanted]
        intensity = raw[:, intensity_index] if intensity_index is not None else np.zeros(len(values), dtype=np.float32)
        return np.column_stack((values, intensity)).astype(np.float32, copy=False)
    if data_kind != "binary":
        raise SceneSemanticError("only ASCII and binary PCD are supported")
    dtype_fields = []
    type_map = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}
    for index, (name, size, kind, count) in enumerate(zip(fields, sizes, types, counts)):
        np_kind = type_map.get((kind.upper(), size))
        if not np_kind or count != 1:
            raise SceneSemanticError(f"unsupported PCD field: {name}")
        dtype_fields.append((f"f{index}", "<" + np_kind))
    data = np.fromfile(path, dtype=np.dtype(dtype_fields), offset=header_size, count=point_count)[::stride]
    if len(data) != math.ceil(point_count / stride):
        raise SceneSemanticError("PCD binary payload is truncated")
    values = np.column_stack([data[f"f{index}"] for index in wanted]).astype(np.float32)
    intensity = data[f"f{intensity_index}"].astype(np.float32) if intensity_index is not None else np.zeros(len(values), dtype=np.float32)
    return np.column_stack((values, intensity))


def _header_lines(path: Path):
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                return
            yield line
            if line.decode("ascii", errors="ignore").strip().upper().startswith("DATA "):
                return


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _softmax(logits: np.ndarray) -> np.ndarray:
    logits = logits.astype(np.float32, copy=False)
    logits -= logits.max(axis=1, keepdims=True)
    values = np.exp(logits)
    return values / np.maximum(values.sum(axis=1, keepdims=True), 1e-8)


class OnnxSemanticModel:
    def __init__(self, model_path: Path, label_count: int = len(LABELS)):
        try:
            import onnxruntime as ort
        except ImportError as exc:  # pragma: no cover - depends on deployed image
            raise SceneSemanticError("onnxruntime is not installed") from exc
        if not model_path.is_file():
            raise SceneSemanticError(f"PTv3 ONNX model not found: {model_path}")
        self.label_count = int(label_count)
        if self.label_count <= 0:
            raise SceneSemanticError("PTv3 label count must be positive")
        providers = ["TensorrtExecutionProvider", "CUDAExecutionProvider", "CPUExecutionProvider"]
        available = ort.get_available_providers()
        self.session = ort.InferenceSession(model_path.as_posix(), providers=[p for p in providers if p in available])
        if not self.session.get_inputs():
            raise SceneSemanticError("PTv3 ONNX model has no inputs")
        self.input_name = self.session.get_inputs()[0].name

    def __call__(self, features: np.ndarray) -> np.ndarray:
        input_meta = self.session.get_inputs()[0]
        shape = input_meta.shape
        value = features[None, ...] if len(shape) == 3 else features
        output = self.session.run(None, {self.input_name: value})[0]
        output = np.asarray(output)
        if output.ndim == 3:
            output = output[0]
        if output.ndim == 1:
            labels = output.astype(np.int64)
            result = np.zeros((len(labels), self.label_count), dtype=np.float32)
            result[np.arange(len(labels)), np.clip(labels, 0, self.label_count - 1)] = 1.0
            return result
        if output.ndim != 2:
            raise SceneSemanticError(f"unsupported PTv3 output shape: {output.shape}")
        if output.shape[0] != len(features) and output.shape[1] == len(features):
            output = output.T
        if output.shape[0] != len(features):
            raise SceneSemanticError(f"PTv3 output point count mismatch: {output.shape}")
        return _softmax(output)


def _catalog(path: Path) -> list[dict]:
    if not path.is_file():
        raise SceneSemanticError(f"scene asset catalog not found: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != "roamerx.scene-assets.v1":
        raise SceneSemanticError("scene asset catalog schema mismatch")
    return [item for item in payload.get("assets", []) if isinstance(item, dict)]


def _cluster(labels: np.ndarray, points: np.ndarray, class_index: int, voxel_size: float) -> list[np.ndarray]:
    selected = np.flatnonzero(labels == class_index)
    if not len(selected):
        return []
    cells = np.floor(points[selected, :3] / max(voxel_size, 0.01)).astype(np.int64)
    cell_to_points: dict[tuple[int, int, int], list[int]] = {}
    for point_index, cell in zip(selected, cells):
        cell_to_points.setdefault(tuple(cell), []).append(int(point_index))
    unvisited = set(cell_to_points)
    clusters = []
    while unvisited:
        start = unvisited.pop()
        stack = [start]
        members = list(cell_to_points[start])
        while stack:
            cell = stack.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        neighbour = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
                        if neighbour in unvisited:
                            unvisited.remove(neighbour)
                            stack.append(neighbour)
                            members.extend(cell_to_points[neighbour])
        clusters.append(np.asarray(members, dtype=np.int64))
    return clusters


def _asset_match(category: str, dimensions: np.ndarray, assets: list[dict]) -> tuple[str | None, float]:
    normalized = str(category or "").strip().lower().replace("-", "_")
    aliases = {
        "drivable_flat": "road", "non_drivable_flat": "road", "vertical_thin": "wall",
        "traffic_cone": "traffic_cone", "groundcover": "vegetation", "static_clutter": "debris",
    }
    catalog_category = aliases.get(normalized, normalized)
    candidates = [
        item for item in assets
        if str(item.get("category") or "").strip().lower().replace("-", "_") == catalog_category
    ]
    if not candidates:
        return None, 0.0
    target = np.maximum(dimensions, 0.05)
    scores = []
    for item in candidates:
        dims = item.get("dimensions_m") or {}
        reference = np.array([float(dims.get(axis, 0.05)) for axis in ("x", "y", "z")], dtype=np.float32)
        ratio = np.maximum(target, reference) / np.maximum(np.minimum(target, reference), 0.05)
        scores.append(float(np.exp(-np.mean(np.log(ratio)))))
    index = int(np.argmax(scores))
    return str(candidates[index].get("asset_id")), scores[index]


def run_scene_semantics(
    map_dir: str | Path,
    *,
    model_path: str | Path,
    model_version: str,
    confidence_threshold: float = 0.80,
    min_support_frames: int = 3,
    voxel_size_m: float = 0.10,
    max_points: int = 1_000_000,
    asset_catalog_path: str | Path = "/home/dogrobot/platform/frontend/public/scene-assets/catalog.json",
    predictor: Callable[[np.ndarray], np.ndarray] | None = None,
    map_sha256: str = "",
    label_names: tuple[str, ...] = LABELS,
) -> dict:
    root = Path(map_dir).expanduser().resolve()
    cloud = next((root / name for name in ("scene_preview.pcd", "map.pcd") if (root / name).is_file()), None)
    if cloud is None:
        raise SceneSemanticError("complete map has no map.pcd or scene_preview.pcd")
    points = read_pcd_xyz_intensity(cloud, max_points)
    if not len(points):
        raise SceneSemanticError("complete map has no usable points")
    ground = float(np.percentile(points[:, 2], 2))
    features = np.column_stack((points[:, :3], points[:, 3], points[:, 2] - ground)).astype(np.float32)
    predictor = predictor or OnnxSemanticModel(Path(model_path).expanduser(), len(label_names))
    probabilities = []
    chunk_size = 65536
    for start in range(0, len(features), chunk_size):
        probabilities.append(np.asarray(predictor(features[start:start + chunk_size]), dtype=np.float32))
    probs = np.concatenate(probabilities, axis=0)
    if probs.shape != (len(points), len(label_names)):
        raise SceneSemanticError(f"model probabilities have invalid shape: {probs.shape}")
    labels = probs.argmax(axis=1)
    assets = _catalog(Path(asset_catalog_path).expanduser())
    support_frames = len(list((root / "keyframes").glob("scan_*.pcd")))
    instances = []
    candidates = []
    for class_index, class_name in enumerate(label_names):
        if class_name not in ASSET_CATEGORIES:
            continue
        for ordinal, member_indices in enumerate(_cluster(labels, points, class_index, voxel_size_m)):
            if len(member_indices) < 3:
                continue
            cloud_points = points[member_indices, :3]
            minimum, maximum = cloud_points.min(axis=0), cloud_points.max(axis=0)
            dimensions = maximum - minimum
            asset_id, shape_score = _asset_match(class_name, dimensions, assets)
            confidence = float(np.mean(probs[member_indices, class_index]))
            confidence = float(0.75 * confidence + 0.25 * shape_score)
            center = (minimum + maximum) / 2.0
            instance = {
                "id": f"{class_name}-{ordinal + 1:04d}",
                "asset_id": asset_id or class_name,
                "class_name": class_name,
                "position": {axis: float(value) for axis, value in zip(("x", "y", "z"), center)},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "scale": 1.0,
                "confidence": confidence,
                "support_frames": support_frames,
                "bbox": {"min": minimum.tolist(), "max": maximum.tolist()},
                "source": "ai",
                "review_state": "generated" if confidence >= confidence_threshold and support_frames >= min_support_frames and asset_id else "pending",
            }
            candidates.append(instance)
            if instance["review_state"] == "generated":
                instances.append(instance)
    return {
        "schema": SCHEMA,
        "model_version": model_version,
        "map_sha256": map_sha256,
        "source": {"cloud": cloud.name, "cloud_sha256": _sha256(cloud), "point_count": int(len(points)), "feature_schema": "xyz_intensity_above_ground", "labels": list(label_names)},
        "instances": instances,
        "review_candidates": [item for item in candidates if item["review_state"] == "pending"],
        "metrics": {"point_count": int(len(points)), "candidate_count": len(candidates), "accepted_count": len(instances)},
        "status": "ready" if instances else "review",
    }


def run_autoware_scene_semantics(
    map_dir: str | Path,
    *,
    bundle_path: str | Path,
    plugin_path: str | Path = "",
    inference_binary_path: str | Path = "",
    model_version: str,
    confidence_threshold: float = 0.80,
    min_support_frames: int = 3,
    max_points: int = 1_000_000,
    enable_detection: bool = True,
    asset_catalog_path: str | Path = "/home/dogrobot/platform/frontend/public/scene-assets/catalog.json",
    map_sha256: str = "",
) -> dict:
    """Run the Autoware PTv3 encoder and segmentation TensorRT chain.

    CUDA preprocessing and TensorRT execution live in the C++ runner.  This
    wrapper only normalizes its voxel-cluster output into the platform sidecar
    and applies the same confidence/support gate as the generic runner.
    """
    root = Path(map_dir).expanduser().resolve()
    bundle = Path(bundle_path).expanduser().resolve()
    cloud = next((root / name for name in ("scene_preview.pcd", "map.pcd") if (root / name).is_file()), None)
    if cloud is None:
        raise SceneSemanticError("complete map has no map.pcd or scene_preview.pcd")
    if not bundle.is_dir():
        raise SceneSemanticError(f"Autoware PTv3 bundle not found: {bundle}")

    binary_candidates = [Path(inference_binary_path).expanduser()] if inference_binary_path else []
    binary_candidates.extend(Path(path) for path in (
        "/home/dogrobot/runtime/nx-edge/install/ptv3/bin/roamerx_ptv3_cli",
        "/home/dogrobot/runtime/nx-edge/install/ptv3/lib/roamerx_ptv3_cli",
    ))
    binary = next((path for path in binary_candidates if path.is_file()), None)
    if binary is None:
        raise SceneSemanticError("Autoware PTv3 TensorRT runner is not installed")

    plugin_candidates = [Path(plugin_path).expanduser()] if plugin_path else []
    plugin_candidates.extend(Path(path) for path in (
        "/home/dogrobot/runtime/nx-edge/install/ptv3/lib/libautoware_tensorrt_plugins.so",
        "/usr/local/lib/libautoware_tensorrt_plugins.so",
    ))
    plugin = next((path for path in plugin_candidates if path.is_file()), None)
    if plugin is None:
        raise SceneSemanticError("Autoware PTv3 TensorRT plugin is not installed")

    try:
        _, _, point_count = _pcd_header(cloud)
    except (OSError, ValueError) as exc:
        raise SceneSemanticError(f"PCD header is invalid: {cloud}") from exc

    fd, raw_path = tempfile.mkstemp(prefix=".ptv3-", suffix=".json", dir=str(root))
    os.close(fd)
    try:
        environment = os.environ.copy()
        trt_lib = "/opt/TensorRT-10.6.0.26/targets/aarch64-linux-gnu/lib"
        environment["LD_LIBRARY_PATH"] = ":".join(filter(None, (
            str(plugin.parent), trt_lib, environment.get("LD_LIBRARY_PATH", ""))))
        command = [
            str(binary), "--pcd", str(cloud), "--bundle", str(bundle),
            "--plugin", str(plugin), "--output", raw_path, "--max-points", str(max_points),
        ]
        if enable_detection:
            command.append("--with-detection")
        completed = subprocess.run(
            command, env=environment, capture_output=True, text=True, timeout=7200, check=False)
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout or "PTv3 runner failed").strip()
            raise SceneSemanticError(detail[-2000:])
        raw = json.loads(Path(raw_path).read_text(encoding="utf-8"))
    except subprocess.TimeoutExpired as exc:
        raise SceneSemanticError("Autoware PTv3 TensorRT inference timed out") from exc
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise SceneSemanticError(f"PTv3 runner output is invalid: {exc}") from exc
    finally:
        try:
            os.unlink(raw_path)
        except OSError:
            pass

    assets = {str(item.get("asset_id")): item for item in _catalog(Path(asset_catalog_path).expanduser())}
    support_frames = len(list((root / "keyframes").glob("scan_*.pcd")))
    accepted = []
    review = []
    for item in raw.get("instances", []) if isinstance(raw, dict) else []:
        if not isinstance(item, dict):
            continue
        asset_id = str(item.get("asset_id") or "")
        confidence = float(item.get("confidence") or 0.0)
        normalized = dict(item)
        normalized["support_frames"] = support_frames
        normalized["map_sha256"] = map_sha256
        normalized["review_state"] = (
            "generated" if confidence >= confidence_threshold
            and support_frames >= min_support_frames and asset_id in assets else "pending"
        )
        (accepted if normalized["review_state"] == "generated" else review).append(normalized)
    return {
        "schema": SCHEMA,
        "model_version": model_version,
        "map_sha256": map_sha256,
        "source": {
            "cloud": cloud.name,
            "cloud_sha256": _sha256(cloud),
            "point_count": int(point_count),
            "feature_schema": "autoware_ptv3_xyzi_voxelized",
            "labels": list(AUTOWARE_LABELS),
        },
        "instances": accepted,
        "dynamic_detections": raw.get("detections", []) if isinstance(raw, dict) and isinstance(raw.get("detections"), list) else [],
        "review_candidates": review,
        "metrics": {
            "point_count": int(point_count),
            "candidate_count": len(accepted) + len(review),
            "accepted_count": len(accepted),
            "detection_count": len(raw.get("detections", [])) if isinstance(raw, dict) and isinstance(raw.get("detections"), list) else 0,
        },
        "status": "ready" if accepted else "review",
    }


def write_scene_semantics(path: str | Path, payload: dict) -> None:
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{destination.name}.", suffix=".tmp", dir=str(destination.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)

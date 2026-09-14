from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from bike_bot.config import ModelConfig
from bike_bot.detector import COCO80_CLASS_NAMES, CPU_PROVIDER, CUDA_PROVIDER, RawDetection, VEHICLE_ALERT_CLASSES, YoloDetector, parse_yolo_predictions


def _parse(predictions, **overrides):
    kwargs = {
        "frame_h": 720,
        "frame_w": 1280,
        "scale": 0.5,
        "pad_x": 10,
        "pad_y": 20,
        "confidence": 0.6,
        "nms_iou_threshold": 0.45,
        "class_names": ["person", "bicycle"],
    }
    kwargs.update(overrides)
    return parse_yolo_predictions(predictions, **kwargs)


def _box_row(cx, cy, width, height, scores) -> np.ndarray:
    return np.array([cx, cy, width, height, *scores], dtype=np.float32)


def test_multiclass_keeps_high_confidence_bicycle() -> None:
    predictions = np.zeros((2, 6), dtype=np.float32)
    predictions[0] = _box_row(100, 100, 40, 40, [0.1, 0.95])
    predictions[1] = _box_row(300, 300, 40, 40, [0.2, 0.2])

    detections = _parse(predictions)

    assert len(detections) == 1
    assert detections[0].label == "bicycle"
    assert detections[0].confidence == pytest.approx(0.95, abs=1e-6)
    assert detections[0].bbox == (140, 120, 80, 80)


def test_below_threshold_is_dropped() -> None:
    predictions = np.zeros((1, 6), dtype=np.float32)
    predictions[0] = _box_row(100, 100, 40, 40, [0.1, 0.4])

    assert _parse(predictions) == []


def test_single_class_uses_class_zero() -> None:
    predictions = np.zeros((1, 5), dtype=np.float32)
    predictions[0] = np.array([80, 60, 20, 10, 0.88], dtype=np.float32)

    detections = _parse(predictions, class_names=["bicycle"])

    assert len(detections) == 1
    assert detections[0].label == "bicycle"
    assert detections[0].confidence == pytest.approx(0.88, abs=1e-6)
    assert detections[0].bbox == (120, 70, 40, 20)


def test_channel_first_layout_is_transposed() -> None:
    predictions = np.zeros((6, 2), dtype=np.float32)
    predictions[:, 0] = _box_row(100, 100, 40, 40, [0.1, 0.91])
    predictions[:, 1] = _box_row(400, 400, 20, 20, [0.05, 0.05])

    detections = _parse(predictions)

    assert len(detections) == 1
    assert detections[0].label == "bicycle"
    assert detections[0].bbox == (140, 120, 80, 80)


def test_nms_drops_overlapping_duplicate() -> None:
    predictions = np.zeros((2, 6), dtype=np.float32)
    predictions[0] = _box_row(100, 100, 40, 40, [0.1, 0.99])
    predictions[1] = _box_row(102, 101, 40, 40, [0.1, 0.90])

    detections = _parse(predictions)

    assert len(detections) == 1
    assert detections[0].confidence == pytest.approx(0.99, abs=1e-6)


def test_unknown_class_id_falls_back_to_index() -> None:
    predictions = np.zeros((1, 7), dtype=np.float32)
    predictions[0] = _box_row(100, 100, 40, 40, [0.1, 0.1, 0.97])

    detections = _parse(predictions, class_names=["person", "bicycle"])

    assert len(detections) == 1
    assert detections[0].label == "2"


def test_coco_vehicle_indexes_keep_their_real_labels() -> None:
    predictions = np.zeros((3, 84), dtype=np.float32)
    for class_id in (1, 2, 3):
        scores = [0.0] * 80
        scores[class_id] = 0.95
        predictions[class_id - 1] = _box_row(100 * class_id, 100, 40, 40, scores)

    detections = _parse(predictions, class_names=list(COCO80_CLASS_NAMES))

    assert {item.label for item in detections} == {"bicycle", "car", "motorcycle"}
    assert {item.label for item in detections} == VEHICLE_ALERT_CLASSES


class _FakeOrt:
    def __init__(self, providers):
        self._providers = providers

    def get_available_providers(self):
        return list(self._providers)


def test_cpu_fallback_provider_requires_explicit_permission() -> None:
    detector = object.__new__(YoloDetector)
    detector.model_config = ModelConfig(path="model.onnx", allow_cpu_fallback=False)

    with pytest.raises(RuntimeError, match="allow_cpu_fallback is false"):
        detector._cuda_cpu_providers(_FakeOrt([CPU_PROVIDER]))


def test_cuda_provider_remains_available_when_cpu_fallback_is_disabled() -> None:
    detector = object.__new__(YoloDetector)
    detector.model_config = ModelConfig(path="model.onnx", allow_cpu_fallback=False)

    assert detector._cuda_cpu_providers(_FakeOrt([CUDA_PROVIDER, CPU_PROVIDER])) == [
        CUDA_PROVIDER
    ]


@pytest.mark.parametrize(
    ("candidate", "expected"),
    [
        (RawDetection("bicycle", (0, 0, 40, 40), 0.8), "passed_single_frame"),
        (RawDetection("car", (0, 0, 40, 40), 0.5), "below_confidence"),
        (RawDetection("motorcycle", (0, 0, 20, 20), 0.8), "below_min_box_area"),
        (None, "not_detected"),
    ],
)
def test_single_frame_diagnostic_explains_each_gate(candidate, expected) -> None:
    detector = object.__new__(YoloDetector)
    detector.model_config = SimpleNamespace(path="model.onnx", image_size=512, confidence=0.6)
    detector.config = SimpleNamespace(
        detection=SimpleNamespace(min_box_area=1600, event_confirm_frames=3, event_cooldown_seconds=10)
    )
    detector.last_timing = SimpleNamespace(providers="TensorrtExecutionProvider")
    detector._predict = lambda frame, confidence=None: [] if candidate is None else [candidate]

    result, _ = detector.diagnose_image(np.zeros((100, 100, 3), dtype=np.uint8))

    assert result["result_code"] == expected
    if candidate is not None:
        assert result["detected_class"] == candidate.label


def test_single_frame_diagnostic_returns_every_vehicle_alert_box() -> None:
    detector = object.__new__(YoloDetector)
    detector.model_config = SimpleNamespace(path="model.onnx", image_size=512, confidence=0.6)
    detector.config = SimpleNamespace(
        detection=SimpleNamespace(min_box_area=1600, event_confirm_frames=3, event_cooldown_seconds=10)
    )
    detector.last_timing = SimpleNamespace(providers="TensorrtExecutionProvider")
    detector._predict = lambda frame, confidence=None: [
        RawDetection("bicycle", (0, 0, 40, 40), 0.92),
        RawDetection("motorcycle", (45, 0, 40, 40), 0.88),
        RawDetection("person", (80, 0, 40, 40), 0.99),
    ]

    result, _ = detector.diagnose_image(np.zeros((100, 160, 3), dtype=np.uint8))

    assert result["detected_class"] == "bicycle"
    assert [item["detected_class"] for item in result["detections"]] == ["bicycle", "motorcycle"]

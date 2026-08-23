from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from bike_bot.detector import parse_yolo_predictions


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

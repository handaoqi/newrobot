from __future__ import annotations

import time
from dataclasses import dataclass


BBoxTuple = tuple[int, int, int, int]


@dataclass
class TrackingDetection:
    bbox: BBoxTuple
    label: str
    confidence: float


@dataclass
class TrackedObject:
    track_id: str
    bbox: BBoxTuple
    label: str
    confidence: float
    first_seen_at: float
    last_seen_at: float


class IoUTracker:
    """Small dependency-free tracker for stable edge-board deployments."""

    def __init__(
        self,
        *,
        iou_threshold: float = 0.3,
        track_ttl_seconds: float = 30.0,
        duplicate_alert_seconds: float = 300.0,
    ) -> None:
        self.iou_threshold = iou_threshold
        self.track_ttl_seconds = track_ttl_seconds
        self.duplicate_alert_seconds = duplicate_alert_seconds
        self._tracks: dict[str, TrackedObject] = {}
        self._last_alert_at: dict[str, float] = {}
        self._next_id = 1

    def update(self, detections: list[TrackingDetection], now: float | None = None) -> list[TrackedObject]:
        now = time.time() if now is None else now
        self._expire_old_tracks(now)

        matches: list[TrackedObject] = []
        unused_track_ids = set(self._tracks)

        for detection in sorted(detections, key=lambda item: item.confidence, reverse=True):
            track = self._match_existing_track(detection, unused_track_ids)
            if track is None:
                track = self._create_track(detection, now)
            else:
                unused_track_ids.discard(track.track_id)
                track.bbox = detection.bbox
                track.label = detection.label
                track.confidence = detection.confidence
                track.last_seen_at = now
            matches.append(track)

        return matches

    def should_alert(self, track_id: str, now: float | None = None) -> bool:
        now = time.time() if now is None else now
        last_alert_at = self._last_alert_at.get(track_id)
        if last_alert_at is not None and now - last_alert_at < self.duplicate_alert_seconds:
            return False
        self._last_alert_at[track_id] = now
        return True

    def _match_existing_track(
        self,
        detection: TrackingDetection,
        candidate_track_ids: set[str],
    ) -> TrackedObject | None:
        best_track: TrackedObject | None = None
        best_iou = 0.0

        for track_id in candidate_track_ids:
            track = self._tracks[track_id]
            if track.label != detection.label:
                continue
            score = bbox_iou(track.bbox, detection.bbox)
            if score > best_iou:
                best_iou = score
                best_track = track

        if best_iou < self.iou_threshold:
            return None
        return best_track

    def _create_track(self, detection: TrackingDetection, now: float) -> TrackedObject:
        track_id = f"bike-{self._next_id:06d}"
        self._next_id += 1
        track = TrackedObject(
            track_id=track_id,
            bbox=detection.bbox,
            label=detection.label,
            confidence=detection.confidence,
            first_seen_at=now,
            last_seen_at=now,
        )
        self._tracks[track_id] = track
        return track

    def _expire_old_tracks(self, now: float) -> None:
        expired = [
            track_id
            for track_id, track in self._tracks.items()
            if now - track.last_seen_at > self.track_ttl_seconds
        ]
        for track_id in expired:
            self._tracks.pop(track_id, None)
            self._last_alert_at.pop(track_id, None)


def bbox_iou(first: BBoxTuple, second: BBoxTuple) -> float:
    first_x1, first_y1, first_w, first_h = first
    second_x1, second_y1, second_w, second_h = second
    first_x2 = first_x1 + first_w
    first_y2 = first_y1 + first_h
    second_x2 = second_x1 + second_w
    second_y2 = second_y1 + second_h

    inter_x1 = max(first_x1, second_x1)
    inter_y1 = max(first_y1, second_y1)
    inter_x2 = min(first_x2, second_x2)
    inter_y2 = min(first_y2, second_y2)
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    intersection = inter_w * inter_h
    if intersection <= 0:
        return 0.0

    first_area = max(0, first_w) * max(0, first_h)
    second_area = max(0, second_w) * max(0, second_h)
    union = first_area + second_area - intersection
    if union <= 0:
        return 0.0
    return intersection / union

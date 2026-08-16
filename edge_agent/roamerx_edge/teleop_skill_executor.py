"""Deterministic, local execution for composable remote-control skills."""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

from .protocol import ProtocolError


PRESET_SKILLS = {
    "prone_forward_5s": {
        "description": "匍匐前进 5 秒",
        "category": "timed_motion",
        "requires_live_pose": False,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "action", "action": "lie_down"},
            {"kind": "velocity", "vx": 0.35, "duration_seconds": 5.0},
        ],
    },
    "prone_backward_5s": {
        "description": "匍匐后退 5 秒",
        "category": "timed_motion",
        "requires_live_pose": False,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "action", "action": "lie_down"},
            {"kind": "velocity", "vx": -0.35, "duration_seconds": 5.0},
        ],
    },
    "micro_reverse_5s": {
        "description": "微速后退 5 秒",
        "category": "timed_motion",
        "requires_live_pose": False,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "speed", "level": "micro"},
            {"kind": "velocity", "vx": -0.35, "duration_seconds": 5.0},
        ],
    },
    "micro_forward_5s": {
        "description": "微速前进 5 秒",
        "category": "timed_motion",
        "requires_live_pose": False,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "speed", "level": "micro"},
            {"kind": "velocity", "vx": 0.35, "duration_seconds": 5.0},
        ],
    },
    "prone_forward_5m": {
        "description": "匍匐前进 5 米",
        "category": "distance_motion",
        "requires_live_pose": True,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "action", "action": "lie_down"},
            {"kind": "distance", "axis": "forward", "distance_m": 5.0, "speed_mps": 0.35},
        ],
    },
    "prone_backward_5m": {
        "description": "匍匐后退 5 米",
        "category": "distance_motion",
        "requires_live_pose": True,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "action", "action": "lie_down"},
            {"kind": "distance", "axis": "backward", "distance_m": 5.0, "speed_mps": 0.35},
        ],
    },
    "micro_forward_5m": {
        "description": "微速前进 5 米",
        "category": "distance_motion",
        "requires_live_pose": True,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "speed", "level": "micro"},
            {"kind": "distance", "axis": "forward", "distance_m": 5.0, "speed_mps": 0.35},
        ],
    },
    "micro_reverse_5m": {
        "description": "微速后退 5 米",
        "category": "distance_motion",
        "requires_live_pose": True,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "speed", "level": "micro"},
            {"kind": "distance", "axis": "backward", "distance_m": 5.0, "speed_mps": 0.35},
        ],
    },
    "turn_left_full_circle": {
        "description": "原地左转一圈",
        "category": "turn",
        "requires_live_pose": True,
        "obstacle_protection": "none",
        "steps": [{"kind": "turn", "angle_rad": math.tau, "yaw_rate": 0.45}],
    },
    "turn_right_full_circle": {
        "description": "原地右转一圈",
        "category": "turn",
        "requires_live_pose": True,
        "obstacle_protection": "none",
        "steps": [{"kind": "turn", "angle_rad": -math.tau, "yaw_rate": 0.45}],
    },
    "left_two_steps_then_avoid_forward": {
        "description": "左移两步后避障前进",
        "category": "front_avoidance",
        "requires_live_pose": True,
        "obstacle_protection": "front_only",
        "steps": [
            {"kind": "distance", "axis": "left", "distance_m": 0.4, "speed_mps": 0.35},
            {"kind": "avoid_forward", "duration_seconds": 3.0, "speed_mps": 0.35},
        ],
    },
    "right_two_steps_then_avoid_forward": {
        "description": "右移两步后避障前进",
        "category": "front_avoidance",
        "requires_live_pose": True,
        "obstacle_protection": "front_only",
        "steps": [
            {"kind": "distance", "axis": "right", "distance_m": 0.4, "speed_mps": 0.35},
            {"kind": "avoid_forward", "duration_seconds": 3.0, "speed_mps": 0.35},
        ],
    },
    "turn_left_and_forward_detour": {
        "description": "左转并前进绕行",
        "category": "detour",
        "requires_live_pose": True,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "turn", "angle_rad": math.pi / 2, "yaw_rate": 0.45},
            {"kind": "distance", "axis": "forward", "distance_m": 0.8, "speed_mps": 0.4},
            {"kind": "turn", "angle_rad": -math.pi / 2, "yaw_rate": 0.45},
            {"kind": "distance", "axis": "forward", "distance_m": 1.0, "speed_mps": 0.4},
        ],
    },
    "turn_right_and_forward_detour": {
        "description": "右转并前进绕行",
        "category": "detour",
        "requires_live_pose": True,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "turn", "angle_rad": -math.pi / 2, "yaw_rate": 0.45},
            {"kind": "distance", "axis": "forward", "distance_m": 0.8, "speed_mps": 0.4},
            {"kind": "turn", "angle_rad": math.pi / 2, "yaw_rate": 0.45},
            {"kind": "distance", "axis": "forward", "distance_m": 1.0, "speed_mps": 0.4},
        ],
    },
    "turn_left_and_backward_detour": {
        "description": "左转并后退绕行（无后向避障）",
        "category": "detour",
        "requires_live_pose": True,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "turn", "angle_rad": math.pi / 2, "yaw_rate": 0.45},
            {"kind": "distance", "axis": "backward", "distance_m": 0.8, "speed_mps": 0.4},
            {"kind": "turn", "angle_rad": -math.pi / 2, "yaw_rate": 0.45},
            {"kind": "distance", "axis": "backward", "distance_m": 1.0, "speed_mps": 0.4},
        ],
    },
    "turn_right_and_backward_detour": {
        "description": "右转并后退绕行（无后向避障）",
        "category": "detour",
        "requires_live_pose": True,
        "obstacle_protection": "none",
        "steps": [
            {"kind": "turn", "angle_rad": -math.pi / 2, "yaw_rate": 0.45},
            {"kind": "distance", "axis": "backward", "distance_m": 0.8, "speed_mps": 0.4},
            {"kind": "turn", "angle_rad": math.pi / 2, "yaw_rate": 0.45},
            {"kind": "distance", "axis": "backward", "distance_m": 1.0, "speed_mps": 0.4},
        ],
    },
}


@dataclass
class SkillRun:
    command_id: str
    description: str
    status: str = "running"
    started_at: float = field(default_factory=time.monotonic)
    completed_steps: int = 0
    total_steps: int = 0
    error: str = ""
    cancel_event: threading.Event = field(default_factory=threading.Event)

    def snapshot(self) -> dict:
        return {
            "command_id": self.command_id,
            "description": self.description,
            "status": self.status,
            "completed_steps": self.completed_steps,
            "total_steps": self.total_steps,
            "elapsed_seconds": round(time.monotonic() - self.started_at, 2),
            "error": self.error or None,
        }


class TeleopSkillExecutor:
    """Runs a single skill locally while retaining explicit cancellation state."""

    REPEAT_SECONDS = 0.15

    def __init__(self, navigation) -> None:
        self.navigation = navigation
        self._lock = threading.Lock()
        self._runs: dict[str, SkillRun] = {}
        self._active_id: str | None = None

    def start(self, command_id: str, command: dict, completed: Callable[[dict], None]) -> dict:
        plan = self._normalize_plan(command)
        with self._lock:
            if self._active_id:
                active = self._runs.get(self._active_id)
                if active and active.status == "running":
                    raise ProtocolError("SKILL_BUSY", f"skill {self._active_id} is still running")
            run = SkillRun(command_id=command_id, description=plan["description"], total_steps=len(plan["steps"]))
            self._runs[command_id] = run
            self._active_id = command_id
        threading.Thread(
            target=self._run, args=(run, plan["steps"], completed), daemon=True, name=f"teleop-skill-{command_id[:8]}"
        ).start()
        return run.snapshot()

    def status(self, command_id: str) -> dict:
        with self._lock:
            run = self._runs.get(command_id)
            if not run:
                raise ProtocolError("SKILL_NOT_FOUND", f"no skill run for command {command_id}")
            return run.snapshot()

    def cancel(self, command_id: str) -> dict:
        with self._lock:
            run = self._runs.get(command_id)
            if not run:
                raise ProtocolError("SKILL_NOT_FOUND", f"no skill run for command {command_id}")
            run.cancel_event.set()
            return run.snapshot()

    @staticmethod
    def list_presets() -> list[dict]:
        """Return the executable preset catalog without exposing raw steps."""
        return [
            {
                "name": name,
                "description": template["description"],
                "category": template["category"],
                "requires_live_pose": template["requires_live_pose"],
                "obstacle_protection": template["obstacle_protection"],
            }
            for name, template in PRESET_SKILLS.items()
        ]

    def _normalize_plan(self, command: dict) -> dict:
        preset = str(command.get("preset") or "").strip()
        if preset:
            template = PRESET_SKILLS.get(preset)
            if not template:
                raise ProtocolError("SKILL_PRESET_UNKNOWN", f"unknown preset {preset}")
            return {"description": template["description"], "steps": [dict(item) for item in template["steps"]]}
        steps = command.get("steps")
        if not isinstance(steps, list) or not steps:
            raise ProtocolError("SKILL_STEPS_INVALID", "provide a preset or a non-empty steps array")
        if len(steps) > 32:
            raise ProtocolError("SKILL_STEPS_INVALID", "at most 32 steps are supported")
        normalized = []
        for index, step in enumerate(steps):
            if not isinstance(step, dict) or not step.get("kind"):
                raise ProtocolError("SKILL_STEPS_INVALID", f"step {index} has no kind")
            normalized.append(dict(step))
        return {"description": str(command.get("description") or "AI组合动作"), "steps": normalized}

    def _run(self, run: SkillRun, steps: list[dict], completed: Callable[[dict], None]) -> None:
        try:
            for index, step in enumerate(steps, start=1):
                self._check_cancelled(run)
                self._run_step(run, step)
                run.completed_steps = index
            run.status = "succeeded"
            completed({"status": "succeeded", "skill": run.snapshot()})
        except ProtocolError as exc:
            run.status = "cancelled" if exc.code == "SKILL_CANCELLED" else "failed"
            run.error = exc.message
            completed({"status": run.status, "skill": run.snapshot(), "error_code": exc.code, "error_message": exc.message})
        except Exception as exc:  # pragma: no cover - defensive device boundary
            run.status = "failed"
            run.error = str(exc)
            completed({"status": "failed", "skill": run.snapshot(), "error_code": "SKILL_EXECUTION_FAILED", "error_message": str(exc)})
        finally:
            self.navigation.teleop_velocity()
            with self._lock:
                if self._active_id == run.command_id:
                    self._active_id = None

    def _run_step(self, run: SkillRun, step: dict) -> None:
        kind = str(step.get("kind"))
        if kind == "action":
            action = str(step.get("action") or "")
            if action not in {"stand_up", "lie_down", "passive"}:
                raise ProtocolError("SKILL_STEP_INVALID", f"unsupported action {action}")
            self.navigation.remote_teleop_action(action)
            return
        if kind == "speed":
            level = str(step.get("level") or "")
            actions = {"micro": "speed_micro", "low": "speed_slow", "medium": "speed_normal", "high": "speed_fast"}
            if level not in actions:
                raise ProtocolError("SKILL_STEP_INVALID", f"unsupported speed level {level}")
            self.navigation.remote_teleop_action(actions[level])
            return
        if kind == "velocity":
            self._drive_for(run, float(step.get("vx", 0.0)), float(step.get("vy", 0.0)), float(step.get("yaw_rate", 0.0)), float(step.get("duration_seconds", 0.0)))
            return
        if kind == "turn":
            self._turn_by_angle(run, float(step.get("angle_rad", 0.0)), float(step.get("yaw_rate", 0.45)))
            return
        if kind == "distance":
            self._drive_distance(run, str(step.get("axis") or ""), float(step.get("distance_m", 0.0)), float(step.get("speed_mps", 0.35)))
            return
        if kind == "avoid_forward":
            self._avoid_forward(run, float(step.get("duration_seconds", 0.0)), float(step.get("speed_mps", 0.35)))
            return
        raise ProtocolError("SKILL_STEP_INVALID", f"unsupported step kind {kind}")

    def _drive_for(self, run: SkillRun, vx: float, vy: float, yaw_rate: float, duration: float) -> None:
        if duration <= 0:
            raise ProtocolError("SKILL_STEP_INVALID", "duration_seconds must be positive")
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._check_cancelled(run)
            self.navigation.teleop_velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)
            run.cancel_event.wait(self.REPEAT_SECONDS)

    def _turn_by_angle(self, run: SkillRun, angle: float, yaw_rate: float) -> None:
        pose = self.navigation.latest_pose()
        if pose is None:
            raise ProtocolError("SKILL_POSE_UNAVAILABLE", "turn angle requires a current localization pose")
        if not angle or not yaw_rate:
            raise ProtocolError("SKILL_STEP_INVALID", "angle_rad and yaw_rate must be non-zero")
        start_yaw = float(pose.yaw)
        previous_yaw = start_yaw
        accumulated = 0.0
        direction = 1.0 if angle > 0 else -1.0
        while abs(accumulated) < abs(angle):
            self._check_cancelled(run)
            latest = self.navigation.latest_pose()
            if latest is None:
                raise ProtocolError("SKILL_POSE_UNAVAILABLE", "localization pose disappeared during turn")
            delta = (float(latest.yaw) - previous_yaw + math.pi) % math.tau - math.pi
            accumulated += delta
            previous_yaw = float(latest.yaw)
            self.navigation.teleop_velocity(yaw_rate=direction * abs(yaw_rate))
            run.cancel_event.wait(self.REPEAT_SECONDS)

    def _drive_distance(self, run: SkillRun, axis: str, distance: float, speed: float) -> None:
        pose = self.navigation.latest_pose()
        if pose is None:
            raise ProtocolError("SKILL_POSE_UNAVAILABLE", "distance movement requires a current localization pose")
        vectors = {"forward": (1.0, 0.0), "backward": (-1.0, 0.0), "left": (0.0, 1.0), "right": (0.0, -1.0)}
        if axis not in vectors or distance <= 0 or speed <= 0:
            raise ProtocolError("SKILL_STEP_INVALID", "distance step has invalid axis, distance_m or speed_mps")
        start_x, start_y = float(pose.x), float(pose.y)
        vx, vy = vectors[axis]
        while True:
            self._check_cancelled(run)
            latest = self.navigation.latest_pose()
            if latest is None:
                raise ProtocolError("SKILL_POSE_UNAVAILABLE", "localization pose disappeared during distance step")
            if math.hypot(float(latest.x) - start_x, float(latest.y) - start_y) >= distance:
                return
            self.navigation.teleop_velocity(vx=vx * speed, vy=vy * speed)
            run.cancel_event.wait(self.REPEAT_SECONDS)

    def _avoid_forward(self, run: SkillRun, duration: float, speed: float) -> None:
        if duration <= 0 or speed <= 0:
            raise ProtocolError("SKILL_STEP_INVALID", "avoid_forward requires positive duration_seconds and speed_mps")
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._check_cancelled(run)
            snapshot = self.navigation.obstacle_monitor_snapshot()
            distance = snapshot.get("front_obstacle_distance_m")
            if distance is not None and float(distance) <= 0.8:
                self.navigation.teleop_velocity()
                self._turn_by_angle(run, math.pi / 2, 0.45)
            self.navigation.teleop_velocity(vx=speed)
            run.cancel_event.wait(self.REPEAT_SECONDS)

    @staticmethod
    def _check_cancelled(run: SkillRun) -> None:
        if run.cancel_event.is_set():
            raise ProtocolError("SKILL_CANCELLED", "skill was cancelled")

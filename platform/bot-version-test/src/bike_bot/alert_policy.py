from __future__ import annotations


class BicycleAlertPolicy:
    """Confirm a bicycle over consecutive frames, then cool down before detecting again."""

    def __init__(self, *, confirm_frames: int = 3, cooldown_seconds: float = 10.0) -> None:
        self.confirm_frames = max(1, int(confirm_frames))
        self.cooldown_seconds = max(0.0, float(cooldown_seconds))
        self._streak = 0
        self._cooldown_until = 0.0

    @property
    def streak(self) -> int:
        return self._streak

    def cooldown_remaining(self, now: float) -> float:
        return max(0.0, self._cooldown_until - now)

    def in_cooldown(self, now: float) -> bool:
        return self.cooldown_remaining(now) > 0.0

    def begin_cooldown(self, now: float) -> None:
        self._cooldown_until = now + self.cooldown_seconds
        self._streak = 0

    def reset_streak(self) -> None:
        self._streak = 0

    def observe(self, bicycle_present: bool, now: float) -> bool:
        """Return True once `confirm_frames` consecutive hits complete; then start cooldown."""
        if self.in_cooldown(now):
            return False
        if not bicycle_present:
            self._streak = 0
            return False
        self._streak += 1
        if self._streak < self.confirm_frames:
            return False
        self.begin_cooldown(now)
        return True

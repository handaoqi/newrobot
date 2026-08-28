from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path


class SimulationBackend(ABC):
    @abstractmethod
    def prepare(self, job: dict, workspace: Path) -> None: ...

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def health(self) -> dict: ...

    @abstractmethod
    def cancel(self) -> None: ...

    @abstractmethod
    def collect(self) -> list[Path]: ...


class MatrixBackend(SimulationBackend):
    """Contract placeholder until a dedicated GPU Runner is provisioned."""

    def prepare(self, job: dict, workspace: Path) -> None:
        raise RuntimeError("RUNNER_UNAVAILABLE: MATRiX/UE GPU Runner 尚未部署")

    def start(self) -> None:  # pragma: no cover - prepare always fails
        raise RuntimeError("RUNNER_UNAVAILABLE")

    def health(self) -> dict:
        return {"ready": False, "code": "RUNNER_UNAVAILABLE"}

    def cancel(self) -> None:
        return None

    def collect(self) -> list[Path]:
        return []

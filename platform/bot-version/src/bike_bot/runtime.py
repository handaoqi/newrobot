from __future__ import annotations

from dataclasses import dataclass
from threading import Lock

from .config import AppConfig
from .models import Motion, Network, Position, Power, RuntimeInfo


@dataclass
class RuntimeSnapshot:
    position: Position
    motion: Motion
    power: Power
    network: Network
    runtime: RuntimeInfo


class RuntimeState:
    def __init__(self, config: AppConfig) -> None:
        self._lock = Lock()
        self._position = Position(
            name=config.location.name,
            latitude=config.location.latitude,
            longitude=config.location.longitude,
        )
        self._motion = Motion(speed=config.runtime.speed, heading=config.runtime.heading)
        self._power = Power(
            battery_level=config.runtime.battery_level,
            charging=config.runtime.charging,
        )
        self._network = Network(
            signal_strength=config.runtime.signal_strength,
            network_type=config.runtime.network_type,
        )
        self._runtime = RuntimeInfo(mode=config.runtime.mode, status=config.runtime.status)

    def snapshot(self) -> RuntimeSnapshot:
        with self._lock:
            return RuntimeSnapshot(
                position=Position(**self._position.__dict__),
                motion=Motion(**self._motion.__dict__),
                power=Power(**self._power.__dict__),
                network=Network(**self._network.__dict__),
                runtime=RuntimeInfo(**self._runtime.__dict__),
            )

    def update_status(
        self,
        *,
        speed: float | None = None,
        heading: float | None = None,
        battery_level: int | None = None,
        charging: bool | None = None,
        signal_strength: int | None = None,
        network_type: str | None = None,
        runtime_status: str | None = None,
        runtime_mode: str | None = None,
    ) -> None:
        with self._lock:
            if speed is not None:
                self._motion.speed = speed
            if heading is not None:
                self._motion.heading = heading
            if battery_level is not None:
                self._power.battery_level = battery_level
            if charging is not None:
                self._power.charging = charging
            if signal_strength is not None:
                self._network.signal_strength = signal_strength
            if network_type is not None:
                self._network.network_type = network_type
            if runtime_status is not None:
                self._runtime.status = runtime_status
            if runtime_mode is not None:
                self._runtime.mode = runtime_mode

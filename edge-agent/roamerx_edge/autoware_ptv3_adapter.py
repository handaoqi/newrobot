"""Deployment metadata and class mapping for the Autoware PTv3 bundle.

The published bundle uses custom ``autoware`` sparse-convolution operators.
This adapter deliberately reports TensorRT readiness instead of silently
falling back to CPU ONNX Runtime, which cannot load those operators.
"""

from __future__ import annotations

import shutil
from pathlib import Path

AUTOWARE_LABELS = (
    "car", "truck", "bus", "bicycle", "pedestrian", "traffic_cone",
    "barrier", "debris", "drivable_flat", "non_drivable_flat", "vegetation",
    "building", "vertical_thin", "static_clutter", "noise",
)

ASSET_BY_LABEL = {
    "car": "vehicle.sedan",
    "truck": "vehicle.van",
    "bus": "vehicle.bus",
    "bicycle": "bicycle.standard",
    "pedestrian": "person.adult",
    "traffic_cone": "traffic-cone.standard",
    "barrier": "barrier.concrete",
    "debris": "debris.pile",
    "drivable_flat": "road.straight",
    "non_drivable_flat": "road.straight",
    "vegetation": "vegetation.groundcover",
    "building": "building.kiosk",
    "vertical_thin": "wall.vertical-thin",
}


class AutowarePtv3Bundle:
    files = (
        "ptv3_encoder.onnx",
        "ptv3_det3d_head.onnx",
        "ptv3_seg3d_head.onnx",
        "ml_package_ptv3_encoder.param.yaml",
        "ml_package_ptv3_det3d_head.param.yaml",
        "ml_package_ptv3_seg3d_head.param.yaml",
    )

    def __init__(self, directory: str | Path, plugin_path: str | Path = "", inference_binary_path: str | Path = ""):
        self.directory = Path(directory).expanduser()
        self.configured_plugin_path = Path(plugin_path).expanduser() if plugin_path else None
        self.configured_inference_binary_path = Path(inference_binary_path).expanduser() if inference_binary_path else None

    def missing_files(self) -> list[str]:
        return [name for name in self.files if not (self.directory / name).is_file()]

    def plugin_path(self) -> Path | None:
        if self.configured_plugin_path and self.configured_plugin_path.is_file():
            return self.configured_plugin_path
        configured = self.directory / "libautoware_tensorrt_plugins.so"
        if configured.is_file():
            return configured
        candidates = sorted(self.directory.glob("**/libautoware_tensorrt_plugins.so"))
        candidates.extend(Path(path) for path in (
            "/home/dogrobot/runtime/nx-edge/install/ptv3/lib/libautoware_tensorrt_plugins.so",
            "/usr/local/lib/libautoware_tensorrt_plugins.so",
        ) if Path(path).is_file())
        return candidates[0] if candidates else None

    def inference_binary_path(self) -> Path | None:
        if self.configured_inference_binary_path and self.configured_inference_binary_path.is_file():
            return self.configured_inference_binary_path
        return next((Path(path) for path in (
            "/home/dogrobot/runtime/nx-edge/install/ptv3/bin/roamerx_ptv3_cli",
            "/home/dogrobot/runtime/nx-edge/install/ptv3/lib/roamerx_ptv3_cli",
        ) if Path(path).is_file()), None)

    def status(self) -> dict:
        missing = self.missing_files()
        trtexec = self.trtexec_path()
        plugin = self.plugin_path()
        runner = self.inference_binary_path()
        return {
            "model_dir": str(self.directory),
            "format": "autoware-three-stage-onnx",
            "labels": list(AUTOWARE_LABELS),
            "missing_files": missing,
            "trtexec_available": trtexec is not None,
            "plugin_path": str(plugin) if plugin else "",
            "plugin_available": plugin is not None,
            "runner_path": str(runner) if runner else "",
            "runner_available": runner is not None,
            "ready": not missing and trtexec is not None and plugin is not None and runner is not None,
            "reason": "ready" if not missing and trtexec and plugin and runner else (
                "autoware sparse-convolution TensorRT plugin is required" if not plugin else
                "PTv3 TensorRT runner, TensorRT/trtexec or model bundle is incomplete"
            ),
        }

    @staticmethod
    def trtexec_path() -> str | None:
        return shutil.which("trtexec") or next((path for path in (
            "/opt/TensorRT-10.6.0.26/targets/aarch64-linux-gnu/bin/trtexec",
            "/usr/src/tensorrt/bin/trtexec",
        ) if Path(path).is_file()), None)

    @staticmethod
    def asset_for_label(label: str) -> str | None:
        return ASSET_BY_LABEL.get(str(label).strip().lower())

    def trtexec_commands(self) -> list[list[str]]:
        """Return architecture-specific engine build commands when available."""
        plugin = self.plugin_path()
        plugin_arg = [f"--plugins={plugin}"] if plugin else []
        return [
            [self.trtexec_path() or "trtexec", *plugin_arg, f"--onnx={self.directory / 'ptv3_encoder.onnx'}", f"--saveEngine={self.directory / 'ptv3_encoder_fp16.engine'}", "--fp16"],
            [self.trtexec_path() or "trtexec", *plugin_arg, f"--onnx={self.directory / 'ptv3_det3d_head.onnx'}", f"--saveEngine={self.directory / 'ptv3_det3d_head_fp16.engine'}", "--fp16"],
            [self.trtexec_path() or "trtexec", *plugin_arg, f"--onnx={self.directory / 'ptv3_seg3d_head.onnx'}", f"--saveEngine={self.directory / 'ptv3_seg3d_head_fp16.engine'}", "--fp16"],
        ]

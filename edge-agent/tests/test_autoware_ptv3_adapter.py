from pathlib import Path

from roamerx_edge.autoware_ptv3_adapter import AutowarePtv3Bundle


def test_autoware_bundle_reports_downloaded_three_stage_package():
    bundle = AutowarePtv3Bundle("/home/dogrobot/runtime/nx-edge/install/models/scene/autoware-ptv3-v4")
    status = bundle.status()
    assert status["missing_files"] == []
    assert status["format"] == "autoware-three-stage-onnx"
    assert status["trtexec_available"] is True
    assert status["ready"] == (status["plugin_available"] and status["runner_available"])
    assert bundle.asset_for_label("bus") == "vehicle.bus"
    assert bundle.asset_for_label("vertical_thin") == "wall.vertical-thin"
    assert any(Path(argument.split("=", 1)[1]).name == "ptv3_encoder.onnx" for argument in bundle.trtexec_commands()[0] if argument.startswith("--onnx="))

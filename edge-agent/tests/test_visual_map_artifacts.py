import json
import tempfile
from pathlib import Path

from PIL import Image

from roamerx_edge.visual_map_artifacts import VisualMapError, create_visual_map_artifacts


def test_rgb_pcd_creates_non_navigation_orthophoto():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        pcd = root / "colored.pcd"
        pcd.write_text(
            "\n".join([
                "# .PCD v0.7", "VERSION 0.7", "FIELDS x y z r g b", "SIZE 4 4 4 1 1 1",
                "TYPE F F F U U U", "COUNT 1 1 1 1 1 1", "WIDTH 2", "HEIGHT 1", "POINTS 2", "DATA ascii",
                "0 0 0 255 0 0", "1 0 0 0 255 0", "",
            ]), encoding="ascii",
        )
        payload = create_visual_map_artifacts(pcd, root / "visual", resolution_m=1.0)
        assert payload["schema"] == "roamerx.visual-map.v1"
        assert payload["navigation_authoritative"] is False
        image = Image.open(root / "visual" / "rgb_orthophoto.png")
        assert image.size == (2, 1)
        assert json.loads((root / "visual" / "visual_manifest.json").read_text())["artifacts"]


def test_intensity_only_pcd_is_rejected():
    with tempfile.TemporaryDirectory() as directory:
        pcd = Path(directory) / "intensity.pcd"
        pcd.write_text(
            "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0 1\n",
            encoding="ascii",
        )
        try:
            create_visual_map_artifacts(pcd, Path(directory) / "visual")
        except VisualMapError as exc:
            assert "no RGB" in str(exc)
        else:
            raise AssertionError("intensity-only cloud must not become an RGB artifact")

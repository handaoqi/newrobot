from __future__ import annotations

from pathlib import Path


def generate_map_preview(pgm_path: Path, output_path: Path, max_size: int = 1200) -> Path:
    """Create a browser-friendly PNG preview without changing the PGM map."""
    from PIL import Image

    with Image.open(pgm_path) as image:
        resampling = getattr(Image, "Resampling", Image)
        image.thumbnail((max_size, max_size), resampling.NEAREST)
        image.convert("RGB").save(output_path, format="PNG", optimize=True)
    return output_path

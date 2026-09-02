#!/usr/bin/env python3
"""Prepare a static Lichtblick index with a default layout.

The Lichtblick bundle is vendored as an immutable tree.  Deployment should not
modify that source tree in place, so this small adapter writes only a prepared
copy of ``index.html`` for the static web server to publish.
"""

from __future__ import annotations

import argparse
import os
import tempfile
from pathlib import Path

from foxglove_web_serve import build_index


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dist", type=Path, required=True, help="vendored Lichtblick dist directory")
    parser.add_argument("--layout", type=Path, required=True, help="default layout JSON")
    parser.add_argument("--output", type=Path, required=True, help="prepared index.html path")
    args = parser.parse_args()

    body = build_index(args.dist, args.layout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=args.output.parent, prefix=f".{args.output.name}.", delete=False) as handle:
        temporary = Path(handle.name)
        handle.write(body)
        handle.flush()
        os.fchmod(handle.fileno(), 0o644)
    os.replace(temporary, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

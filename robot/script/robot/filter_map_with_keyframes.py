#!/usr/bin/env python3
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "edge_agent"))

from roamerx_edge.keyframe_visibility_filter import filter_with_keyframe_visibility

parser = argparse.ArgumentParser(description="Offline keyframe visibility filter for a saved map")
parser.add_argument("map_dir", type=Path)
parser.add_argument("--output", type=Path)
parser.add_argument("--voxel-size", type=float, default=0.3)
parser.add_argument("--min-free", type=int, default=1)
parser.add_argument("--max-hits", type=int, default=10)
args = parser.parse_args()
output = args.output or args.map_dir / "keyframe_visibility"
print(json.dumps(filter_with_keyframe_visibility(args.map_dir, output, voxel_size_m=args.voxel_size, min_free_observations=args.min_free, max_hit_observations=args.max_hits), ensure_ascii=False, indent=2))

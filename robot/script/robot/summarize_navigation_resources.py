#!/usr/bin/env python3
"""Summarize task-scoped tegrastats and pidstat logs into stable JSON."""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import defaultdict
from pathlib import Path
from statistics import mean


def percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = int(math.floor(position))
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def distribution(values: list[float], digits: int = 3) -> dict:
    if not values:
        return {"samples": 0, "mean": None, "p90": None, "p99": None, "max": None}
    return {
        "samples": len(values),
        "mean": round(mean(values), digits),
        "p90": round(percentile(values, 0.90), digits),
        "p99": round(percentile(values, 0.99), digits),
        "max": round(max(values), digits),
    }


TEGRA_PATTERNS = {
    "ram": re.compile(r"\bRAM (\d+)/(\d+)MB"),
    "swap": re.compile(r"\bSWAP (\d+)/(\d+)MB"),
    "gpu": re.compile(r"\bGR3D_FREQ\s+(?:\[)?(\d+(?:\.\d+)?)%"),
    "power": re.compile(r"\bVDD_IN\s+(\d+(?:\.\d+)?)mW"),
    "cpu_temp": re.compile(r"\bcpu@(\d+(?:\.\d+)?)C"),
    "gpu_temp": re.compile(r"\bgpu@(\d+(?:\.\d+)?)C"),
}


def parse_tegrastats(path: Path) -> dict:
    samples: dict[str, list[float]] = defaultdict(list)
    cpu_re = re.compile(r"\bCPU \[([^]]+)\]")
    if not path.is_file():
        return {"available": False, "path": str(path), "metrics": {}}

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        ram = TEGRA_PATTERNS["ram"].search(line)
        if ram:
            used, total = map(float, ram.groups())
            samples["ram_used_mb"].append(used)
            if total > 0:
                samples["ram_used_percent"].append(used / total * 100.0)
        swap = TEGRA_PATTERNS["swap"].search(line)
        if swap:
            samples["swap_used_mb"].append(float(swap.group(1)))
        cpu = cpu_re.search(line)
        if cpu:
            cores = []
            for token in cpu.group(1).split(","):
                match = re.search(r"(\d+(?:\.\d+)?)%@", token)
                cores.append(float(match.group(1)) if match else 0.0)
            if cores:
                samples["cpu_total_percent"].append(mean(cores))
                samples["cpu_core_sum_percent"].append(sum(cores))
                samples["cpu_max_core_percent"].append(max(cores))
        for name in ("gpu", "power", "cpu_temp", "gpu_temp"):
            match = TEGRA_PATTERNS[name].search(line)
            if not match:
                continue
            value = float(match.group(1))
            if name == "power":
                value /= 1000.0
            samples[{"gpu": "gpu_percent", "power": "power_w"}.get(name, name + "_c")].append(value)

    return {
        "available": bool(samples),
        "path": str(path),
        "metrics": {name: distribution(values) for name, values in sorted(samples.items())},
    }


def parse_pidstat(path: Path, top_processes: int = 30) -> dict:
    if not path.is_file():
        return {"available": False, "path": str(path), "processes": []}
    header: list[str] | None = None
    values: dict[tuple[int, str], dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if line.startswith("# Time"):
            header = line[2:].split()
            continue
        if not header or not line or line.startswith("Linux"):
            continue
        columns = line.split(maxsplit=len(header) - 1)
        if len(columns) != len(header):
            continue
        row = dict(zip(header, columns))
        try:
            pid = int(row["PID"])
        except (KeyError, ValueError):
            continue
        command = row.get("Command", "unknown")
        process = values[(pid, command)]
        for source, target in (
            ("%CPU", "cpu_percent"),
            ("RSS", "rss_kib"),
            ("kB_rd/s", "read_kib_per_s"),
            ("kB_wr/s", "write_kib_per_s"),
            ("majflt/s", "major_faults_per_s"),
        ):
            try:
                value = float(row[source])
            except (KeyError, ValueError):
                continue
            if value >= 0:
                process[target].append(value)

    processes = []
    for (pid, command), metrics in values.items():
        processes.append({
            "pid": pid,
            "command": command,
            "metrics": {name: distribution(series) for name, series in sorted(metrics.items())},
        })
    processes.sort(
        key=lambda item: (
            item["metrics"].get("cpu_percent", {}).get("p99") or 0.0,
            item["metrics"].get("rss_kib", {}).get("max") or 0.0,
        ),
        reverse=True,
    )
    return {
        "available": bool(processes),
        "path": str(path),
        "processes_total": len(processes),
        "processes": processes[:top_processes],
    }


def build_summary(tegrastats_path: Path, pidstat_path: Path) -> dict:
    return {
        "schema": "roamerx.navigation-resource-summary.v1",
        "sample_interval_seconds": 1,
        "system": parse_tegrastats(tegrastats_path),
        "process": parse_pidstat(pidstat_path),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tegrastats", type=Path, required=True)
    parser.add_argument("--pidstat", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    summary = build_summary(args.tegrastats, args.pidstat)
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

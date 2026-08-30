from pathlib import Path

from summarize_navigation_resources import build_summary


def test_build_summary_reports_system_percentiles_and_processes(tmp_path: Path):
    tegra = tmp_path / "tegrastats.log"
    tegra.write_text(
        "08-30-2026 18:50:23 RAM 4000/16000MB SWAP 5/8000MB "
        "CPU [10%@1000,20%@1000,off,30%@1000] GR3D_FREQ 40% "
        "cpu@50.0C gpu@49.0C VDD_IN 8000mW/8000mW\n"
        "08-30-2026 18:50:24 RAM 8000/16000MB SWAP 7/8000MB "
        "CPU [50%@1000,60%@1000,70%@1000,80%@1000] GR3D_FREQ 90% "
        "cpu@55.0C gpu@54.0C VDD_IN 12000mW/10000mW\n",
        encoding="utf-8",
    )
    pidstat = tmp_path / "pidstat.log"
    pidstat.write_text(
        "# Time UID PID %usr %system %guest %wait %CPU CPU minflt/s majflt/s "
        "VSZ RSS %MEM kB_rd/s kB_wr/s kB_ccwr/s iodelay Command\n"
        "18:50:23 1000 42 20 5 0 0 25 2 0 0 1000 500 1 3 4 0 0 localization\n"
        "18:50:24 1000 42 30 10 0 0 40 3 0 0 1000 700 1 5 6 0 0 localization\n",
        encoding="utf-8",
    )

    result = build_summary(tegra, pidstat)

    assert result["system"]["metrics"]["cpu_total_percent"]["max"] == 65.0
    assert result["system"]["metrics"]["gpu_percent"]["p99"] == 89.5
    assert result["system"]["metrics"]["ram_used_mb"]["max"] == 8000.0
    process = result["process"]["processes"][0]
    assert process["command"] == "localization"
    assert process["metrics"]["cpu_percent"]["p90"] == 38.5
    assert process["metrics"]["rss_kib"]["max"] == 700.0

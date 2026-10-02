# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Regressions for the temporary isolated Visual Studio diagnostic runner."""

import json
import os
from pathlib import Path
import subprocess
import sys


def test_debug_runner_streams_unicode_and_continues_after_failure(tmp_path):
    """A Windows code page must not stop collection of later diagnostic cases."""
    runner = Path(__file__).resolve().parents[2] / ".github/scripts/run_visual_studio_debug_checks.py"
    (tmp_path / "pytest.py").write_text(
        "def main(arguments):\n"
        "    print('progress: \\u258f\\u2588')\n"
        "    return int('test_bskLogging.py' in arguments[-1])\n",
        encoding="utf-8",
    )
    report_dir = tmp_path / "reports"
    result = subprocess.run(
        [sys.executable, str(runner), "--case", "logging-flush", "--case", "gravity-tesseral-warning",
         "--report-dir", str(report_dir), "--timeout", "30"],  # [s]
        env={**os.environ, "PYTHONPATH": str(tmp_path), "PYTHONIOENCODING": "cp1252"},
        capture_output=True, encoding="utf-8", timeout=90,  # [s]
    )
    assert result.returncode == 1, result.stdout + result.stderr
    assert "progress: ▏█" in result.stdout
    assert "UnicodeEncodeError" not in result.stderr
    summary = json.loads((report_dir / "summary.json").read_text(encoding="utf-8"))
    assert [case["returnCode"] for case in summary] == [1, 0]
    for case in summary:
        assert "progress: ▏█" in (report_dir / (case["case"] + ".log")).read_text(encoding="utf-8")

#!/usr/bin/env python3
# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Run known Visual Studio Debug failures separately and retain their diagnostics.

This runner is temporary and belongs to the combined-binding CI experiment.
"""

import argparse
import faulthandler
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TIMEOUT_SECONDS = 600  # [s]
SHUTDOWN_GRACE_SECONDS = 30  # [s]
OUTPUT_POLL_SECONDS = 0.2  # [s]
CASES = {
    "package-paths": (
        "tests/test_incremental_build_checks.py::"
        "test_package_configuration_checks_native_targets[None-None]"
    ),
    "logging-flush": "architecture/utilities/tests/test_bskLogging.py::test_warning_level_output_is_flushed",
    "gravity-tesseral-warning": (
        "simulation/dynamics/gravityEffector/_UnitTest/test_gravityNonRotatingWarning.py::"
        "test_resetEmitsWarningForTesseralFieldWithoutRotation"
    ),
    "gravity-polyhedral-warning": (
        "simulation/dynamics/gravityEffector/_UnitTest/test_gravityNonRotatingWarning.py::"
        "test_resetEmitsWarningForPolyhedralWithoutRotation"
    ),
    "horizon-opnav": "fswAlgorithms/imageProcessing/horizonOpNav/_UnitTest/test_horizonOpNav.py::test_horizonOpNav",
    "heading-sukf": "fswAlgorithms/attDetermination/headingSuKF/_UnitTest/test_headingSuKF.py::test_all_heading_kf",
    "hinged-motor-sensor": (
        "simulation/sensors/hingedRigidBodyMotorSensor/_UnitTest/test_hingedRigidBodyMotorSensor.py::"
        "test_hingedRigidBodyMotorSensor[1.01--0.23--1--1-0-0-0.0-0.0-1e-12]"
    ),
    "constrained-dynamics": (
        "tests/test_scenarioConstrainedDynamicsFrequencyAnalysis.py::"
        "test_scenarioConstrainedDynamicsFrequencyAnalysis[gain_list0-MEV2]"
    ),
    "flex-panel-comparison": "tests/test_scenarioDynamicsComparison.py::test_scenarios[scenarioCompareFlexPanels]",
    # Run last: the benchmark currently requests a Release build even in Debug.
    "eigen-benchmark": "../benchmarks/tests/test_benchmark_smoke.py::test_eigen_linear_algebra_benchmark_smoke",
}


def run_child(case, report_dir, timeout_seconds):
    """Run one pytest case with a watchdog that also works while native code holds the GIL."""
    import pytest

    faulthandler.enable()
    # A native watchdog can still dump the Python stack if a C assertion dialog
    # or other native call prevents the Python timeout thread from running.
    faulthandler.dump_traceback_later(timeout_seconds, exit=True)
    try:
        return pytest.main([
            "-c", str(REPOSITORY_ROOT / "src/pytest.ini"),
            "-n", "0", "-vv", "-s", "-ra", "--tb=long", "--error-for-skips",
            "--timeout=0", f"--junitxml={report_dir / (case + '.xml')}", CASES[case],
        ])
    finally:
        faulthandler.cancel_dump_traceback_later()


def terminate_process_tree(process):
    """Stop a stalled diagnostic process and any build or simulation children."""
    if os.name == "nt":
        subprocess.run(
            ["taskkill", "/PID", str(process.pid), "/T", "/F"],
            check=False, timeout=SHUTDOWN_GRACE_SECONDS,
        )
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if process.poll() is None:
        process.kill()
    process.wait(timeout=SHUTDOWN_GRACE_SECONDS)


def run_case(case, report_dir, timeout_seconds):
    """Stream one isolated case to the console and its own persistent log."""
    command = [
        sys.executable, "-u", "-X", "faulthandler", str(Path(__file__).resolve()),
        "--child", case, "--report-dir", str(report_dir), "--timeout", str(timeout_seconds),
    ]
    print(f"::group::Debug diagnostic: {case}", flush=True)
    print(CASES[case], flush=True)
    start_time = time.monotonic()
    timed_out = False
    log_path = report_dir / f"{case}.log"
    with log_path.open("w", encoding="utf-8") as log, log_path.open(
        encoding="utf-8", errors="replace"
    ) as output:
        with subprocess.Popen(
            command, cwd=REPOSITORY_ROOT / "src", stdout=log,
            stderr=subprocess.STDOUT,
            env={**os.environ, "PYTHONIOENCODING": "utf-8"},
            start_new_session=(os.name != "nt"),
        ) as process:
            while process.poll() is None:
                print(output.read(), end="", flush=True)
                if time.monotonic() - start_time > timeout_seconds + SHUTDOWN_GRACE_SECONDS:
                    timed_out = True
                    terminate_process_tree(process)
                    break
                time.sleep(OUTPUT_POLL_SECONDS)
            print(output.read(), end="", flush=True)
            return_code = process.returncode
    result = {
        "case": case, "nodeid": CASES[case], "returnCode": return_code or int(timed_out),
        "supervisorTimeout": timed_out,
        "durationSeconds": round(time.monotonic() - start_time, 2),
    }
    print(json.dumps(result), flush=True)
    print("::endgroup::", flush=True)
    return result


def main():
    """Run every selected diagnostic even when an earlier case fails or crashes."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", choices=CASES)
    parser.add_argument("--child", choices=CASES, help=argparse.SUPPRESS)
    parser.add_argument("--report-dir", type=Path, default=REPOSITORY_ROOT / "visual-studio-debug-checks")
    parser.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT_SECONDS, help="Per-case timeout in seconds")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    report_dir = args.report_dir.resolve()
    report_dir.mkdir(parents=True, exist_ok=True)
    if args.child:
        return run_child(args.child, report_dir, args.timeout)

    results = []
    for case in args.case or CASES:
        try:
            result = run_case(case, report_dir, args.timeout)
        except (OSError, RuntimeError, subprocess.SubprocessError) as error:
            result = {"case": case, "nodeid": CASES[case], "returnCode": 1, "error": str(error)}
            print(f"::error::{case}: {error}", flush=True)
        results.append(result)
        # Persist after each case, including failures that cannot produce JUnit XML.
        (report_dir / "summary.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    failed = [result["case"] for result in results if result["returnCode"] != 0]
    if failed:
        print("::error::Debug diagnostics failed: " + ", ".join(failed), flush=True)
    return int(bool(failed))


if __name__ == "__main__":
    raise SystemExit(main())

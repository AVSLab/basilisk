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

"""Check the startup benchmark without rebuilding libraries during CI."""

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest


REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPT = REPOSITORY / "benchmarks/startup/benchmark_fsw_bindings.py"
SUBPROCESS_TIMEOUT_SECONDS = 120  # [s]


@pytest.fixture
def benchmark():
    """Load the standalone command without importing Basilisk or invoking a build."""
    spec = importlib.util.spec_from_file_location("fsw_startup_benchmark", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_fsw_startup_smoke():
    """Import one binding without changing the build cache, objects, or native files."""
    build = REPOSITORY / "dist3"
    if not (build / "CMakeCache.txt").is_file() or not (build / "Basilisk/__init__.py").is_file():
        pytest.skip("The FSW startup smoke test requires a configured and built dist3 directory.")
    paths = [build / "CMakeCache.txt", *build.glob("autoSource/*fsw*"),
             *build.glob("Basilisk/fswAlgorithms/*"),
             *build.glob("CMakeFiles/*mrpFeedback*.dir/**/*")]
    before = {path: path.stat().st_mtime_ns for path in paths if path.is_file()}
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--build-dir", str(build), "--smoke"],
        cwd=REPOSITORY, capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT_SECONDS,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert json.loads(result.stdout)["seconds"] >= 0
    assert before == {path: path.stat().st_mtime_ns for path in before}


def test_cleanup_checks_all_paths_before_removing_files(benchmark, tmp_path):
    """Reject an outside path before deleting any valid build output."""
    build = tmp_path / "build"
    build.mkdir()
    generated = build / "generated.so"
    external = tmp_path / "external.so"
    for path in (generated, external):
        path.write_bytes(b"preserve")
    with pytest.raises(ValueError, match="outside the build directory"):
        benchmark.remove_outputs(build, [generated, external])
    assert generated.read_bytes() == external.read_bytes() == b"preserve"


def test_linker_output_selection(benchmark, tmp_path):
    """Select native project link outputs across platforms, excluding dependencies."""
    listing = "\n".join([
        "Basilisk/fswAlgorithms/_mrpFeedback.so: CXX_MODULE_LIBRARY_LINKER__mrpFeedback_Release",
        "Basilisk/fswAlgorithms/_mrpFeedback.pyd: CXX_MODULE_LIBRARY_LINKER__mrpFeedback_Release",
        "Basilisk/fswAlgorithms/_mrpFeedback.lib: CXX_MODULE_LIBRARY_LINKER__mrpFeedback_Release",
        "Basilisk/libBasilisk.dll: CXX_SHARED_LIBRARY_LINKER__Basilisk_Release",
        "Basilisk/libBasilisk.dylib: CXX_SHARED_LIBRARY_LINKER__Basilisk_Release",
        "external/dependency.so: CXX_SHARED_LIBRARY_LINKER__dependency_Release",
        "Basilisk/_wrapper.so: CUSTOM_COMMAND",
    ])
    assert {path.relative_to(tmp_path).as_posix()
            for path in benchmark.linker_outputs(tmp_path, listing)} == {
        "Basilisk/fswAlgorithms/_mrpFeedback.so", "Basilisk/fswAlgorithms/_mrpFeedback.pyd",
        "Basilisk/libBasilisk.dll", "Basilisk/libBasilisk.dylib",
    }


@pytest.mark.parametrize("collection_error", [False, True])
def test_collection_profiler_preserves_errors_and_skips_execution(tmp_path, collection_error):
    """Clear selected tests while keeping import errors visible in pytest's exit code."""
    test_file = tmp_path / "test_probe.py"
    test_file.write_text(
        "raise RuntimeError('collection failed')\n" if collection_error else
        "def test_must_not_execute():\n    raise AssertionError('test executed')\n",
        encoding="utf-8",
    )
    env = os.environ.copy()
    env.update(PYTEST_DISABLE_PLUGIN_AUTOLOAD="1", PYTEST_ADDOPTS="",
               PYTHONPATH=str(SCRIPT.parent), BSK_STARTUP_OUTPUT=str(tmp_path),
               BSK_STARTUP_STARTED=str(time.time()))
    env.pop("PYTEST_XDIST_WORKER", None)
    result = subprocess.run(
        [sys.executable, "-m", "pytest", "-p", "_collection_profile", str(test_file)],
        cwd=tmp_path, env=env, capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT_SECONDS,
    )
    assert result.returncode == (2 if collection_error else 5), result.stdout + result.stderr
    profile = json.loads((tmp_path / "worker-controller.json").read_text(encoding="utf-8"))
    assert profile["exit_code"] == result.returncode
    assert profile["selected_items"] == (0 if collection_error else 1)
    assert profile["ready_seconds"] >= 0


def test_incremental_failure_restores_source_timestamp(benchmark, tmp_path, monkeypatch):
    """A failed incremental build must leave the touched source's timestamp unchanged."""
    source = tmp_path / "fswAlgorithms/attControl/mrpFeedback/mrpFeedback.c"
    source.parent.mkdir(parents=True)
    source.write_text("/* Source remains unchanged. */\n", encoding="utf-8")
    original = source.stat().st_mtime_ns
    runner = benchmark.Benchmark.__new__(benchmark.Benchmark)
    runner.source_dir = tmp_path
    monkeypatch.setattr(runner, "native_outputs", lambda: [source])
    monkeypatch.setattr(benchmark, "MTIME_MARGIN_SECONDS", 0.0)  # [s]

    def fail_build(label):
        raise RuntimeError("simulated compiler failure")

    monkeypatch.setattr(runner, "build", fail_build)
    with pytest.raises(RuntimeError, match="simulated compiler failure"):
        runner.incremental("c", 0)
    assert source.stat().st_mtime_ns == original
    assert source.read_text(encoding="utf-8") == "/* Source remains unchanged. */\n"


@pytest.mark.parametrize("failure", [KeyboardInterrupt, RuntimeError])
@pytest.mark.parametrize("recovery_fails", [False, True])
def test_interrupted_full_relink_recovers_non_fsw_outputs(benchmark, tmp_path, monkeypatch,
                                                       failure, recovery_fails):
    """Recover every deleted library and report an unsuccessful recovery honestly."""
    build = tmp_path / "build"
    (build / "autoSource").mkdir(parents=True)
    (build / "autoSource/fswCoreBindings.txt").write_text("mrpFeedback\n", encoding="utf-8")
    (build / "CMakeCache.txt").write_text(
        f"CMAKE_HOME_DIRECTORY:INTERNAL={REPOSITORY / 'src'}\n"
        "CMAKE_GENERATOR:INTERNAL=Ninja\n"
        "CMAKE_COMMAND:INTERNAL=cmake\n"
        "CMAKE_MAKE_PROGRAM:FILEPATH=ninja\n", encoding="utf-8",
    )
    suffix = ".pyd" if os.name == "nt" else ".so"
    native_paths = [build / f"Basilisk/fswAlgorithms/_fswCoreNative{suffix}",
                    build / f"Basilisk/simulation/_spacecraft{suffix}"]
    for path in native_paths:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"original native output")
    runner = benchmark.Benchmark(SimpleNamespace(
        build_dir=build, smoke=False, parallel=1, output=tmp_path / "results",
    ))
    monkeypatch.setattr(runner, "configure", lambda: None)
    listing = "\n".join(f"{path.relative_to(build).as_posix()}: CXX_MODULE_LIBRARY_LINKER"
                        for path in native_paths)
    monkeypatch.setattr(benchmark.subprocess, "check_output", lambda *args, **kwargs: listing)

    def interrupted_build(label, all_targets=False):
        """Model a build's target selection after a failure immediately after deletion."""
        if label == "all-native-relink":
            assert not any(path.exists() for path in native_paths)
            raise failure("interrupted relink")
        assert label == "restore"
        if recovery_fails:
            raise RuntimeError("recovery failed")
        for path in native_paths if all_targets else native_paths[:1]:
            path.write_bytes(b"rebuilt native output")

    monkeypatch.setattr(runner, "build", interrupted_build)
    monkeypatch.setattr(runner, "measure_group",
                        lambda: runner.relink("all-native-relink", all_targets=True))
    expected_error = RuntimeError if recovery_fails else failure
    expected_message = "recovery failed" if recovery_fails else "interrupted relink"
    with pytest.raises(expected_error, match=expected_message):
        runner.run()
    saved = json.loads((runner.output / "results.json").read_text(encoding="utf-8"))
    assert saved["restored_build"] is not recovery_fails
    assert saved["error"] == "interrupted relink"
    assert all(path.exists() for path in native_paths) is not recovery_fails

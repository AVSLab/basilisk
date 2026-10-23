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

"""Exercise Cargo output filtering and the real Corrosion build integration."""

import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
WRAPPER = REPOSITORY_ROOT / "src/cmake/runCargo.py"


def run(command, directory, environment=None):
    """Capture subprocess output and status without converting failures into exceptions."""
    return subprocess.run(
        command, cwd=directory, env=environment, capture_output=True,
        check=False, timeout=120,  # [s]
    )


@pytest.mark.parametrize("verbose", [False, True])
@pytest.mark.parametrize("exit_code", [0, 37])
@pytest.mark.parametrize("diagnostics_first", [False, True])
def test_cargo_wrapper_preserves_diagnostics_and_status(tmp_path, verbose, exit_code, diagnostics_first):
    """Retain raw diagnostic bytes, stdout, arguments, and the child's exit status."""
    progress = b"   Compiling fixture v0.1.0\n"
    colored = b"\x1b[1m\x1b[32m   Compiling\x1b[0m fixture v0.1.0\r\n"
    finished = b"    Finished `release` profile [optimized] target(s) in 0.03s\n"
    diagnostics = b"warning: build-script warning\nerror: example error\n\xff diagnostic byte\n"
    # A source excerpt resembling progress must survive once a diagnostic starts.
    prefix = b"" if diagnostics_first else progress + colored + finished
    stderr = prefix + diagnostics + progress + finished
    stdout = b"   Compiling stdout text must survive v0.1.0\n"
    child = tmp_path / "fake cargo.py"
    child.write_text(
        "import os, sys\n"
        "assert sys.argv[1:] == ['argument with spaces', '--locked']\n"
        f"os.write(1, {stdout!r})\n"
        f"os.write(2, {stderr!r})\n"
        f"sys.exit({exit_code})\n",
        encoding="utf-8",
    )
    command = [sys.executable, str(WRAPPER)]
    if verbose:
        command.append("--verbose")
    command += [sys.executable, str(child), "argument with spaces", "--locked"]
    result = run(command, tmp_path)
    assert result.returncode == exit_code
    assert result.stdout == stdout
    assert result.stderr == (stderr if verbose else diagnostics + progress + finished)


def test_cargo_wrapper_preserves_unknown_output(tmp_path):
    """Unknown output and diagnostic-like progress text are passed through unchanged."""
    stderr = b"Cargo diagnostic without a standard prefix\n   Compiling fixture v0.1.0\n"
    result = run(
        [sys.executable, str(WRAPPER), sys.executable, "-c", f"import os; os.write(2, {stderr!r})"],
        tmp_path,
    )
    assert result.returncode == 0
    assert result.stderr == stderr


def test_cargo_wrapper_reports_missing_executable(tmp_path):
    """A failed process launch remains a visible build failure."""
    result = run([sys.executable, str(WRAPPER), str(tmp_path / "missing-cargo")], tmp_path)
    assert result.returncode == 127
    assert b"error: could not start Cargo:" in result.stderr


# Windows has no POSIX signal exit convention; do not collect an inapplicable
# test there, since platform CI deliberately treats skipped tests as errors.
if os.name != "nt":
    def test_cargo_wrapper_preserves_signal_failure(tmp_path):
        """Signal termination must not become a successful wrapper invocation."""
        result = run(
            [sys.executable, str(WRAPPER), sys.executable, "-c",
             "import os, signal; os.kill(os.getpid(), signal.SIGTERM)"], tmp_path,
        )
        assert result.returncode == 128 + signal.SIGTERM


def find_tool(name):
    """Locate build tools on PATH or beside the current Python interpreter."""
    executable = name + (".exe" if os.name == "nt" else "")
    local = Path(sys.executable).with_name(executable)
    return shutil.which(executable) or (str(local) if local.is_file() else None)


CMAKE = find_tool("cmake")
CARGO = find_tool("cargo")
NINJA = find_tool("ninja")
GENERATORS = [None] if os.name == "nt" else ["Unix Makefiles"]
if NINJA and (os.name != "nt" or os.environ.get("VCINSTALLDIR")):
    GENERATORS.extend(["Ninja", "Ninja Multi-Config"])


@pytest.mark.skipif(not CMAKE or not CARGO, reason="CMake and Cargo are required")
@pytest.mark.parametrize("generator", GENERATORS)
def test_rust_clean_cached_verbose_and_failed_builds(tmp_path_factory, generator):
    """Keep native progress concise without suppressing real Cargo or Rust diagnostics."""
    versions = (REPOSITORY_ROOT / "src/cmake/bskRustSupportVersions.cmake").read_text(encoding="utf-8")
    revision = re.search(r'set\(BSK_CORROSION_GIT_TAG "([^"\n]+)"\)', versions).group(1)
    cache = REPOSITORY_ROOT / ".bsk-cache/fetchcontent"
    corrosion_file = cache / f"corrosion-{revision}/cmake/Corrosion.cmake"
    if not corrosion_file.is_file():
        pytest.skip("The pinned Corrosion source cache is required; this test never downloads it")
    original_corrosion = corrosion_file.read_bytes()
    # Cargo adds deep artifact paths. A short test root avoids MSVC linker path
    # limits while retaining spaces in the project and build directory names.
    tmp_path = tmp_path_factory.mktemp("rust")
    project = tmp_path / "src space"
    build = tmp_path / "build space"
    project.mkdir()
    manifest = project / "Cargo.toml"
    manifest_text = ('''[package]
name = "output_fixture"
version = "0.1.0"
edition = "2021"
[lib]
path = "lib.rs"
crate-type = ["staticlib"]
''')
    manifest.write_text(manifest_text, encoding="utf-8")
    (project / "lib.rs").write_text("pub fn fixture() {}\n", encoding="utf-8")
    (project / "build.rs").write_text('fn main() { println!("cargo:rerun-if-changed=build.rs"); }\n', encoding="utf-8")
    environment = os.environ.copy()
    environment["CARGO_NET_OFFLINE"] = "true"
    environment["CARGO_TERM_COLOR"] = "never"
    environment.pop("CARGO_TERM_QUIET", None)
    environment.pop("CARGO_TERM_VERBOSE", None)
    lock = run([CARGO, "generate-lockfile", "--offline"], project, environment)
    assert lock.returncode == 0, lock.stdout + lock.stderr
    helpers = (REPOSITORY_ROOT / "src/cmake").as_posix()
    (project / "CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.26)
project(rustOutput C)
include("{helpers}/bskAddRustModuleSources.cmake")
include("{helpers}/bskRustBuildOutput.cmake")
set(Python3_EXECUTABLE "{Path(sys.executable).as_posix()}")
set(BSK_FETCHCONTENT_CACHE_DIR "{cache.as_posix()}")
set(BSK_OFFLINE_BUILD ON)
_bsk_load_corrosion()
corrosion_import_crate(MANIFEST_PATH "${{CMAKE_SOURCE_DIR}}/Cargo.toml"
  CRATES output_fixture CRATE_TYPES staticlib LOCKED)
bsk_configure_rust_build_output(output_fixture)
''', encoding="utf-8")
    configure = [CMAKE, "-S", str(project), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release"]
    if generator:
        configure.extend(["-G", generator])
    if generator and generator.startswith("Ninja"):
        configure.append(f"-DCMAKE_MAKE_PROGRAM={NINJA}")
    if sys.platform == "darwin" and not os.environ.get("SDKROOT"):
        configure.append("-DCMAKE_OSX_SYSROOT=macosx")
    compile_command = [CMAKE, "--build", str(build), "--config", "Release"]
    for verbose in (False, False, True, False):
        configured = run(configure + ["--log-level=VERBOSE" if verbose else "--log-level=NOTICE"], project, environment)
        assert configured.returncode == 0, configured.stdout + configured.stderr
        compiled = run(compile_command, project, environment)
        output = compiled.stdout + compiled.stderr
        assert compiled.returncode == 0, output
        assert b"Building Rust module output_fixture" in output
        if verbose:
            assert b"Finished `release`" in output
        else:
            # Toolchains can warn even for this fixture. Once a warning starts,
            # the wrapper deliberately preserves all subsequent output.
            before_warning = output.partition(b"warning:")[0]
            assert b"Finished `release`" not in before_warning
            assert b"   Compiling " not in before_warning
        if generator and generator.startswith("Ninja"):
            assert b" && " not in output
        assert corrosion_file.read_bytes() == original_corrosion

    manifest.write_text(manifest_text.replace('[lib]', 'unused_manifest_key = true\n[lib]'), encoding="utf-8")
    (project / "build.rs").write_text('fn main() { println!("cargo:warning=BSK build-script warning sentinel"); }\n', encoding="utf-8")
    (project / "lib.rs").write_text("pub fn fixture() { let unused_rust_sentinel = 1; }\n", encoding="utf-8")
    warned = run(compile_command, project, environment)
    warning_output = warned.stdout + warned.stderr
    assert warned.returncode == 0, warning_output
    assert b"unused manifest key" in warning_output
    assert b"BSK build-script warning sentinel" in warning_output
    assert b"unused_rust_sentinel" in warning_output
    assert b"Finished `release`" in warning_output

    (project / "lib.rs").write_text('compile_error!("BSK Rust error sentinel");\n', encoding="utf-8")
    failed = run(compile_command, project, environment)
    assert failed.returncode != 0
    assert b"BSK Rust error sentinel" in failed.stdout + failed.stderr

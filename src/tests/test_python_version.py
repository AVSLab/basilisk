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

"""Verify Python compatibility checks without building native modules."""

import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys

import pytest
from packaging.markers import default_environment
from packaging.requirements import Requirement

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
VERSION_CHECK_SCRIPT = Path("src/buildSupport/checkPythonVersion.py")
CMAKE_NAME = "cmake.exe" if os.name == "nt" else "cmake"
ENVIRONMENT_CMAKE = Path(sys.executable).with_name(CMAKE_NAME)
CMAKE = shutil.which(CMAKE_NAME) or (
    str(ENVIRONMENT_CMAKE) if ENVIRONMENT_CMAKE.is_file() else None
)


@pytest.fixture(scope="module")
def version_checker():
    """Load the shared validator without importing build tools."""
    return runpy.run_path(str(REPOSITORY_ROOT / VERSION_CHECK_SCRIPT))


@pytest.mark.parametrize("version, supported", [
    ("3.8.20", False),
    ("3.9.0", True),
    ("3.10.0", True),
    ("3.11.0", True),
    ("3.12.0", True),
    ("3.13.0", True),
    ("3.14.0", True),
    ("3.14.99", True),
    ("3.15.0", True),
    ("3.15.2", True),
    ("3.15.99", True),
    ("3.16.0", False),
    ("4.0.0", False),
])
def test_python_version_boundaries(version_checker, version, supported):
    """Enforce both endpoints while preserving every supported minor version."""
    metadata = REPOSITORY_ROOT / "pyproject.toml"
    check_version = version_checker["check_python_version"]
    if supported:
        check_version(metadata, version)
    else:
        with pytest.raises(ValueError, match="Basilisk requires Python") as error:
            check_version(metadata, version)
        assert f"found Python {version}" in str(error.value)


@pytest.fixture(scope="module")
def dependency_groups():
    """Read the build, runtime, development, and example requirements."""
    with (REPOSITORY_ROOT / "pyproject.toml").open("rb") as metadata_file:
        metadata = tomllib.load(metadata_file)
    return {
        "build": metadata["build-system"]["requires"],
        "runtime": (REPOSITORY_ROOT / "requirements.txt").read_text(encoding="utf-8").splitlines(),
        "development": (REPOSITORY_ROOT / "requirements_dev.txt").read_text(encoding="utf-8").splitlines(),
        "examples": metadata["project"]["optional-dependencies"]["examples"],
    }


def active_requirement(requirements, name, python_version):
    """Select the unique requirement for a dependency on a target interpreter."""
    environment = default_environment()
    environment.update(python_version=python_version, python_full_version=f"{python_version}.0")
    active = []
    for line in requirements:
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        requirement = Requirement(line)
        if requirement.name == name and (
            requirement.marker is None or requirement.marker.evaluate(environment)
        ):
            active.append(requirement)
    assert len(active) == 1, (name, python_version, active)
    return active[0]


@pytest.mark.parametrize("python_version, numpy_version, numba_version", [
    ("3.9", "2.0.2", "0.60.0"),
    ("3.10", "2.2.6", "0.68.0"),
    ("3.11", "2.3.5", "0.68.0"),
    ("3.12", "2.3.5", "0.67.0"),
    ("3.13", "2.3.5", "0.67.0"),
    ("3.14", "2.3.5", "0.67.0"),
    ("3.15", "2.5.2", "0.68.0"),
])
def test_dependency_ranges_preserve_supported_interpreters(
        dependency_groups, python_version, numpy_version, numba_version,
):
    """Keep compatible dependency choices available across Python 3.9 through 3.15."""
    for group in ("build", "runtime"):
        requirement = active_requirement(dependency_groups[group], "numpy", python_version)
        assert requirement.specifier.contains(numpy_version)
    for group in ("development", "examples"):
        requirement = active_requirement(dependency_groups[group], "numba", python_version)
        assert requirement.specifier.contains(numba_version)


def test_python315_excludes_incompatible_dependencies(dependency_groups):
    """Prevent older NumPy and Numba releases from being selected on Python 3.15."""
    for group in ("build", "runtime"):
        requirement = active_requirement(dependency_groups[group], "numpy", "3.15")
        assert not requirement.specifier.contains("2.5.1")
    for group in ("development", "examples"):
        requirement = active_requirement(dependency_groups[group], "numba", "3.15")
        assert not requirement.specifier.contains("0.67.0")


def test_updated_metadata_controls_supported_versions(tmp_path, version_checker):
    """Honor changed bounds and exclusions without changing the validator."""
    metadata = tmp_path / "pyproject.toml"
    metadata.write_text(
        '[project]\nrequires-python = ">=3.10, <3.16, !=3.12.*"\n', encoding="utf-8",
    )
    check_version = version_checker["check_python_version"]
    check_version(metadata, "3.15.2")
    for version in ("3.9.20", "3.12.0", "3.16.0"):
        with pytest.raises(ValueError, match="Basilisk requires Python"):
            check_version(metadata, version)


@pytest.mark.parametrize("content", [
    "[project]\n",
    'project = "invalid"\n',
    '[project]\nrequires-python = ""\n',
    '[project]\nrequires-python = 315\n',
    '[project]\nrequires-python = "invalid"\n',
    "[project\n",
])
def test_invalid_metadata_stops_validation(tmp_path, version_checker, capsys, content):
    """Fail with a diagnostic instead of silently accepting incomplete metadata."""
    metadata = tmp_path / "pyproject.toml"
    metadata.write_text(content, encoding="utf-8")
    assert version_checker["main"]([str(metadata)]) == 1
    assert "Python version check failed:" in capsys.readouterr().err


def test_missing_metadata_stops_validation(tmp_path, version_checker, capsys):
    """Report a missing metadata file as a failed check."""
    assert version_checker["main"]([str(tmp_path / "missing.toml")]) == 1
    assert "missing.toml" in capsys.readouterr().err


def test_conan_script_rejects_before_imports_or_clean(tmp_path):
    """Reject an unsupported interpreter before importing Conan or touching outputs."""
    repository = tmp_path / "repository with spaces"
    for resource in (Path("conanfile.py"), VERSION_CHECK_SCRIPT):
        destination = repository / resource
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPOSITORY_ROOT / resource, destination)
    (repository / "pyproject.toml").write_text(
        '[project]\nrequires-python = ">=99"\n', encoding="utf-8",
    )
    (repository / "conan.py").write_text(
        'raise RuntimeError("Conan must not be imported")\n', encoding="utf-8",
    )
    existing_output = repository / "dist3" / "keep.txt"
    existing_output.parent.mkdir()
    existing_output.write_text("existing build", encoding="utf-8")
    # The invoking directory's metadata must not override the repository's range.
    (tmp_path / "pyproject.toml").write_text(
        '[project]\nrequires-python = ">=0"\n', encoding="utf-8",
    )

    result = subprocess.run(
        [sys.executable, str(repository / "conanfile.py"), "--clean"],
        cwd=tmp_path, capture_output=True, text=True, check=False, timeout=30,  # [s]
    )

    assert result.returncode == 1
    assert "Basilisk requires Python >=99" in result.stderr
    assert sys.executable in result.stderr
    assert "Conan must not be imported" not in result.stderr
    assert "Traceback" not in result.stderr
    assert existing_output.read_text(encoding="utf-8") == "existing build"


@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
def test_cmake_checks_selected_interpreter_and_rechecks_changed_metadata(tmp_path):
    """Validate the actual executable and rerun the check when metadata changes."""
    source = tmp_path / "source with spaces"
    source.mkdir()
    metadata = source / "pyproject.toml"
    metadata.write_text('[project]\nrequires-python = ">=0"\n', encoding="utf-8")
    module = (REPOSITORY_ROOT / "src/cmake/bskCheckPythonVersion.cmake").as_posix()
    (source / "CMakeLists.txt").write_text(
        f'''cmake_minimum_required(VERSION 3.26)
project(PythonVersionCheck NONE)
include("{module}")
set(Python3_EXECUTABLE "{Path(sys.executable).as_posix()}")
# Deliberately inconsistent: validation must inspect the executable itself.
set(Python3_VERSION "99.0.0")
bsk_check_python_version("${{CMAKE_CURRENT_SOURCE_DIR}}/pyproject.toml")
''', encoding="utf-8",
    )
    build = tmp_path / "build"
    configure = subprocess.run(
        [CMAKE, "-S", str(source), "-B", str(build)],
        capture_output=True, text=True, check=False, timeout=30,  # [s]
    )
    assert configure.returncode == 0, configure.stdout + configure.stderr

    metadata.write_text('[project]\nrequires-python = ">=99"\n', encoding="utf-8")
    rebuild = subprocess.run(
        [CMAKE, "--build", str(build)],
        capture_output=True, text=True, check=False, timeout=30,  # [s]
    )
    assert rebuild.returncode != 0
    output = " ".join((rebuild.stdout + rebuild.stderr).split())
    assert "Basilisk requires Python >=99" in output
    actual_version = ".".join(str(component) for component in sys.version_info[:3])
    assert f"found Python {actual_version}" in output

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

"""Check native library creation and cleanup for the combined binding layouts."""

import importlib.machinery
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


SOURCE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "generateFswBindings", SOURCE / "cmake/generateFswBindings.py")
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)
SUBPROCESS_TIMEOUT_SECONDS = 180  # [s]


def find_build_tool(name):
    """Find a build tool on PATH or beside the active Python executable."""
    executable = name + (".exe" if os.name == "nt" else "")
    local = Path(sys.executable).with_name(executable)
    return shutil.which(executable) or (str(local) if local.is_file() else None)


CMAKE = find_build_tool("cmake")
NINJA = find_build_tool("ninja")
BUILD_GENERATORS = [None] if os.name == "nt" else ["Unix Makefiles"]
if NINJA and (os.name != "nt" or os.environ.get("VCINSTALLDIR")):
    BUILD_GENERATORS.append("Ninja")
if sys.platform == "darwin":
    BUILD_GENERATORS.append("Xcode")


def test_layout_switch_preserves_optional_bindings_and_retires_stale_files(tmp_path):
    """A changed group replaces stale binaries and removes only its old shims.

    :param tmp_path: Temporary directory supplied by pytest.
    """
    package = tmp_path / "fswAlgorithms"
    package.mkdir()
    manifest = tmp_path / "fswCoreBindings.txt"
    loader = SOURCE / "fswAlgorithms/_load_fsw.py"
    suffix = importlib.machinery.EXTENSION_SUFFIXES[0]
    optional = package / f"_optional{suffix}"
    optional.write_bytes(b"optional native binding")
    for name in ("first", "second"):
        (package / f"_{name}{suffix}").touch()
    manifest.write_text("first\nsecond\n")
    GENERATOR.generate_layout(manifest, package, loader)
    assert not (package / f"_first{suffix}").exists()
    assert not (package / f"_second{suffix}").exists()
    first = package / "_first.py"
    first_time = first.stat().st_mtime_ns
    GENERATOR.generate_layout(manifest, package, loader)
    assert first.stat().st_mtime_ns == first_time
    manifest.write_text("first\n")
    GENERATOR.generate_layout(manifest, package, loader)
    assert first.exists()
    assert not (package / "_second.py").exists()
    combined = package / f"_fswCoreNative{suffix}"
    combined.touch()
    manifest.write_text("")
    GENERATOR.generate_layout(manifest, package, loader)
    assert not first.exists()
    assert not combined.exists()
    assert not (package / "_load_fsw.py").exists()
    assert optional.read_bytes() == b"optional native binding"


@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("build_generator", BUILD_GENERATORS)
@pytest.mark.parametrize("group", ["fswCoreNative", "simulationCoreNative", "mujocoNative", "opNavNative"])
def test_combined_library_is_linked_and_reconfigure_is_incremental(tmp_path, build_generator, group):
    """Build and import both native entry points, then check an unchanged rebuild.

    :param tmp_path: Temporary directory supplied by pytest.
    :param build_generator: CMake generator used for the isolated native build.
    :param group: Native binding container to build.
    """
    if build_generator == "Xcode":
        xcodebuild = shutil.which("xcodebuild")
        if xcodebuild is None:
            pytest.skip("Xcode is not installed")
        version = subprocess.run([xcodebuild, "-version"], capture_output=True, text=True,
                                 timeout=SUBPROCESS_TIMEOUT_SECONDS)
        if version.returncode:
            pytest.skip("A full Xcode installation must be selected to test its generator")

    project = tmp_path / "source"
    build = tmp_path / "build"
    (project / "cmake").mkdir(parents=True)
    (project / "fswAlgorithms").mkdir()
    shutil.copyfile(SOURCE / "cmake/generateFswBindings.py", project / "cmake/generateFswBindings.py")
    shutil.copyfile(SOURCE / "cmake/generateGroupedBindings.py", project / "cmake/generateGroupedBindings.py")
    shutil.copyfile(SOURCE / "fswAlgorithms/_load_fsw.py", project / "fswAlgorithms/_load_fsw.py")
    for value, name in enumerate(("first", "second"), start=1):
        (project / f"{name}.cpp").write_text(
            f"""#include <Python.h>
static PyObject *value(PyObject *, PyObject *) {{ return PyLong_FromLong({value}); }}
static PyMethodDef methods[] = {{
    {{"value", value, METH_NOARGS, nullptr}}, {{nullptr, nullptr, 0, nullptr}}
}};
static PyModuleDef module = {{PyModuleDef_HEAD_INIT, "_{name}", nullptr, -1, methods}};
PyMODINIT_FUNC PyInit__{name}(void) {{ return PyModule_Create(&module); }}
""", encoding="utf-8",
        )
    if group == "fswCoreNative":
        helper = SOURCE / "cmake/bskCombineFswBindings.cmake"
        registration = """set_property(GLOBAL APPEND PROPERTY BSK_FSW_OBJECT_TARGETS ${name}Objects)
  set_property(GLOBAL APPEND PROPERTY BSK_FSW_COMBINED_MODULES ${name})
  set_property(GLOBAL APPEND PROPERTY BSK_FSW_BINDING_TARGETS ${name}Objects)"""
        finalizer = "bsk_finalize_fsw_bindings()"
        packages = ("fswAlgorithms", "fswAlgorithms")
        output_package = "fswAlgorithms"
    else:
        helper = SOURCE / "cmake/bskCombineSimulationBindings.cmake"
        packages = ("simulation", "fswAlgorithms" if group == "opNavNative" else "simulation")
        output_package = "" if group == "opNavNative" else "simulation"
        registration = f"""set_property(GLOBAL APPEND PROPERTY BSK_{group}_OBJECT_TARGETS ${{name}}Objects)
  if(name STREQUAL "first")
    set(package "{packages[0]}")
  else()
    set(package "{packages[1]}")
  endif()
  set_property(GLOBAL APPEND PROPERTY BSK_{group}_MODULES "${{package}}.${{name}}")
  set_property(GLOBAL APPEND PROPERTY BSK_SIMULATION_BINDING_TARGETS ${{name}}Objects)"""
        finalizer = "bsk_finalize_simulation_bindings()"
    (project / "CMakeLists.txt").write_text(
        f"""cmake_minimum_required(VERSION 3.26)
project(bindingBuildTest LANGUAGES CXX)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
include("{helper.as_posix()}")
foreach(name first second)
  add_library(${{name}}Objects OBJECT ${{name}}.cpp)
  set_target_properties(${{name}}Objects PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_STANDARD 17)
  target_link_libraries(${{name}}Objects PRIVATE Python3::Module)
  {registration}
endforeach()
{finalizer}
file(MAKE_DIRECTORY "${{CMAKE_BINARY_DIR}}/Basilisk/fswAlgorithms")
file(MAKE_DIRECTORY "${{CMAKE_BINARY_DIR}}/Basilisk/simulation")
file(WRITE "${{CMAKE_BINARY_DIR}}/Basilisk/__init__.py" "")
file(WRITE "${{CMAKE_BINARY_DIR}}/Basilisk/fswAlgorithms/__init__.py" "")
file(WRITE "${{CMAKE_BINARY_DIR}}/Basilisk/simulation/__init__.py" "")
""", encoding="utf-8",
    )

    def run(command):
        """Run a fixture command and include its output when it fails."""
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=SUBPROCESS_TIMEOUT_SECONDS)
        assert result.returncode == 0, result.stdout + result.stderr

    configure = [CMAKE, "-S", str(project), "-B", str(build),
                 f"-DPython3_EXECUTABLE={sys.executable}", "-DCMAKE_BUILD_TYPE=Release"]
    if build_generator:
        configure.extend(["-G", build_generator])
    if build_generator == "Ninja":
        configure.append(f"-DCMAKE_MAKE_PROGRAM={NINJA}")
    build_command = [CMAKE, "--build", str(build), "--target", group, "--config", "Release"]
    run(configure)
    run(build_command)
    suffix = ".pyd" if os.name == "nt" else ".so"
    library = build / "Basilisk" / output_package / f"_{group}{suffix}"
    assert library.is_file(), "The build succeeded without producing the combined library"
    run([sys.executable, "-c", f"""import sys
sys.path.insert(0, sys.argv[1])
from Basilisk.{packages[0]} import _first
from Basilisk.{packages[1]} import _second
assert _first.value() == 1
assert _second.value() == 2
assert _first.__file__ == _second.__file__
""", str(build)])
    original_mtime = library.stat().st_mtime_ns
    run(configure)
    run(build_command)
    assert library.stat().st_mtime_ns == original_mtime

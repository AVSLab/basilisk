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

from concurrent.futures import ThreadPoolExecutor
import importlib.machinery
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

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
SWIG = find_build_tool("swig")
BUILD_GENERATORS = [None] if os.name == "nt" else ["Unix Makefiles"]
if NINJA and (os.name != "nt" or os.environ.get("VCINSTALLDIR")):
    BUILD_GENERATORS.append("Ninja")
if sys.platform == "darwin":
    BUILD_GENERATORS.append("Xcode")

# Exercise the full compiler/API matrix once, plus one representative real SWIG
# build with each other generator to retain coverage of its generated build graph.
DIRECTOR_GENERATOR = "Ninja" if "Ninja" in BUILD_GENERATORS else BUILD_GENERATORS[0]
DIRECTOR_CONFIGURATIONS = [
    pytest.param(generator, limited_api, threads,
                 id=f"{generator}-{'limited' if limited_api else 'full'}-api-"
                    f"{'threads' if threads else 'no-threads'}")
    for generator in BUILD_GENERATORS
    for limited_api in (False, True)
    for threads in (False, True)
    if generator == DIRECTOR_GENERATOR or (limited_api and threads)
]


@pytest.mark.parametrize("update_order", ["source-first", "destination-first", "parallel"])
@pytest.mark.parametrize("source_group,destination_group,package_name", [
    ("simulationCoreNative", "mujocoNative", "simulation"),
    ("mujocoNative", "simulationCoreNative", "simulation"),
    ("simulationCoreNative", "opNavNative", "simulation"),
    ("opNavNative", "simulationCoreNative", "simulation"),
    ("mujocoNative", "opNavNative", "simulation"),
    ("opNavNative", "mujocoNative", "simulation"),
    ("fswCoreNative", "opNavNative", "fswAlgorithms"),
    ("opNavNative", "fswCoreNative", "fswAlgorithms"),
])
def test_moving_bindings_preserves_destination_shims(
    tmp_path, source_group, destination_group, package_name, update_order,
):
    """Moving bindings preserves their new shims regardless of layout update order.

    :param tmp_path: Temporary directory supplied by pytest.
    :param source_group: Group that previously owned the moving binding.
    :param destination_group: Group that will own the moving binding.
    :param package_name: Python package containing the binding.
    :param update_order: Order in which the two layout generators run.
    """
    spec = importlib.util.spec_from_file_location(
        "generateGroupedBindings", SOURCE / "cmake/generateGroupedBindings.py")
    grouped_generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(grouped_generator)
    root = tmp_path / "Basilisk"
    package = root / package_name
    loader = SOURCE / "fswAlgorithms/_load_fsw.py"
    manifests = {group: tmp_path / f"{group}.txt" for group in (source_group, destination_group)}
    active_manifest = tmp_path / "combinedBindingModules.txt"

    def write_manifest(group, names):
        """Write the group's own manifest format."""
        prefix = "" if group == "fswCoreNative" else package_name + "."
        manifests[group].write_text("".join(prefix + name + "\n" for name in names), encoding="utf-8")

    def generate(group):
        """Run the real generator for one group's layout."""
        if group == "fswCoreNative":
            GENERATOR.generate_layout(manifests[group], package, loader, active_manifest)
        else:
            native = f"_{group}" if group == "opNavNative" else f"simulation._{group}"
            loader_name = f"_load_{group}"
            grouped_generator.generate_layout(manifests[group], root, loader, native, loader_name, active_manifest)

    write_manifest(source_group, ["moving", "retired"])
    write_manifest(destination_group, [])
    active_manifest.write_text(f"{package_name}.moving\n{package_name}.retired\n", encoding="utf-8")
    generate(source_group)
    generate(destination_group)
    write_manifest(source_group, [])
    write_manifest(destination_group, ["moving", "added"])
    active_manifest.write_text(f"{package_name}.moving\n{package_name}.added\n", encoding="utf-8")
    if update_order == "parallel":
        with ThreadPoolExecutor(max_workers=2) as executor:
            list(executor.map(generate, (source_group, destination_group)))
    else:
        order = (source_group, destination_group)
        if update_order == "destination-first":
            order = tuple(reversed(order))
        for group in order:
            generate(group)

    moving = package / "_moving.py"
    assert moving.is_file(), "The former group deleted the destination group's shim"
    expected_loader = ("from ._load_fsw" if destination_group == "fswCoreNative"
                       else f"from Basilisk._load_{destination_group}")
    assert expected_loader in moving.read_text(encoding="utf-8")
    assert (package / "_added.py").is_file()
    assert not (package / "_retired.py").exists()
    original_mtime = moving.stat().st_mtime_ns
    generate(source_group)
    generate(destination_group)
    assert moving.stat().st_mtime_ns == original_mtime
    write_manifest(destination_group, [])
    active_manifest.write_text("", encoding="utf-8")
    generate(destination_group)
    assert not moving.exists()
    assert not (package / "_added.py").exists()


def test_layout_switch_preserves_optional_bindings_and_retires_stale_files(tmp_path):
    """A changed group replaces stale binaries and removes only its old shims.

    :param tmp_path: Temporary directory supplied by pytest.
    """
    package = tmp_path / "fswAlgorithms"
    package.mkdir()
    manifest = tmp_path / "fswCoreBindings.txt"
    active_manifest = tmp_path / "combinedBindingModules.txt"
    loader = SOURCE / "fswAlgorithms/_load_fsw.py"
    suffix = importlib.machinery.EXTENSION_SUFFIXES[0]
    optional = package / f"_optional{suffix}"
    optional.write_bytes(b"optional native binding")
    for name in ("first", "second"):
        (package / f"_{name}{suffix}").touch()
    manifest.write_text("first\nsecond\n")
    active_manifest.write_text("fswAlgorithms.first\nfswAlgorithms.second\n", encoding="utf-8")
    GENERATOR.generate_layout(manifest, package, loader, active_manifest)
    assert not (package / f"_first{suffix}").exists()
    assert not (package / f"_second{suffix}").exists()
    first = package / "_first.py"
    first_time = first.stat().st_mtime_ns
    GENERATOR.generate_layout(manifest, package, loader, active_manifest)
    assert first.stat().st_mtime_ns == first_time
    manifest.write_text("first\n")
    active_manifest.write_text("fswAlgorithms.first\n", encoding="utf-8")
    GENERATOR.generate_layout(manifest, package, loader, active_manifest)
    assert first.exists()
    assert not (package / "_second.py").exists()
    combined = package / f"_fswCoreNative{suffix}"
    combined.touch()
    manifest.write_text("")
    active_manifest.write_text("", encoding="utf-8")
    GENERATOR.generate_layout(manifest, package, loader, active_manifest)
    assert not first.exists()
    assert not combined.exists()
    assert not (package / "_load_fsw.py").exists()
    assert optional.read_bytes() == b"optional native binding"


@pytest.mark.ciSkip
@pytest.mark.buildIntegration
@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("build_generator", BUILD_GENERATORS)
def test_cmake_binding_moves_preserve_shims_after_reconfiguration(tmp_path, build_generator):
    """CMake protects moved shims and still removes bindings retired from all groups.

    :param tmp_path: Temporary directory supplied by pytest.
    :param build_generator: CMake generator used for the isolated layout build.
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
    for name in ("generateFswBindings.py", "generateGroupedBindings.py"):
        shutil.copyfile(SOURCE / "cmake" / name, project / "cmake" / name)
    shutil.copyfile(SOURCE / "fswAlgorithms/_load_fsw.py", project / "fswAlgorithms/_load_fsw.py")
    (project / "CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.26)
project(bindingMoveTest LANGUAGES NONE)
include("{SOURCE.as_posix()}/cmake/bskCombineFswBindings.cmake")
include("{SOURCE.as_posix()}/cmake/bskCombineSimulationBindings.cmake")
if(MOVE_BINDINGS)
  set_property(GLOBAL PROPERTY BSK_mujocoNative_MODULES simulation.moving)
  set_property(GLOBAL PROPERTY BSK_opNavNative_MODULES fswAlgorithms.moving)
else()
  set_property(GLOBAL PROPERTY BSK_FSW_COMBINED_MODULES moving retired)
  set_property(GLOBAL PROPERTY BSK_simulationCoreNative_MODULES simulation.moving simulation.retired)
endif()
bsk_finalize_fsw_bindings()
bsk_finalize_simulation_bindings()
add_custom_target(sourceLayouts DEPENDS fswBindingsLayout simulationCoreNativeLayout)
add_custom_target(destinationLayouts DEPENDS opNavNativeLayout mujocoNativeLayout)
add_custom_target(allLayouts DEPENDS sourceLayouts destinationLayouts)
''', encoding="utf-8")

    def run(command):
        """Run a CMake command and include its output when it fails."""
        result = subprocess.run(command, capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT_SECONDS)
        assert result.returncode == 0, result.stdout + result.stderr

    def prepare_reconfiguration():
        """Separate timestamps for Make versions with whole-second resolution."""
        if build_generator == "Unix Makefiles":
            timestamp = time.time_ns() - 2_000_000_000  # [ns]
            # Age both inputs and outputs so only changed manifests trigger work.
            # This models a later reconfiguration without sleeping in the test.
            for directory in (build / "autoSource", build / "Basilisk",
                              project / "cmake", project / "fswAlgorithms"):
                for path in directory.rglob("*"):
                    if path.is_file():
                        os.utime(path, ns=(timestamp, timestamp))

    configure = [CMAKE, "-S", str(project), "-B", str(build), f"-DPython3_EXECUTABLE={sys.executable}"]
    if build_generator:
        configure.extend(["-G", build_generator])
    if build_generator == "Ninja":
        configure.append(f"-DCMAKE_MAKE_PROGRAM={NINJA}")
    build_command = [CMAKE, "--build", str(build), "--config", "Release", "--parallel", "4", "--target"]
    run(configure + ["-DMOVE_BINDINGS=OFF"])
    run(build_command + ["allLayouts"])
    prepare_reconfiguration()
    run(configure + ["-DMOVE_BINDINGS=ON"])
    run(build_command + ["destinationLayouts"])
    shims = [build / "Basilisk" / package / "_moving.py" for package in ("fswAlgorithms", "simulation")]
    destination_text = [shim.read_text(encoding="utf-8") for shim in shims]
    assert "Basilisk._opNavNative" in destination_text[0]
    assert "Basilisk.simulation._mujocoNative" in destination_text[1]
    run(build_command + ["sourceLayouts"])
    assert [shim.read_text(encoding="utf-8") for shim in shims] == destination_text
    assert not any(shim.with_name("_retired.py").exists() for shim in shims)

    timestamps = [shim.stat().st_mtime_ns for shim in shims]
    run(configure + ["-DMOVE_BINDINGS=ON"])
    run(build_command + ["allLayouts"])
    assert [shim.stat().st_mtime_ns for shim in shims] == timestamps
    # Move back with the previous owners' cleanup running last again.
    prepare_reconfiguration()
    run(configure + ["-DMOVE_BINDINGS=OFF"])
    run(build_command + ["sourceLayouts"])
    run(build_command + ["destinationLayouts"])
    assert "from ._load_fsw" in shims[0].read_text(encoding="utf-8")
    assert "Basilisk.simulation._simulationCoreNative" in shims[1].read_text(encoding="utf-8")


@pytest.mark.ciSkip
@pytest.mark.buildIntegration
@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("build_generator", BUILD_GENERATORS)
def test_combined_library_is_linked_and_reconfigure_is_incremental(tmp_path, build_generator):
    """Build all four groups together and verify their imports and unchanged rebuild.

    :param tmp_path: Temporary directory supplied by pytest.
    :param build_generator: CMake generator used for the isolated native build.
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
    declarations = [f'''cmake_minimum_required(VERSION 3.26)
project(bindingBuildTest LANGUAGES CXX)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
include("{SOURCE.as_posix()}/cmake/bskCombineFswBindings.cmake")
include("{SOURCE.as_posix()}/cmake/bskCombineSimulationBindings.cmake")
''']
    groups = ("fswCoreNative", "simulationCoreNative", "mujocoNative", "opNavNative")
    libraries = []
    probe = ["import importlib, sys", "sys.path.insert(0, sys.argv[1])", "native_files = set()"]
    suffix = ".pyd" if os.name == "nt" else ".so"
    # Share compiler discovery and generator startup, retaining two native
    # entry points per group and opNav imports from both public packages.
    for group in groups:
        packages = (("fswAlgorithms", "fswAlgorithms") if group == "fswCoreNative" else
                    ("simulation", "fswAlgorithms" if group == "opNavNative" else "simulation"))
        output_package = "" if group == "opNavNative" else packages[0]
        libraries.append(build / "Basilisk" / output_package / f"_{group}{suffix}")
        probe.append("modules = []")
        for value, package in enumerate(packages, start=1):
            name = f"{group}{value}"
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
            declarations.append(f'''add_library({name}Objects OBJECT {name}.cpp)
set_target_properties({name}Objects PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_STANDARD 17)
target_link_libraries({name}Objects PRIVATE Python3::Module)
''')
            if group == "fswCoreNative":
                declarations.append(f'''set_property(GLOBAL APPEND PROPERTY BSK_FSW_OBJECT_TARGETS {name}Objects)
set_property(GLOBAL APPEND PROPERTY BSK_FSW_COMBINED_MODULES {name})
set_property(GLOBAL APPEND PROPERTY BSK_FSW_BINDING_TARGETS {name}Objects)
''')
            else:
                declarations.append(f'''set_property(GLOBAL APPEND PROPERTY BSK_{group}_OBJECT_TARGETS {name}Objects)
set_property(GLOBAL APPEND PROPERTY BSK_{group}_MODULES {package}.{name})
set_property(GLOBAL APPEND PROPERTY BSK_SIMULATION_BINDING_TARGETS {name}Objects)
''')
            probe.extend([
                f"module = importlib.import_module('Basilisk.{package}._{name}')",
                f"assert module.value() == {value}",
                "modules.append(module)",
            ])
        probe.extend([
            "assert modules[0].__file__ == modules[1].__file__",
            f"assert modules[0].__file__.endswith('_{group}{suffix}')",
            "native_files.add(modules[0].__file__)",
        ])
    probe.append("assert len(native_files) == 4")
    declarations.append('''bsk_finalize_fsw_bindings()
bsk_finalize_simulation_bindings()
add_custom_target(allBindings DEPENDS fswCoreNative simulationCoreNative mujocoNative opNavNative)
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/Basilisk/simulation")
file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/__init__.py" "")
file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/fswAlgorithms/__init__.py" "")
file(WRITE "${CMAKE_BINARY_DIR}/Basilisk/simulation/__init__.py" "")
''')
    (project / "CMakeLists.txt").write_text("\n".join(declarations), encoding="utf-8")

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
    build_command = [CMAKE, "--build", str(build), "--target", "allBindings", "--config", "Release"]
    run(configure)
    run(build_command)
    for library in libraries:
        assert library.is_file(), f"The build succeeded without producing {library}"
    run([sys.executable, "-I", "-c", "\n".join(probe), str(build)])
    original_mtimes = [library.stat().st_mtime_ns for library in libraries]
    run(configure)
    run(build_command)
    assert [library.stat().st_mtime_ns for library in libraries] == original_mtimes


@pytest.mark.parametrize("semicolon", [";", ""], ids=["swig44", "swig45"])
def test_director_mutex_syntax_variants(tmp_path, semicolon):
    """Accept the mutex forms emitted by both supported SWIG runtime versions.

    :param tmp_path: Temporary directory supplied by pytest.
    :param semicolon: Terminator emitted after the macro invocation.
    """
    spec = importlib.util.spec_from_file_location(
        "prepareGroupedWrapper", SOURCE / "cmake/prepareGroupedWrapper.py")
    preparer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preparer)
    source = tmp_path / "wrapper.cxx"
    destination = tmp_path / "grouped.cxx"
    definition = "SWIG_GUARD_DEFINITION(Director, swig_mutex_own)" + semicolon
    source.write_text("#define SWIG_DIRECTORS\n  " + definition + "\n", encoding="utf-8")
    preparer.prepare_wrapper(source, destination)
    assert destination.read_text(encoding="utf-8") == (
        "#define SWIG_DIRECTORS\n  #ifdef SWIG_THREADS\n  inline " + definition + "\n  #endif\n")
    for count in (0, 2):
        source.write_text("#define SWIG_DIRECTORS\n" + (definition + "\n") * count, encoding="utf-8")
        with pytest.raises(ValueError, match=f"found {count} definitions"):
            preparer.prepare_wrapper(source, destination)


@pytest.mark.ciSkip
@pytest.mark.buildIntegration
@pytest.mark.skipif(CMAKE is None or SWIG is None, reason="CMake and SWIG are required")
@pytest.mark.parametrize("build_generator,limited_api,threads", DIRECTOR_CONFIGURATIONS)
def test_grouped_swig_directors_preserve_cross_module_callbacks(tmp_path, build_generator, limited_api, threads):
    """Compile real SWIG wrappers and dispatch Python overrides across modules.

    :param tmp_path: Temporary directory supplied by pytest.
    :param build_generator: CMake generator used for the isolated native build.
    :param limited_api: Compile against the stable Python API when true.
    :param threads: Generate SWIG's threaded director runtime when true.
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
    for name in ("generateGroupedBindings.py", "prepareGroupedWrapper.py"):
        shutil.copyfile(SOURCE / "cmake" / name, project / "cmake" / name)
    shutil.copyfile(SOURCE / "fswAlgorithms/_load_fsw.py", project / "fswAlgorithms/_load_fsw.py")
    (project / "base.h").write_text(
        "#pragma once\nclass Base { public: virtual ~Base() = default; virtual int value() { return 2; } };\n",
        encoding="utf-8",
    )
    (project / "first.i").write_text('''%module(package="Basilisk.simulation", directors="1") first
%feature("director") Base;
%{
#include "base.h"
%}
%include "base.h"
''', encoding="utf-8")
    (project / "second.i").write_text('''%module(package="Basilisk.simulation", directors="1") second
%{
#include "base.h"
%}
%import "first.i"
%feature("director") Other;
%inline %{
class Other { public: virtual ~Other() = default; virtual int value() { return 3; } };
int call(Base *object) { return object->value(); }
int callOther(Other *object) { return object->value(); }
bool isDirector(Base *object) { return dynamic_cast<Swig::Director *>(object) != nullptr; }
%}
''', encoding="utf-8")
    helper = SOURCE / "cmake/bskCombineSimulationBindings.cmake"
    (project / "CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.26)
project(directorBuildTest LANGUAGES CXX)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(SWIG 4.4.1 REQUIRED)
include(${{SWIG_USE_FILE}})
set(SWIG_USE_SWIG_DEPENDENCIES ON)
set(CMAKE_CXX_STANDARD 17)
include("{helper.as_posix()}")
if(LIMITED_API)
  add_compile_definitions(Py_LIMITED_API=0x03090000)
endif()
set(out "${{CMAKE_BINARY_DIR}}/Basilisk/simulation")
foreach(name first second)
  set(interface "${{CMAKE_CURRENT_SOURCE_DIR}}/${{name}}.i")
  set_source_files_properties("${{interface}}" PROPERTIES CPLUSPLUS ON USE_TARGET_INCLUDE_DIRECTORIES TRUE)
  set_property(SOURCE "${{interface}}" APPEND PROPERTY SWIG_FLAGS -interface "_${{name}}" {"-threads" if threads else "-nothreads"})
  swig_add_library(${{name}}GroupedObjects LANGUAGE python TYPE OBJECT SOURCES "${{interface}}"
    OUTFILE_DIR "${{out}}" OUTPUT_DIR "${{out}}")
  target_link_libraries(${{name}}GroupedObjects PRIVATE Python3::Module)
  target_include_directories(${{name}}GroupedObjects PRIVATE "${{CMAKE_CURRENT_SOURCE_DIR}}")
  set_target_properties(${{name}}GroupedObjects PROPERTIES POSITION_INDEPENDENT_CODE ON)
  set(wrapper "${{out}}/${{name}}PYTHON_wrap.cxx")
  set(grouped "${{out}}/${{name}}PYTHON_grouped.cxx")
  add_custom_command(OUTPUT "${{grouped}}"
    COMMAND "${{Python3_EXECUTABLE}}" "${{CMAKE_SOURCE_DIR}}/cmake/prepareGroupedWrapper.py" "${{wrapper}}" "${{grouped}}"
    DEPENDS "${{wrapper}}" "${{CMAKE_SOURCE_DIR}}/cmake/prepareGroupedWrapper.py" VERBATIM)
  get_target_property(wrapper_sources ${{name}}GroupedObjects SOURCES)
  list(REMOVE_ITEM wrapper_sources "${{wrapper}}")
  set_property(TARGET ${{name}}GroupedObjects PROPERTY SOURCES ${{wrapper_sources}} "${{grouped}}")
  set_property(GLOBAL APPEND PROPERTY BSK_mujocoNative_OBJECT_TARGETS ${{name}}GroupedObjects)
  set_property(GLOBAL APPEND PROPERTY BSK_mujocoNative_MODULES "simulation.${{name}}")
  set_property(GLOBAL APPEND PROPERTY BSK_SIMULATION_BINDING_TARGETS ${{name}}GroupedObjects)
endforeach()
bsk_finalize_simulation_bindings()
file(WRITE "${{CMAKE_BINARY_DIR}}/Basilisk/__init__.py" "")
file(WRITE "${{out}}/__init__.py" "")
''', encoding="utf-8")
    configure = [CMAKE, "-S", str(project), "-B", str(build),
                 f"-DPython3_EXECUTABLE={sys.executable}", f"-DSWIG_EXECUTABLE={SWIG}",
                 f"-DLIMITED_API={'ON' if limited_api else 'OFF'}", "-DCMAKE_BUILD_TYPE=Release"]
    if build_generator:
        configure.extend(["-G", build_generator])
    if build_generator == "Ninja":
        configure.append(f"-DCMAKE_MAKE_PROGRAM={NINJA}")
    probe = '''import sys
sys.path.insert(0, sys.argv[1])
from Basilisk.simulation import first, second
class First(first.Base):
    def value(self): return 42
class Second(second.Other):
    def value(self): return 17
first_object = First()
assert second.call(first_object) == 42
assert second.callOther(Second()) == 17
assert second.isDirector(first_object)
assert first._first.__file__ == second._second.__file__
'''
    for command in (
        configure,
        [CMAKE, "--build", str(build), "--target", "mujocoNative", "--config", "Release"],
        [sys.executable, "-I", "-c", probe, str(build)],
    ):
        result = subprocess.run(command, capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT_SECONDS)
        assert result.returncode == 0, result.stdout + result.stderr

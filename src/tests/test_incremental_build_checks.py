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

"""Exercise the incremental-build checks with native CMake generators."""

import os
from pathlib import Path
import runpy
import shutil
import subprocess
import sys

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CHECKS = runpy.run_path(str(REPOSITORY_ROOT / ".github/scripts/test_incremental_build.py"))
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


@pytest.mark.ciSkip
@pytest.mark.buildIntegration
@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("generator", BUILD_GENERATORS)
def test_package_configuration_checks_native_targets(tmp_path, monkeypatch, generator):
    """Accept header-only packages and detect accidental binary targets.

    :param tmp_path: Temporary directory supplied by pytest.
    :param monkeypatch: Pytest helper for selecting the CMake executable.
    :param generator: Native CMake generator to exercise.
    """
    if generator == "Xcode":
        xcodebuild = shutil.which("xcodebuild")
        if xcodebuild is None:
            pytest.skip("Xcode is not installed")
        version = subprocess.run(
            [xcodebuild, "-version"], capture_output=True, text=True,
            timeout=SUBPROCESS_TIMEOUT_SECONDS,
        )
        if version.returncode:
            pytest.skip("A full Xcode installation must be selected to test its generator")

    project = tmp_path / "source"
    build = tmp_path / "build"
    (project / "cmake").mkdir(parents=True)
    for helper in ("bskCollectWrapperCustomFiles.cmake", "bskSourceInventory.cmake"):
        shutil.copyfile(REPOSITORY_ROOT / "src/cmake" / helper, project / "cmake" / helper)
    (project / "fixture.c").write_text("int fixture(void) { return 0; }\n", encoding="utf-8")
    declarations = [
        "cmake_minimum_required(VERSION 3.26)",
        "project(packageCheckFixture LANGUAGES C)",
        # A similar name must not be mistaken for a header-only package.
        "add_library(communicationLibHelper STATIC fixture.c)",
    ]
    declarations.append('''foreach(package communicationLib vizardLib)
  if(package STREQUAL BINARY_PACKAGE)
    add_library(${package} STATIC fixture.c)
  else()
    add_library(${package} INTERFACE)
  endif()
endforeach()''')
    (project / "CMakeLists.txt").write_text("\n".join(declarations) + "\n", encoding="utf-8")
    command = [CMAKE, "-S", str(project), "-B", str(build)]
    if generator:
        command.extend(["-G", generator])
    if generator == "Ninja":
        command.append(f"-DCMAKE_MAKE_PROGRAM={NINJA}")
    check = CHECKS["assert_package_configuration"]
    monkeypatch.setitem(check.__globals__, "cmake_executable", lambda: CMAKE)
    # Reuse compiler detection while exercising each actual File API target set.
    for binary_package in ("", "communicationLib", "vizardLib", ""):
        result = subprocess.run(
            command + [f"-DBINARY_PACKAGE={binary_package}"], capture_output=True, text=True,
            timeout=SUBPROCESS_TIMEOUT_SECONDS,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        if binary_package:
            with pytest.raises(CHECKS["IncrementalBuildError"], match=f"binary library targets: {binary_package}"):
                check(project, build)
        else:
            check(project, build)


@pytest.mark.ciSkip
@pytest.mark.buildIntegration
@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("discovery_source", ["PATH", "CMAKE_PROGRAM_PATH"])
def test_protobuf_compiler_follows_build_environment(tmp_path, discovery_source):
    """Replace a cached Protobuf compiler and follow subsequent dependency upgrades.

    :param tmp_path: Temporary directory supplied by pytest.
    :param discovery_source: Environment or toolchain path used to locate the compiler.
    """
    project = tmp_path / "source"
    build = tmp_path / "build"
    project.mkdir()
    # No schemas or native targets are needed to exercise compiler discovery.
    (project / "protobuf-config.cmake").write_text("", encoding="utf-8")
    discovery = REPOSITORY_ROOT / "src/utilities/vizProtobuffer/CMakeLists.txt"
    (project / "CMakeLists.txt").write_text(
        'cmake_minimum_required(VERSION 3.26)\n'
        'project(protobufCompilerFixture LANGUAGES NONE)\n'
        'set(BUILD_VIZINTERFACE ON)\n'
        'set(protobuf_DIR "${CMAKE_CURRENT_SOURCE_DIR}")\n'
        f'include("{discovery.as_posix()}")\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/selected-protoc.txt" "${PROTOC_EXE}")\n',
        encoding="utf-8",
    )
    compilers = []
    for name in ("legacy", "current", "upgraded"):
        directory = tmp_path / name
        directory.mkdir()
        executable = directory / ("protoc.exe" if os.name == "nt" else "protoc")
        executable.touch()
        executable.chmod(0o755)
        compilers.append(executable)

    command = [CMAKE, "-S", str(project), "-B", str(build)]
    if NINJA:
        command.extend(["-G", "Ninja", f"-DCMAKE_MAKE_PROGRAM={NINJA}"])
    for index, compiler in enumerate(compilers[1:]):
        environment = os.environ.copy()
        arguments = command.copy()
        search_path = compiler.parent if discovery_source == "PATH" else compilers[0].parent
        environment["PATH"] = str(search_path) + os.pathsep + environment.get("PATH", "")
        if discovery_source == "CMAKE_PROGRAM_PATH":
            arguments.append(f"-DCMAKE_PROGRAM_PATH={compiler.parent.as_posix()}")
        if index == 0:
            arguments.append(f"-DPROTOC_EXE:FILEPATH={compilers[0].as_posix()}")
        result = subprocess.run(
            arguments, env=environment, capture_output=True, text=True,
            timeout=SUBPROCESS_TIMEOUT_SECONDS,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        selected = (build / "selected-protoc.txt").read_text(encoding="utf-8")
        assert Path(selected).resolve() == compiler.resolve()


@pytest.mark.ciSkip
@pytest.mark.buildIntegration
@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("target_os", ["Macos", "Linux", "Windows"])
def test_mujoco_libraries_follow_dependency_upgrades(tmp_path, target_os):
    """Select each upgraded Conan package while retaining the build and cache.

    :param tmp_path: Temporary directory supplied by pytest.
    :param target_os: Platform whose MuJoCo package metadata is generated.
    """
    pytest.importorskip("conan")
    project = tmp_path / "source"
    build = tmp_path / "build"
    recipe = tmp_path / "recipe"
    generators = tmp_path / "generators"
    project.mkdir()
    recipe.mkdir()
    profile = tmp_path / "profile"
    profile.write_text(
        f"[settings]\nos={target_os}\narch=x86_64\nbuild_type=Release\n",
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["CONAN_HOME"] = str(tmp_path / "conan-home")
    mujoco_recipe = REPOSITORY_ROOT / "libs/mujoco/conanfile.py"
    # Use production package metadata with small stand-ins for the binaries.
    # No downloads or native compilers are needed to test library discovery.
    (recipe / "conanfile.py").write_text(
        f'''import importlib.util
import os
from conan.tools.files import save

spec = importlib.util.spec_from_file_location(
    "mujoco_test_recipe", {str(mujoco_recipe)!r},
)
recipe_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recipe_module)

class MujocoDiscoveryRecipe(recipe_module.MujocoConan):
    def set_version(self):
        pass

    def package(self):
        paths = {{
            "Macos": ["lib/mujoco.framework/mujoco"],
            "Linux": ["lib/libmujoco.so"],
            "Windows": ["lib/mujoco.lib", "bin/mujoco.dll"],
        }}
        for path in paths[str(self.settings.os)]:
            save(self, os.path.join(self.package_folder, path), str(self.version))
''',
        encoding="utf-8",
    )

    def run_conan(arguments):
        """Generate local packages and CMakeDeps files without remote access."""
        result = subprocess.run(
            [sys.executable, "-m", "conans.conan", *arguments,
             "-pr:h", str(profile), "-pr:b", str(profile), "--no-remote"],
            cwd=tmp_path, env=environment, capture_output=True, text=True,
            timeout=SUBPROCESS_TIMEOUT_SECONDS,
        )
        assert result.returncode == 0, result.stdout + result.stderr

    discovery = REPOSITORY_ROOT / "src/cmake/usingMujoco.cmake"
    (project / "CMakeLists.txt").write_text(
        'cmake_minimum_required(VERSION 3.26)\n'
        'project(mujocoDiscoveryFixture LANGUAGES NONE)\n'
        f'include("{discovery.as_posix()}")\n'
        f'include("{discovery.as_posix()}")\n'
        'if(APPLE)\n'
        '  set(selected "${CONAN_FRAMEWORK_mujoco_FOUND}/mujoco")\n'
        'else()\n'
        '  get_target_property(selected CONAN_LIB::mujoco_mujoco_RELEASE\n'
        '    IMPORTED_LOCATION_RELEASE)\n'
        '  if(WIN32)\n'
        '    get_target_property(import_library '
        'CONAN_LIB::mujoco_mujoco_RELEASE\n'
        '      IMPORTED_IMPLIB_RELEASE)\n'
        '    list(APPEND selected "${import_library}")\n'
        '  endif()\n'
        'endif()\n'
        'file(WRITE "${CMAKE_BINARY_DIR}/selected-libraries.txt" '
        '"${selected}")\n',
        encoding="utf-8",
    )
    system = "Darwin" if target_os == "Macos" else target_os
    command = [
        CMAKE, "-S", str(project), "-B", str(build),
        f"-DCMAKE_SYSTEM_NAME={system}", "-DCMAKE_BUILD_TYPE=Release",
        f"-Dmujoco_DIR={generators.as_posix()}",
    ]
    if NINJA:
        command.extend(["-G", "Ninja", f"-DCMAKE_MAKE_PROGRAM={NINJA}"])
    else:
        generator = "NMake Makefiles" if os.name == "nt" else "Unix Makefiles"
        command.extend(["-G", generator])
    retained_libraries = []
    for version in ("1.0", "2.0", "3.0"):
        run_conan(["export-pkg", str(recipe), "--version", version])
        run_conan([
            "install", "--requires", f"mujoco/{version}", "-g", "CMakeDeps",
            "-of", str(generators),
        ])
        result = subprocess.run(
            command, capture_output=True, text=True,
            timeout=SUBPROCESS_TIMEOUT_SECONDS,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        selected = (build / "selected-libraries.txt").read_text(encoding="utf-8")
        libraries = [Path(path) for path in selected.split(";")]
        assert len(libraries) == (2 if target_os == "Windows" else 1)
        for library in libraries:
            assert library.read_text(encoding="utf-8") == version
        # Old packages must remain available throughout every reconfiguration.
        assert all(path.is_file() for path in retained_libraries)
        retained_libraries.extend(libraries)

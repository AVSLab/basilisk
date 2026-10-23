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

"""Exercise build verbosity with real Conan packages, CMake, and compiler diagnostics."""

import argparse
import importlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

import pytest


pytest.importorskip("conan")
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def run_command(command, directory, environment):
    """Capture complete subprocess diagnostics without changing their exit status."""
    return subprocess.run(
        command, cwd=directory, env=environment, capture_output=True, text=True,
        check=False, timeout=120,  # [s]
    )


def test_clean_cached_and_verbose_build_output(tmp_path, monkeypatch):
    """Keep warnings and errors while hiding routine dependency details by default.

    A private cache builds a local fixture dependency once, then reuses it through
    normal, verbose, and normal-again builds. No external packages are downloaded.
    """
    executable_suffix = ".exe" if os.name == "nt" else ""
    cmake = shutil.which("cmake") or str(Path(sys.executable).with_name("cmake" + executable_suffix))
    if not Path(cmake).is_file():
        pytest.skip("CMake is required")
    monkeypatch.syspath_prepend(str(REPOSITORY_ROOT))
    recipe_module = importlib.import_module("conanfile")
    environment = os.environ.copy()
    environment["CONAN_HOME"] = str(tmp_path / "conan-home")
    environment["PATH"] = os.pathsep.join([str(Path(cmake).parent), environment.get("PATH", "")])
    environment.pop("CONAN_LOG_LEVEL", None)
    environment.pop("CMAKE_MESSAGE_LOG_LEVEL", None)
    conan = [sys.executable, "-m", "conans.conan"]
    profile = run_command(conan + ["profile", "detect"], tmp_path, environment)
    assert profile.returncode == 0, profile.stdout + profile.stderr

    dependency = tmp_path / "dependency"
    dependency.mkdir()
    (dependency / "conanfile.py").write_text('''from conan import ConanFile
from conan.tools.files import save
import os

class OutputDependency(ConanFile):
    name = "eigen"
    version = "5.0.1"
    package_type = "header-library"

    def package(self):
        save(self, os.path.join(self.package_folder, "include", "fixture.h"), "// Fixture header\\n")

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "Eigen3")
        self.output.warning("BSK Conan deprecation sentinel", warn_tag="deprecated")
''', encoding="utf-8")
    exported = run_command(conan + ["export", str(dependency)], tmp_path, environment)
    assert exported.returncode == 0, exported.stdout + exported.stderr

    project = tmp_path / "project"
    source = project / "src"
    source.mkdir(parents=True)
    (project / "conanfile.py").write_text(f'''import importlib.util
spec = importlib.util.spec_from_file_location("bsk_recipe", {str(REPOSITORY_ROOT / 'conanfile.py')!r})
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class OutputRecipe(module.BasiliskConan):
    requires = ()

    def requirements(self):
        self.requires("eigen/5.0.1")

    def build_requirements(self):
        pass

    def configure(self):
        pass
''', encoding="utf-8")
    # Exercise the production dependency declaration with actual CMakeDeps output.
    dependency_call = re.search(
        r"^find_package\(Eigen3[^\n]+", (REPOSITORY_ROOT / "src/CMakeLists.txt").read_text(), re.M,
    ).group()
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.26)\n"
        "project(buildOutputTest LANGUAGES CXX)\n"
        f"{dependency_call}\n"
        'message(STATUS "BSK CMake status sentinel")\n'
        'message(VERBOSE "BSK CMake verbose sentinel")\n'
        'message(NOTICE "BSK CMake notice sentinel")\n'
        'message(WARNING "BSK CMake warning sentinel")\n'
        'message(DEPRECATION "BSK CMake deprecation sentinel")\n'
        "add_library(outputTest STATIC example.cpp)\n"
        # CMake does not enable MSVC's level-3 deprecation warning by default.
        "target_compile_options(outputTest PRIVATE\n"
        '  "$<$<CXX_COMPILER_ID:MSVC>:/w14996>"\n'
        '  "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wdeprecated-declarations>")\n',
        encoding="utf-8",
    )
    (source / "example.cpp").write_text(
        '[[deprecated("BSK compiler deprecation sentinel")]] int old_api() { return 1; }\n'
        "int use_api() { return old_api(); }\n",
        encoding="utf-8",
    )
    values = {"buildType": "Release", "generator": None, "offline": False}
    for group in (recipe_module.bskModuleOptionsBool, recipe_module.bskModuleOptionsString,
                  recipe_module.bskModuleOptionsFlag):
        values.update({name: definition[1] for name, definition in group.items()})
    values.update(vizInterface=False, buildTesting=False)

    for index, verbose in enumerate([False, False, True, False]):
        arguments = argparse.Namespace(**values, verbose=verbose)
        command = recipe_module.create_conan_build_command(arguments) + ["--no-remote"]
        process = run_command(command, project, environment)
        output = process.stdout + process.stderr
        assert process.returncode == 0, output
        assert "BSK Conan deprecation sentinel" in output
        assert "BSK CMake notice sentinel" in output
        assert "BSK CMake warning sentinel" in output
        assert "BSK CMake deprecation sentinel" in output
        assert ("BSK CMake status sentinel" in output) == verbose
        assert ("BSK CMake verbose sentinel" in output) == verbose
        assert ("Profile host:" in output) == verbose
        assert ("Conan: Target declared" in output) == verbose
        if index == 0:
            assert "Building from source" in output
            assert "BSK compiler deprecation sentinel" in output
        else:
            assert "Already installed!" in output

    with (source / "CMakeLists.txt").open("a", encoding="utf-8") as cmake_file:
        cmake_file.write("find_package(BskMissingDependency CONFIG REQUIRED QUIET)\n")
    failed = run_command(command, project, environment)
    assert failed.returncode != 0
    assert "BskMissingDependency" in failed.stdout + failed.stderr
    assert "CMake Error" in failed.stdout + failed.stderr

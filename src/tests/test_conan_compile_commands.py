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

"""Exercise compilation database export through the real Conan and CMake tools."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


pytest.importorskip("conan")

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def _find_tool(name):
    """Find a build tool on PATH or beside the active Python interpreter."""
    executable = name + (".exe" if os.name == "nt" else "")
    environment_tool = Path(sys.executable).with_name(executable)
    return shutil.which(executable) or (
        str(environment_tool) if environment_tool.is_file() else None
    )


CMAKE = _find_tool("cmake")
NINJA = _find_tool("ninja")
GENERATORS = ["Ninja"] if NINJA else []
if os.name != "nt":
    GENERATORS.append("Unix Makefiles")
elif not NINJA:
    GENERATORS.append("NMake Makefiles")


def _run_conan(arguments, environment, directory):
    """Run Conan and report its captured diagnostics if the command fails."""
    result = subprocess.run(
        [sys.executable, "-m", "conans.conan", *arguments],
        cwd=directory,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
        timeout=120,  # [s]
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.fixture(scope="module")
def conan_environment(tmp_path_factory):
    """Detect a native compiler using a temporary Conan cache and profile."""
    if CMAKE is None:
        pytest.skip("CMake is required")
    directory = tmp_path_factory.mktemp("compile-commands-conan")
    environment = os.environ.copy()
    environment["CONAN_HOME"] = str(directory / "conan-home")
    tool_directories = [str(Path(tool).parent) for tool in (CMAKE, NINJA) if tool]
    environment["PATH"] = os.pathsep.join([
        *tool_directories, environment.get("PATH", ""),
    ])
    # A developer's CMake default must not mask a missing recipe setting.
    environment.pop("CMAKE_EXPORT_COMPILE_COMMANDS", None)
    _run_conan(["profile", "detect"], environment, directory)
    return environment


@pytest.fixture
def compile_commands_project(tmp_path):
    """Use the production recipe with a small project and no package dependencies."""
    project = tmp_path / "project with spaces"
    source = project / "src"
    source.mkdir(parents=True)
    (project / "conanfile.py").write_text(
        f"""import importlib.util

spec = importlib.util.spec_from_file_location(
    "basilisk_test_recipe", {str(REPOSITORY_ROOT / 'conanfile.py')!r},
)
recipe_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recipe_module)

class CompileCommandsRecipe(recipe_module.BasiliskConan):
    requires = ()

    def requirements(self):
        pass

    def build_requirements(self):
        pass
""",
        encoding="utf-8",
    )
    (source / "CMakeLists.txt").write_text(
        """cmake_minimum_required(VERSION 3.26)
project(compileCommandsTest LANGUAGES C CXX)
add_library(compileCommandsTest STATIC example.c example.cpp)
target_compile_definitions(compileCommandsTest PRIVATE COMPILE_COMMANDS_TEST=1)
""",
        encoding="utf-8",
    )
    for name in ("example.c", "example.cpp"):
        (source / name).write_text("int example(void) { return 0; }\n", encoding="utf-8")
    return project


@pytest.mark.parametrize("generator", GENERATORS)
@pytest.mark.parametrize("export_option", [None, False], ids=["default", "disabled"])
def test_compile_commands_export(
        conan_environment, compile_commands_project, generator, export_option,
):
    """Export C and C++ commands by default and omit the database when disabled."""
    project = compile_commands_project
    arguments = [
        "build", ".", "--no-remote", "--build=never",
        "-o", f"&:generator={generator}",
        "-o", "&:buildProject=False",
        "-o", "&:vizInterface=False",
    ]
    if export_option is not None:
        arguments.extend(["-o", f"&:exportCompileCommands={export_option}"])

    _run_conan(arguments, conan_environment, project)

    database = project / "dist3" / "compile_commands.json"
    if export_option is False:
        assert not database.exists()
        return

    commands = json.loads(database.read_text(encoding="utf-8"))
    assert len(commands) == 2
    assert {
        (Path(entry["directory"]) / entry["file"]).resolve()
        for entry in commands
    } == {project / "src" / name for name in ("example.c", "example.cpp")}
    for entry in commands:
        assert Path(entry["directory"]).is_dir()
        assert "COMPILE_COMMANDS_TEST=1" in entry["command"]

    # Exercise cleanup and re-enabling in the same configured build directory.
    _run_conan(
        [*arguments, "-o", "&:exportCompileCommands=False"],
        conan_environment,
        project,
    )
    assert not database.exists()
    _run_conan(arguments, conan_environment, project)
    assert json.loads(database.read_text(encoding="utf-8")) == commands

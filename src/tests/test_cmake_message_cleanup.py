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

"""Exercise generated C-message retirement through the real CMake generator."""

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
ENVIRONMENT_CMAKE = Path(sys.executable).with_name("cmake")
CMAKE = shutil.which("cmake") or (
    str(ENVIRONMENT_CMAKE) if ENVIRONMENT_CMAKE.is_file() else None
)
ENVIRONMENT_NINJA = Path(sys.executable).with_name("ninja")
NINJA = shutil.which("ninja") or (
    str(ENVIRONMENT_NINJA) if ENVIRONMENT_NINJA.is_file() else None
)
GENERATORS = [None] if os.name == "nt" else ["Unix Makefiles"]
if NINJA and os.name != "nt":
    GENERATORS.append("Ninja")


def _run(command):
    """Run a fixture command and expose its full diagnostics on failure.

    :param command: Executable and command-line arguments.
    """
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.skipif(CMAKE is None, reason="CMake is required")
@pytest.mark.parametrize("change", ["delete", "rename", "move_to_cpp"])
@pytest.mark.parametrize("generator", GENERATORS)
def test_removed_c_message_interfaces_are_retired_during_configuration(tmp_path, change, generator):
    """Remove obsolete C interfaces before any build-time Rust consumer runs.

    :param tmp_path: Temporary directory supplied by pytest.
    :param change: Payload inventory change applied after initial generation.
    :param generator: Native build generator to exercise.
    """
    source = REPOSITORY_ROOT / "src"
    project = tmp_path / "project"
    external = tmp_path / "External"
    c_payloads = external / "msgPayloadDefC"
    cpp_payloads = external / "msgPayloadDefCpp"
    project.mkdir()
    c_payloads.mkdir(parents=True)
    cpp_payloads.mkdir()
    (tmp_path / "LICENSE").write_text("Test fixture license.\n", encoding="utf-8")
    removed = c_payloads / "RemovedMsgPayload.h"
    removed.write_text("typedef struct { double value; } RemovedMsgPayload;\n", encoding="utf-8")
    retained = c_payloads / "RetainedMsgPayload.h"
    retained.write_text("typedef struct { double value; } RetainedMsgPayload;\n", encoding="utf-8")
    (project / "CMakeLists.txt").write_text(
        f"""cmake_minimum_required(VERSION 3.26)
project(messageCleanup CXX)
include("{source.as_posix()}/cmake/bskSourceInventory.cmake")
set(EXTERNAL_MODULES_PATH "{external.as_posix()}")
bsk_collect_source_inventory(BSK_HEADER_FILES "${{EXTERNAL_MODULES_PATH}}")
# Include one built-in payload as well as the external payloads.
list(APPEND BSK_HEADER_FILES "{source.as_posix()}/architecture/msgPayloadDefC/AttRefMsgPayload.h")
add_subdirectory("{source.as_posix()}/architecture/messaging/cMsgCInterface" c-messages EXCLUDE_FROM_ALL)
""",
        encoding="utf-8",
    )
    build = tmp_path / "build"

    def configure():
        """Run the production C-message CMakeLists against the fixture inventory."""
        command = [CMAKE, "-S", str(project), "-B", str(build)]
        if generator:
            command.extend(["-G", generator])
        if generator == "Ninja":
            command.append(f"-DCMAKE_MAKE_PROGRAM={NINJA}")
        if sys.platform == "darwin" and not os.environ.get("SDKROOT"):
            # Resolve the SDK through the selected developer tools, as the main
            # build does. The compiler's implicit SDK may point to another CLT
            # installation after an update. Preserve an explicit SDKROOT.
            command.append("-DCMAKE_OSX_SYSROOT=macosx")
        _run(command)

    configure()
    interfaces = build / "autoSource/cMsgCInterface"
    assert (interfaces / "RemovedMsg_C.h").is_file()
    assert (interfaces / "RemovedMsg_C.cpp").is_file()
    retained_paths = [
        interfaces / f"{name}_C.{suffix}"
        for name in ("AttRefMsg", "RetainedMsg")
        for suffix in ("h", "cpp")
    ]
    retained_times = {path: path.stat().st_mtime_ns for path in retained_paths}
    unrelated = interfaces / "User_C.h"
    unrelated.write_text("Keep this file.\n", encoding="utf-8")

    if change == "rename":
        renamed = removed.rename(c_payloads / "RenamedMsgPayload.h")
        renamed.write_text("typedef struct { double value; } RenamedMsgPayload;\n", encoding="utf-8")
    elif change == "move_to_cpp":
        removed.rename(cpp_payloads / removed.name)
    else:
        removed.unlink()

    # A normal native build must discover the inventory change and clean up
    # during regeneration, without an explicit configure or Python-wrapper build.
    _run([CMAKE, "--build", str(build)])
    assert not (interfaces / "RemovedMsg_C.h").exists()
    assert not (interfaces / "RemovedMsg_C.cpp").exists()
    if change == "rename":
        assert (interfaces / "RenamedMsg_C.h").is_file()
        assert (interfaces / "RenamedMsg_C.cpp").is_file()
    assert {path: path.stat().st_mtime_ns for path in retained_paths} == retained_times
    assert unrelated.read_text(encoding="utf-8") == "Keep this file.\n"

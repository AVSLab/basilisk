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

"""Regression tests for draft-module paths and preservation of existing work."""

import ast
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest


GENERATOR_PATH = Path(__file__).resolve().parents[1] / "makeDraftModule.py"


@pytest.fixture(params=["C", "C++"])
def draft(request, tmp_path, monkeypatch):
    """Configure a real generator in an isolated source tree for either language."""
    spec = importlib.util.spec_from_file_location("draft_module_test", GENERATOR_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source_path = tmp_path / "repository with spaces" / "src"
    (source_path / "moduleTemplates").mkdir(parents=True)
    shutil.copyfile(GENERATOR_PATH.parents[2] / "LICENSE", source_path.parent / "LICENSE")
    monkeypatch.setattr(module, "pathToSrc", str(source_path))

    generator = module.moduleGenerator()
    if request.param == "C":
        module.fillCInfo(generator)
        create = generator.createCModule
        extension = ".c"
    else:
        module.fillCppInfo(generator)
        create = generator.createCppModule
        extension = ".cpp"
    generator.verbose = False
    generator.moduleName = "draftExample"
    generator.briefDescription = "A draft with a Unicode description: café."
    caller = tmp_path / "caller directory"
    caller.mkdir()
    # Import happened above, before entering this caller's directory.
    monkeypatch.chdir(caller)
    return SimpleNamespace(
        module=module,
        generator=generator,
        create=create,
        extension=extension,
        source_path=source_path,
        destination=source_path / "moduleTemplates" / generator.moduleName,
        caller=caller,
    )


def _existing_work(draft):
    """Create an existing module containing work that must survive failures."""
    draft.destination.mkdir()
    marker = draft.destination / "existing-work.txt"
    marker.write_bytes(b"Keep this module work.\n")
    return marker


@pytest.mark.parametrize("relative_path", [
    "moduleTemplates",
    "moduleTemplates/",
    "fswAlgorithms/attControl",
    "fswAlgorithms/attControl/",
    os.path.join("fswAlgorithms", "attControl", "nested", ""),
    Path("fswAlgorithms") / "attControl" / "nested",
])
def test_destination_paths_generate_valid_imports(draft, relative_path):
    """Normalize native paths and trailing separators without changing the caller."""
    draft.generator.modulePathRelSrc = relative_path
    parent = draft.source_path / relative_path
    parent.mkdir(parents=True, exist_ok=True)

    draft.create()

    name = draft.generator.moduleName
    destination = parent / name
    files = {path.relative_to(destination).as_posix() for path in destination.rglob("*") if path.is_file()}
    assert files == {
        f"{name}.h", f"{name}{draft.extension}", f"{name}.i", f"{name}.rst",
        f"_UnitTest/test_{name}.py",
    }
    test_source = (destination / "_UnitTest" / f"test_{name}.py").read_text(encoding="utf-8")
    tree = ast.parse(test_source)
    module_import = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and any(alias.name == name for alias in node.names)
    )
    assert module_import.module == f"Basilisk.{Path(relative_path).parts[0]}"
    implementation = (destination / f"{name}{draft.extension}").read_text(encoding="utf-8")
    assert f'#include "{Path(relative_path).as_posix()}/{name}/{name}.h"' in implementation
    assert "café" in (destination / f"{name}.rst").read_text(encoding="utf-8")
    assert Path.cwd() == draft.caller
    assert list(parent.iterdir()) == [destination]


@pytest.mark.parametrize("automatic", [False, True])
def test_existing_module_is_replaced_only_after_generation(draft, monkeypatch, automatic):
    """Keep existing files throughout rendering, then publish the complete draft."""
    marker = _existing_work(draft)
    draft.generator.cleanBuild = automatic
    monkeypatch.setattr("builtins.input", lambda prompt: "y")
    original_create_test = draft.generator.createTestFile

    def check_existing_work(module_type):
        """Inspect the destination during the final generation step."""
        assert marker.read_bytes() == b"Keep this module work.\n"
        assert Path.cwd() == draft.caller
        original_create_test(module_type)

    monkeypatch.setattr(draft.generator, "createTestFile", check_existing_work)
    draft.create()

    assert not marker.exists()
    assert len([path for path in draft.destination.rglob("*") if path.is_file()]) == 5
    assert list(draft.destination.parent.iterdir()) == [draft.destination]
    assert Path.cwd() == draft.caller


def test_declining_replacement_preserves_existing_module(draft, monkeypatch):
    """Reject an overwrite without deleting files or changing directories."""
    marker = _existing_work(draft)
    monkeypatch.setattr("builtins.input", lambda prompt: "n")

    with pytest.raises(FileExistsError, match="cancelled"):
        draft.create()

    assert marker.read_bytes() == b"Keep this module work.\n"
    assert list(draft.destination.parent.iterdir()) == [draft.destination]
    assert Path.cwd() == draft.caller


@pytest.mark.parametrize("invalid_input", ["wrapper", "missing_description", "duplicate", "empty_type"])
def test_invalid_specification_preserves_existing_module(draft, invalid_input):
    """Validate message and variable metadata before replacing existing work."""
    marker = _existing_work(draft)
    draft.generator.cleanBuild = True
    if invalid_input == "wrapper":
        draft.generator.inMsgList[0]["wrap"] = "unsupported"
    elif invalid_input == "missing_description":
        del draft.generator.inMsgList[0]["desc"]
    elif invalid_input == "duplicate":
        draft.generator.variableList[0]["var"] = draft.generator.inMsgList[0]["var"]
    else:
        draft.generator.variableList[0]["type"] = ""

    with pytest.raises(ValueError):
        draft.create()

    assert marker.read_bytes() == b"Keep this module work.\n"
    assert list(draft.destination.parent.iterdir()) == [draft.destination]
    assert Path.cwd() == draft.caller


def test_c_module_rejects_cpp_message_before_overwrite(draft):
    """Reject a C++ message in a C specification before deleting the destination."""
    marker = _existing_work(draft)
    draft.generator.cleanBuild = True
    draft.generator.inMsgList[0]["wrap"] = "C++"

    with pytest.raises(ValueError, match="C modules require message wrappers"):
        draft.generator.createCModule()

    assert marker.read_bytes() == b"Keep this module work.\n"
    assert list(draft.destination.parent.iterdir()) == [draft.destination]
    assert Path.cwd() == draft.caller


@pytest.mark.parametrize("already_exists", [False, True])
def test_rendering_failure_preserves_destination(draft, monkeypatch, already_exists):
    """Discard partial draft files while preserving the original destination."""
    marker = _existing_work(draft) if already_exists else None
    draft.generator.cleanBuild = True

    def fail_test_file(module_type):
        """Simulate a write failure after earlier draft files were generated."""
        raise OSError("test file write failed")

    monkeypatch.setattr(draft.generator, "createTestFile", fail_test_file)
    with pytest.raises(OSError, match="test file write failed"):
        draft.create()

    if marker is not None:
        assert marker.read_bytes() == b"Keep this module work.\n"
        assert list(draft.destination.iterdir()) == [marker]
    assert list(draft.destination.parent.iterdir()) == ([draft.destination] if already_exists else [])
    assert Path.cwd() == draft.caller


def test_failed_installation_restores_existing_module(draft, monkeypatch):
    """Restore the original module when moving the completed draft fails."""
    marker = _existing_work(draft)
    draft.generator.cleanBuild = True
    original_rename = Path.rename

    def fail_installation(path, target):
        """Fail only the move from staging into the final destination."""
        if path == draft.generator._output_path:
            raise OSError("installation failed")
        return original_rename(path, target)

    monkeypatch.setattr(Path, "rename", fail_installation)
    with pytest.raises(OSError, match="installation failed"):
        draft.create()

    assert marker.read_bytes() == b"Keep this module work.\n"
    assert list(draft.destination.iterdir()) == [marker]
    assert list(draft.destination.parent.iterdir()) == [draft.destination]
    assert Path.cwd() == draft.caller


def test_failed_restoration_retains_backup(draft, monkeypatch):
    """Keep the recoverable original files even if rollback itself fails."""
    _existing_work(draft)
    draft.generator.cleanBuild = True
    original_rename = Path.rename

    def fail_installation_and_restoration(path, target):
        """Allow the backup move but reject both moves into the destination."""
        if target == draft.destination:
            raise OSError("destination unavailable")
        return original_rename(path, target)

    monkeypatch.setattr(Path, "rename", fail_installation_and_restoration)
    with pytest.raises(OSError, match="original module; its files remain at") as error:
        draft.create()

    backups = list(draft.destination.parent.iterdir())
    assert len(backups) == 1
    backup = backups[0] / draft.generator.moduleName
    assert (backup / "existing-work.txt").read_bytes() == b"Keep this module work.\n"
    assert str(backup) in str(error.value)
    assert Path.cwd() == draft.caller


@pytest.mark.parametrize("link_parent", [False, True])
def test_symbolic_links_cannot_replace_outside_work(draft, link_parent):
    """Reject destination links and parent links that escape the source tree."""
    outside = draft.caller / "outside"
    outside.mkdir()
    marker = outside / "existing-work.txt"
    marker.write_bytes(b"Keep outside work.\n")
    link = draft.source_path / "linkedPackage" if link_parent else draft.destination
    try:
        link.symlink_to(outside, target_is_directory=True)
    except OSError:
        pytest.skip("Creating directory symlinks requires platform permission")
    if link_parent:
        draft.generator.modulePathRelSrc = "linkedPackage"
    draft.generator.cleanBuild = True

    with pytest.raises(ValueError):
        draft.create()

    assert marker.read_bytes() == b"Keep outside work.\n"
    assert link.is_symlink()
    assert Path.cwd() == draft.caller


def test_reuses_generator_for_both_languages(draft):
    """Retain the sequential C++/C generation API used by the Conan build."""
    for configure, create in (
        (draft.module.fillCppInfo, draft.generator.createCppModule),
        (draft.module.fillCInfo, draft.generator.createCModule),
    ):
        configure(draft.generator)
        create()
        name = draft.generator.moduleName
        destination = draft.source_path / "moduleTemplates" / name
        assert (destination / "_UnitTest" / f"test_{name}.py").is_file()
        assert Path.cwd() == draft.caller


@pytest.mark.parametrize("relative_path", ["", ".", "../outside", "moduleTemplates/../outside", "absolute"])
def test_rejects_paths_outside_source_packages(draft, relative_path):
    """Reject ambiguous or escaping paths before any destination is touched."""
    marker = _existing_work(draft)
    draft.generator.cleanBuild = True
    draft.generator.modulePathRelSrc = (
        str(draft.source_path / "moduleTemplates") if relative_path == "absolute" else relative_path
    )

    with pytest.raises(ValueError, match="modulePathRelSrc"):
        draft.create()

    assert marker.read_bytes() == b"Keep this module work.\n"
    assert Path.cwd() == draft.caller


@pytest.mark.parametrize("module_name", ["../draftExample", "bad-name", "for"])
def test_rejects_invalid_module_names(draft, module_name):
    """Reject module names that could escape the destination or break imports."""
    draft.generator.moduleName = module_name
    with pytest.raises(ValueError, match="moduleName"):
        draft.create()
    assert not list(draft.destination.parent.iterdir())
    assert Path.cwd() == draft.caller


def test_missing_parent_raises_error(draft):
    """Report an invalid destination as an exception rather than a successful exit."""
    draft.generator.modulePathRelSrc = "missingPackage"
    with pytest.raises(NotADirectoryError):
        draft.create()
    assert Path.cwd() == draft.caller


def test_invalid_specification_exits_unsuccessfully(draft):
    """An unhandled generator error must produce a nonzero process status."""
    script = (
        "import runpy, sys\n"
        "module = runpy.run_path(sys.argv[1])\n"
        "generator = module['moduleGenerator']()\n"
        "generator.verbose = False\n"
        "generator.createCModule()\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", script, str(GENERATOR_PATH)],
        cwd=draft.caller, capture_output=True, text=True, check=False,
    )
    assert result.returncode != 0
    assert "ValueError: moduleName must be a nonempty string" in result.stderr

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

"""Check canonical module navigation without importing the full Sphinx configuration."""

import ast
from glob import glob
import os
from pathlib import Path
import sys

import numpy as np
import pytest


@pytest.fixture
def crawler(tmp_path, monkeypatch):
    """Load only the crawler class, avoiding ``conf.py`` build-time side effects."""
    conf = Path(__file__).resolve().parents[2] / "docs/source/conf.py"
    tree = ast.parse(conf.read_text(encoding="utf8"), filename=str(conf))
    crawler_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "fileCrawler"
    )
    namespace = {
        "os": os, "sys": sys, "np": np, "glob": glob, "Path": Path,
        "officialSrc": str(tmp_path / "src"),
    }
    module = ast.Module(body=[crawler_class], type_ignores=[])
    exec(compile(module, str(conf), "exec"), namespace)
    monkeypatch.setattr(sys, "path", sys.path.copy())
    return namespace["fileCrawler"](newFiles=True, documentation_root=tmp_path / "docs")


def make_module(tmp_path, language="C", category="fswAlgorithms/attControl", tests=2):
    """Create a minimal module source tree, with independently documented tests."""
    source = tmp_path / "src" / category / "demo"
    source.mkdir(parents=True)
    (source / "demo.rst").write_text(
        ".. note::\n\n   An introductory note.\n\n"
        "Executive Summary\n-----------------\nA module.\n"
    )
    extensions = {"C": (".i", ".c", ".h"), "C++": (".i", ".cpp", ".h"),
                  "Python": (".py",), "Rust": (".rs",)}
    for extension in extensions[language]:
        (source / ("demo" + extension)).write_text("")
    if language == "Rust":
        (source / "Cargo.toml").write_text('[package]\nname = "demo"\n')
    if tests:
        test_root = source / "_UnitTest"
        test_root.mkdir()
        for index in range(tests):
            (test_root / f"test_demo_{index}.py").write_text("")
    return source


@pytest.mark.parametrize("language", ["C", "C++", "Python", "Rust"])
@pytest.mark.parametrize(
    "category", [
        "fswAlgorithms/attControl", "fswAlgorithms/attDetermination",
        "fswAlgorithms/attGuidance", "simulation/sensors", "moduleTemplates",
    ]
)
def test_module_navigation_parent(crawler, tmp_path, language, category):
    """Go straight to the module and retain all tests behind a single index link."""
    source = make_module(tmp_path, language, category)
    crawler.run(str(source.parent) + os.sep)
    generated = tmp_path / "docs" / category
    category = (generated / "index.rst").read_text()
    module = (generated / "demo/demo.rst").read_text()
    legacy = (generated / "demo/index.rst").read_text()
    tests = (generated / "demo/_UnitTest/index.rst").read_text()

    assert "demo <demo/demo>" in category
    assert "demo/index" not in category
    assert ":caption: Modules:" in category
    assert module.count(":doc:`Unit tests <_UnitTest/index>`") == 1
    assert ":class: bsk-module-auxiliary" in module
    assert ".. sidebar:: Auxiliary Files" in module
    assert module.index("An introductory note.") < module.index(".. sidebar::")
    assert module.index(".. sidebar::") < module.index("Executive Summary")
    assert "   Unit tests <_UnitTest/index>" in module
    assert "test_demo_0" not in module and "test_demo_1" not in module
    assert "test_demo_0" in tests and "test_demo_1" in tests
    assert ".. _demo:" in module
    assert legacy.startswith(":orphan:\n")
    assert ".. _Folder_demo:" in legacy
    assert ":doc:`Module documentation <demo>`" in legacy
    assert ".. toctree::" not in legacy


@pytest.mark.parametrize(
    "extra", ["_doc.rst", "_default.rst", "overview.rst"]
)
def test_custom_or_multiple_file_folders_keep_their_index(crawler, tmp_path, extra):
    """Never bypass an authored overview or an additional documentation file."""
    source = make_module(tmp_path)
    (source / extra).write_text("Additional content\n")
    assert crawler._directModuleName(source) is None
    crawler.run(str(source.parent) + os.sep)
    category = (tmp_path / "docs/fswAlgorithms/attControl/index.rst").read_text()
    assert "   demo/index" in category
    assert "demo <demo/demo>" not in category


def test_other_categories_and_nested_directories_are_unchanged(crawler, tmp_path):
    """Keep support libraries unchanged and retain links to helper directories."""
    outside = make_module(tmp_path, category="architecture/utilities")
    assert crawler._directModuleName(outside) is None
    source = make_module(tmp_path)
    (source / "helpers").mkdir()
    assert crawler._directModuleName(source) == "demo"
    assert ("helpers", "helpers/index") in crawler._moduleAuxiliaryPages(source)


@pytest.mark.parametrize("folder_name", ["Demo", "legacyFolder"])
def test_module_name_differs_from_folder(crawler, tmp_path, folder_name):
    """Resolve the sole C module even when its directory has a legacy name."""
    source = make_module(tmp_path, category="fswAlgorithms/attDetermination")
    source = source.rename(source.with_name(folder_name))
    assert crawler._directModuleName(source) == "demo"
    crawler.run(str(source.parent) + os.sep)
    generated = tmp_path / "docs/fswAlgorithms/attDetermination"
    assert f"demo <{folder_name}/demo>" in (generated / "index.rst").read_text()
    assert ":class: bsk-module-auxiliary" in (generated / folder_name / "demo.rst").read_text()
    legacy = (generated / folder_name / "index.rst").read_text()
    assert f".. _Folder_{folder_name}:" in legacy
    assert ":doc:`Module documentation <demo>`" in legacy


def test_no_test_link_without_test_directory(crawler, tmp_path):
    """A module without generated test documentation has no dangling test link."""
    source = make_module(tmp_path, tests=0)
    crawler.run(str(source.parent) + os.sep)
    module = (tmp_path / "docs/fswAlgorithms/attControl/demo/demo.rst").read_text()
    assert "Unit tests" not in module
    assert "_UnitTest" not in module
    assert "Auxiliary Files" not in module


@pytest.mark.parametrize("with_helper", [False, True])
def test_multiple_modules_share_test_index(crawler, tmp_path, with_helper):
    """List grouped modules separately while keeping one parent for shared tests."""
    source = make_module(tmp_path, category="fswAlgorithms/effectorInterfaces")
    for extension in (".c", ".h", ".i"):
        (source / ("secondModule" + extension)).write_text("")
    (source / "secondModule.rst").write_text(
        "Executive Summary\n-----------------\nAnother module.\n"
    )
    if with_helper:
        (source / "helper.h").write_text("")
    crawler.run(str(source.parent) + os.sep)
    generated = tmp_path / "docs/fswAlgorithms/effectorInterfaces"
    category = (generated / "index.rst").read_text()
    group = (generated / "demo/index.rst").read_text()
    assert "   demo/index" in category
    assert ":caption: Modules:" in group
    assert "   demo <demo>" in group
    assert "   secondModule <secondModule>" in group
    assert group.count("   Unit tests <_UnitTest/index>") == 1
    assert (":caption: Files:" in group) == with_helper
    if with_helper:
        assert "   helper\n" in group
        assert "bsk-module-auxiliary" not in (generated / "demo/helper.rst").read_text()
    for name in ("demo", "secondModule"):
        contents = (generated / "demo" / (name + ".rst")).read_text()
        assert ":class: bsk-module-auxiliary" in contents
        assert ":doc:`Unit tests <_UnitTest/index>`" in contents
        assert "Shared by the modules in this folder" in contents
        assert ".. toctree::" not in contents
    test_index = (generated / "demo/_UnitTest/index.rst").read_text()
    assert "test_demo_0" in test_index and "test_demo_1" in test_index


@pytest.mark.parametrize("with_tests", [False, True])
def test_auxiliary_files_and_folders(crawler, tmp_path, with_tests):
    """Keep helper files and subdirectories reachable from a direct module page."""
    source = make_module(tmp_path, tests=2 if with_tests else 0)
    for name in ("parseLookup.py", "helper.h", "helper.cpp"):
        (source / name).write_text("")
    (source / "tools").mkdir()
    (source / "tools/convert.py").write_text("")
    # Internal documentation assets and Cargo artifacts are not public pages.
    for name in ("_Documentation", "__pycache__"):
        (source / name).mkdir()
    crawler.run(str(source.parent) + os.sep)
    generated = tmp_path / "docs/fswAlgorithms/attControl"
    assert "demo <demo/demo>" in (generated / "index.rst").read_text()
    module = (generated / "demo/demo.rst").read_text()
    assert ".. sidebar:: Auxiliary Files" in module
    for label, target in (
        ("parseLookup", "parseLookup"), ("helper", "helper"), ("tools", "tools/index")
    ):
        assert module.count(f":doc:`{label} <{target}>`") == 1
        assert module.count(f"   {label} <{target}>\n") == 1
        assert (generated / "demo" / (target + ".rst")).is_file()
    assert ("Unit tests" in module) == with_tests
    assert "_Documentation" not in module
    assert "__pycache__" not in module
    assert ".. toctree::" not in (generated / "demo/index.rst").read_text()


def test_navigation_sorts_displayed_module_names(crawler, tmp_path):
    """Sort visible module names, not their sometimes unrelated directory names."""
    category = tmp_path / "src/simulation/dynamics"
    for folder, name in (("ZuluFolder", "alpha"), ("Beta", "Beta"), ("gamma", "gamma")):
        source = category / folder
        source.mkdir(parents=True)
        for extension in (".h", ".c", ".i"):
            (source / (name + extension)).write_text("")
        (source / (name + ".rst")).write_text("Executive Summary\n-----------------\n")
    (category / "_GeneralModuleFiles").mkdir()
    crawler.run(str(category) + os.sep)
    contents = (tmp_path / "docs/simulation/dynamics/index.rst").read_text()
    entries = ["   _GeneralModuleFiles/index", "   alpha <", "   Beta <", "   gamma <"]
    positions = [contents.index(entry) for entry in entries]
    assert positions == sorted(positions)


def test_grouped_module_and_auxiliary_sorting(crawler, tmp_path):
    """Apply the same order to grouped modules, helper files, and helper folders."""
    source = make_module(tmp_path)
    for name in ("alpha", "Beta"):
        for extension in (".h", ".c", ".i"):
            (source / (name + extension)).write_text("")
        (source / (name + ".rst")).write_text("Executive Summary\n-----------------\n")
    for name in ("zeta", "Apple", "banana", "_helpers"):
        (source / (name + ".h")).write_text("")
    (source / "cherry").mkdir()
    crawler.run(str(source.parent) + os.sep)
    group = (tmp_path / "docs/fswAlgorithms/attControl/demo/index.rst").read_text()
    for entries in (
        ["   alpha <", "   Beta <", "   demo <"],
        ["   _helpers\n", "   Apple\n", "   banana\n", "   zeta\n"],
    ):
        positions = [group.index(entry) for entry in entries]
        assert positions == sorted(positions)
    assert [label for label, _ in crawler._moduleAuxiliaryPages(source)] == [
        "_helpers", "Unit tests", "Apple", "banana", "cherry", "zeta",
    ]

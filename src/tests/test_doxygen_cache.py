"""Tests for incremental per-project Doxygen XML generation."""

import importlib.util
import os
import pickle
import re
import shutil
import signal
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest


DOXYGEN_CACHE_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs"
    / "source"
    / "_ext"
    / "doxygen_cache.py"
)
sys.path.insert(0, str(DOXYGEN_CACHE_PATH.parent))
SPEC = importlib.util.spec_from_file_location("doxygen_cache", DOXYGEN_CACHE_PATH)
DOXYGEN_CACHE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DOXYGEN_CACHE)

# Tool-dependent tests carry both markers: normal CI excludes ciSkip, while
# the docs job explicitly selects docsIntegration after installing its tools.


def _write_file(path, contents):
    """Write a test file, creating its parent directory first."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf8")


def _project_name_from_config(config_path):
    """Read the project name from a generated test configuration."""
    config = config_path.read_text(encoding="utf8")
    match = re.search(r'^PROJECT_NAME\s*=\s*"([^"]+)"', config, re.MULTILINE)
    assert match
    return match.group(1)


def _install_fake_doxygen(monkeypatch):
    """Install a fake XML generator and return its project-call list."""
    calls = []

    def fake_get_version(_executable):
        return "test-doxygen"

    def fake_run(_executable, config_path, configuration_directory):
        project_name = _project_name_from_config(config_path)
        calls.append(project_name)
        config = config_path.read_text(encoding="utf8")
        assert configuration_directory.is_absolute()
        assert config.endswith(
            f'OUTPUT_DIRECTORY = "{config_path.parent.as_posix()}"\nXML_OUTPUT = xml\n'
        )
        input_config = config.split("INPUT", 1)[1].splitlines()[0]
        input_paths = re.findall(r'"([^"]+)"', input_config)
        input_contents = "".join(
            Path(input_path).read_text(encoding="utf8") for input_path in input_paths
        )
        xml_root = config_path.parent / "xml"
        _write_file(xml_root / "index.xml", f"<index>{project_name}</index>\n")
        _write_file(xml_root / "compound.xml", input_contents)

    monkeypatch.setattr(DOXYGEN_CACHE, "_get_doxygen_version", fake_get_version)
    monkeypatch.setattr(DOXYGEN_CACHE, "_run_doxygen", fake_run)
    return calls


def test_build_doxygen_projects_reuses_unchanged_modules(tmp_path, monkeypatch):
    """Verify only a project with changed input is regenerated."""
    calls = _install_fake_doxygen(monkeypatch)
    source_root = tmp_path / "source"
    cache_root = tmp_path / "cache"
    _write_file(source_root / "module-a" / "a.h", "first A\n")
    _write_file(source_root / "module-b" / "b.h", "first B\n")
    projects = {
        "moduleA": ("source/module-a", ["a.h"]),
        "moduleB": ("source/module-b", ["b.h"]),
    }

    project_paths = DOXYGEN_CACHE.build_doxygen_projects(
        projects, tmp_path, cache_root, doxygen_executable="test-doxygen"
    )
    assert calls == ["moduleA", "moduleB"]

    module_b_xml = Path(project_paths["moduleB"]) / "compound.xml"
    original_timestamp = 1_600_000_000_000_000_000  # [ns]
    os.utime(module_b_xml, ns=(original_timestamp, original_timestamp))
    calls.clear()

    project_paths = DOXYGEN_CACHE.build_doxygen_projects(
        projects, tmp_path, cache_root, doxygen_executable="test-doxygen"
    )
    assert calls == []
    assert module_b_xml.stat().st_mtime_ns == original_timestamp

    _write_file(source_root / "module-a" / "a.h", "changed A\n")
    project_paths = DOXYGEN_CACHE.build_doxygen_projects(
        projects, tmp_path, cache_root, doxygen_executable="test-doxygen"
    )

    assert calls == ["moduleA"]
    assert (Path(project_paths["moduleA"]) / "compound.xml").read_text(
        encoding="utf8"
    ) == "changed A\n"
    assert module_b_xml.stat().st_mtime_ns == original_timestamp


def test_build_doxygen_projects_retains_removed_module_xml(tmp_path, monkeypatch):
    """Verify removed XML remains available to Sphinx until ``make clean``."""
    _install_fake_doxygen(monkeypatch)
    source_root = tmp_path / "source"
    cache_root = tmp_path / "cache"
    _write_file(source_root / "a.h", "A\n")
    _write_file(source_root / "b.h", "B\n")
    projects = {
        "moduleA": ("source", ["a.h"]),
        "moduleB": ("source", ["b.h"]),
    }

    project_paths = DOXYGEN_CACHE.build_doxygen_projects(
        projects, tmp_path, cache_root, doxygen_executable="test-doxygen"
    )
    removed_project_root = Path(project_paths["moduleB"]).parent

    remaining_projects = DOXYGEN_CACHE.build_doxygen_projects(
        {"moduleA": projects["moduleA"]},
        tmp_path,
        cache_root,
        doxygen_executable="test-doxygen",
    )

    assert "moduleB" not in remaining_projects
    assert removed_project_root.exists()


def test_failed_generation_preserves_cached_xml(tmp_path, monkeypatch):
    """Verify failed regeneration cannot replace a valid cached project."""
    _install_fake_doxygen(monkeypatch)
    source_file = tmp_path / "source" / "module.h"
    cache_root = tmp_path / "cache"
    _write_file(source_file, "original\n")
    projects = {"module": ("source", ["module.h"])}
    project_paths = DOXYGEN_CACHE.build_doxygen_projects(
        projects, tmp_path, cache_root, doxygen_executable="test-doxygen"
    )
    cached_xml = Path(project_paths["module"]) / "compound.xml"

    def fail_generation(_executable, config_path, configuration_directory):
        _write_file(config_path.parent / "xml" / "index.xml", "partial\n")
        raise subprocess.CalledProcessError(1, ["test-doxygen", "Doxyfile"])

    _write_file(source_file, "changed\n")
    monkeypatch.setattr(DOXYGEN_CACHE, "_run_doxygen", fail_generation)

    with pytest.raises(subprocess.CalledProcessError):
        DOXYGEN_CACHE.build_doxygen_projects(
            projects, tmp_path, cache_root, doxygen_executable="test-doxygen"
        )

    assert cached_xml.read_text(encoding="utf8") == "original\n"


def test_config_and_version_changes_invalidate_cache(tmp_path, monkeypatch):
    """Verify Doxygen settings and version are part of the cache key."""
    calls = _install_fake_doxygen(monkeypatch)
    source_file = tmp_path / "source" / "module.h"
    cache_root = tmp_path / "cache"
    _write_file(source_file, "source\n")
    projects = {"module": ("source", ["module.h"])}

    DOXYGEN_CACHE.build_doxygen_projects(
        projects,
        tmp_path,
        cache_root,
        options={"WARN_AS_ERROR": "NO"},
        doxygen_executable="test-doxygen",
    )
    calls.clear()

    DOXYGEN_CACHE.build_doxygen_projects(
        projects,
        tmp_path,
        cache_root,
        options={"WARN_AS_ERROR": "YES"},
        doxygen_executable="test-doxygen",
    )
    assert calls == ["module"]
    calls.clear()

    monkeypatch.setattr(
        DOXYGEN_CACHE, "_get_doxygen_version", lambda _executable: "new-version"
    )
    DOXYGEN_CACHE.build_doxygen_projects(
        projects,
        tmp_path,
        cache_root,
        options={"WARN_AS_ERROR": "YES"},
        doxygen_executable="test-doxygen",
    )
    assert calls == ["module"]


def test_configuration_directory_change_invalidates_cache(tmp_path, monkeypatch):
    """Do not reuse XML generated from a different configuration directory."""
    calls = _install_fake_doxygen(monkeypatch)
    source_root = tmp_path / "source"
    _write_file(source_root / "module.h", "source\n")
    for directory_name in ("first", "first", "second"):
        configuration_root = tmp_path / directory_name
        configuration_root.mkdir(exist_ok=True)
        DOXYGEN_CACHE.build_doxygen_projects(
            {"module": (str(source_root), ["module.h"])}, configuration_root,
            tmp_path / "cache", doxygen_executable="test-doxygen",
        )
    assert calls == ["module", "module"]


def test_output_options_cannot_redirect_staged_xml(tmp_path, monkeypatch):
    """Keep output under cache control even when extra settings redirect it."""
    _install_fake_doxygen(monkeypatch)
    _write_file(tmp_path / "source" / "module.h", "source\n")
    paths = DOXYGEN_CACHE.build_doxygen_projects(
        {"module": ("source", ["module.h"])}, tmp_path, tmp_path / "cache",
        options={"OUTPUT_DIRECTORY": "elsewhere", "XML_OUTPUT": "other-xml"},
        doxygen_executable="test-doxygen",
    )
    assert (Path(paths["module"]) / "index.xml").is_file()
    assert not (tmp_path / "elsewhere").exists()


def test_config_options_preserve_insertion_order(tmp_path):
    """Keep include search paths and override precedence in their supplied order."""
    options = {
        "PREDEFINED": "FEATURE_ENABLED=0",
        "@INCLUDE_PATH": "config",
        "@INCLUDE": "common.cfg",
    }
    config = DOXYGEN_CACHE._create_doxygen_config(
        "module", [tmp_path / "module.h"], options, {}, tmp_path / "output"
    )
    expected_options = "\n".join(f"{name}={value}" for name, value in options.items())
    assert expected_options in config


@pytest.mark.parametrize("command", [
    "include", "include{doc}", "includedoc", "includelineno", "dontinclude",
    "snippet", "snippet{doc}", "snippetdoc", "snippetlineno", "verbinclude",
    "htmlinclude", "latexinclude", "rtfinclude", "maninclude", "docbookinclude",
    "xmlinclude", "example", "image", "dotfile", "mscfile", "diafile",
    "plantumlfile", "mermaidfile",
])
def test_documentation_file_commands_disable_reuse(tmp_path, monkeypatch, command):
    """Regenerate file-backed documentation with either command prefix or preprocessing mode."""
    calls = _install_fake_doxygen(monkeypatch)
    source_file = tmp_path / "source" / "module.h"
    for prefix in ("@", "\\"):
        _write_file(source_file, f"/** {prefix}{command} external-file */\n")
        for preprocessing in ("YES", "NO"):
            calls.clear()
            for _ in range(2):
                DOXYGEN_CACHE.build_doxygen_projects(
                    {"module": ("source", ["module.h"])}, tmp_path, tmp_path / "cache",
                    options={"ENABLE_PREPROCESSING": preprocessing},
                    doxygen_executable="test-doxygen",
                )
            assert calls == ["module", "module"]


def test_documentation_aliases_disable_reuse(tmp_path, monkeypatch):
    """Inspect both alias settings even when the source contains only an alias call."""
    calls = _install_fake_doxygen(monkeypatch)
    _write_file(tmp_path / "source" / "module.h", "/** @externalDescription */\n")
    for options, aliases in (
        ({}, {"externalDescription": r"\include{doc} description.dox"}),
        ({"ALIASES": r'externalDescription="\include{doc} description.dox"'}, {}),
    ):
        calls.clear()
        for _ in range(2):
            DOXYGEN_CACHE.build_doxygen_projects(
                {"module": ("source", ["module.h"])}, tmp_path, tmp_path / "cache",
                options=options, aliases=aliases, doxygen_executable="test-doxygen",
            )
        assert calls == ["module", "module"]


def test_preprocessing_disabled_ignores_cpp_includes(tmp_path):
    """Do not follow C includes when Doxygen preprocessing is disabled."""
    source_file = tmp_path / "module.h"
    _write_file(source_file, '#include "config.h"\n/** @brief Local documentation. */\n')
    _write_file(tmp_path / "config.h", "/** @include{doc} external.dox */\n")
    assert DOXYGEN_CACHE._project_dependencies(
        [source_file], {"ENABLE_PREPROCESSING": "NO"}, tmp_path
    ) == [source_file]


@pytest.mark.parametrize("options", [{}, {"SEARCH_INCLUDES": "NO"}])
def test_indirect_header_changes_invalidate_consumers(tmp_path, monkeypatch, options):
    """Track transitive includes, cycles, and newly created or removed headers."""
    calls = _install_fake_doxygen(monkeypatch)
    source_root = tmp_path / "source"
    _write_file(source_root / "module" / "a.h", '#include "../shared/config.h"\n')
    _write_file(source_root / "shared" / "config.h", '#include "feature.inc"\n')
    _write_file(source_root / "other" / "b.h", "unrelated\n")
    projects = {
        "module": ("source/module", ["a.h"]),
        "other": ("source/other", ["b.h"]),
    }

    def build():
        calls.clear()
        DOXYGEN_CACHE.build_doxygen_projects(
            projects, tmp_path, tmp_path / "cache", options=options,
            doxygen_executable="test-doxygen",
        )
        return calls.copy()

    assert build() == ["module", "other"]
    assert build() == []
    feature_file = source_root / "shared" / "feature.inc"
    _write_file(feature_file, '#include "config.h"\n#define FEATURE_ENABLED 1\n')
    assert build() == ["module"]
    assert build() == []
    _write_file(feature_file, '#include "config.h"\n#define FEATURE_ENABLED 0\n')
    assert build() == ["module"]
    feature_file.unlink()
    assert build() == ["module"]


def test_dependency_scan_handles_comments_and_continuations(tmp_path):
    """Keep string contents intact and track includes in inactive branches."""
    source_file = tmp_path / "module.h"
    _write_file(source_file, '''const char *url = "https://example.com/*literal";
/* #include COMPUTED_COMMENT */
#/**/include \\
"config.h" // trailing comment
#if 0
#include <optional.h>
#endif
''')
    config_file = tmp_path / "config.h"
    optional_file = tmp_path / "optional.h"
    _write_file(config_file, "#define VALUE 1\n")
    _write_file(optional_file, "#define OPTIONAL 1\n")
    assert DOXYGEN_CACHE._project_dependencies([source_file], {}, tmp_path) == sorted(
        [source_file, config_file, optional_file]
    )


@pytest.mark.parametrize("options", [{}, {"SEARCH_INCLUDES": "NO"}])
@pytest.mark.parametrize("include_name", ['"settings/config.h"', "<settings/config.h>"])
def test_configuration_headers_invalidate_consumers(tmp_path, monkeypatch, options, include_name):
    """Track configuration-directory headers and their transitive dependencies."""
    calls = _install_fake_doxygen(monkeypatch)
    configuration_root = tmp_path / "docs" / "source"
    configuration_root.mkdir(parents=True)
    _write_file(tmp_path / "src" / "module.h", '#include "bridge.h"\n')
    _write_file(tmp_path / "src" / "bridge.h", f"#include {include_name}\n")
    _write_file(tmp_path / "src" / "other.h", "unrelated\n")
    # Doxygen's configuration directory is independent of the caller's directory.
    monkeypatch.chdir(tmp_path)
    projects = {
        "module": ("../../src", ["module.h"]),
        "other": ("../../src", ["other.h"]),
    }

    def build():
        calls.clear()
        DOXYGEN_CACHE.build_doxygen_projects(
            projects, configuration_root, tmp_path / "cache", options=options,
            doxygen_executable="test-doxygen",
        )
        return calls.copy()

    assert build() == ["module", "other"]
    assert build() == []
    config_file = configuration_root / "settings" / "config.h"
    feature_file = configuration_root / "feature.h"
    _write_file(config_file, '#include "feature.h"\n')
    assert build() == ["module"]
    # This transitive include also resolves from the configuration directory,
    # not from its including header's settings/ directory.
    _write_file(feature_file, "#define FEATURE_ENABLED 1\n")
    assert build() == ["module"]
    assert build() == []
    _write_file(feature_file, "#define FEATURE_ENABLED 0\n")
    assert build() == ["module"]
    feature_file.unlink()
    assert build() == ["module"]
    config_file.unlink()
    assert build() == ["module"]
    assert build() == []


@pytest.mark.parametrize("options", [{}, {"INCLUDE_PATH": "shared"}])
def test_unresolved_dependency_rules_disable_reuse(tmp_path, monkeypatch, options):
    """Regenerate when computed includes or custom search paths need Doxygen."""
    calls = _install_fake_doxygen(monkeypatch)
    source = '#include HEADER_NAME\n' if not options else '#include "config.h"\n'
    _write_file(tmp_path / "source" / "module.h", source)
    for _ in range(2):
        DOXYGEN_CACHE.build_doxygen_projects(
            {"module": ("source", ["module.h"])}, tmp_path, tmp_path / "cache",
            options=options, doxygen_executable="test-doxygen",
        )
    assert calls == ["module", "module"]


@pytest.mark.parametrize("preprocessing", ["YES", "NO"])
def test_bibliography_changes_disable_reuse(tmp_path, monkeypatch, preprocessing):
    """Regenerate bibliography-dependent XML even when preprocessing is disabled."""
    calls = _install_fake_doxygen(monkeypatch)
    fake_run = DOXYGEN_CACHE._run_doxygen
    bibliography = tmp_path / "references.bib"
    _write_file(tmp_path / "source" / "module.h", "/** @cite Reference */\n")

    def generate_bibliography(executable, config_path, configuration_directory):
        fake_run(executable, config_path, configuration_directory)
        _write_file(
            config_path.parent / "xml" / "citelist.xml",
            bibliography.read_text(encoding="utf8"),
        )

    monkeypatch.setattr(DOXYGEN_CACHE, "_run_doxygen", generate_bibliography)
    for title in ("First reference", "First reference", "Second reference"):
        contents = f"@book{{Reference, title={{{title}}}}}\n"
        _write_file(bibliography, contents)
        project_paths = DOXYGEN_CACHE.build_doxygen_projects(
            {"module": ("source", ["module.h"])}, tmp_path, tmp_path / "cache",
            options={
                "CITE_BIB_FILES": '"references.bib"',
                "ENABLE_PREPROCESSING": preprocessing,
            },
            doxygen_executable="test-doxygen",
        )
        assert (Path(project_paths["module"]) / "citelist.xml").read_text(
            encoding="utf8"
        ) == contents
    assert calls == ["module"] * 3


def test_empty_bibliography_setting_allows_reuse(tmp_path):
    """An empty bibliography setting adds no external dependency."""
    source_file = tmp_path / "module.h"
    _write_file(source_file, "/** Documented module. */\n")
    assert DOXYGEN_CACHE._project_dependencies(
        [source_file], {"CITE_BIB_FILES": ""}, tmp_path
    ) == [source_file]


@pytest.mark.parametrize(
    "options, aliases",
    [
        ({"PREDEFINED": "FEATURE_ENABLED=$(BSK_DOXYGEN_TEST_VALUE)"}, {}),
        ({}, {"featureDescription": "$(BSK_DOXYGEN_TEST_VALUE)"}),
    ],
)
def test_environment_references_disable_reuse(tmp_path, monkeypatch, options, aliases):
    """Regenerate environment-dependent options and aliases, including unset values."""
    calls = _install_fake_doxygen(monkeypatch)
    _write_file(tmp_path / "source" / "module.h", "source\n")
    values = [None, "first", "first", "second", None]
    for value in values:
        if value is None:
            monkeypatch.delenv("BSK_DOXYGEN_TEST_VALUE", raising=False)
        else:
            monkeypatch.setenv("BSK_DOXYGEN_TEST_VALUE", value)
        DOXYGEN_CACHE.build_doxygen_projects(
            {"module": ("source", ["module.h"])}, tmp_path, tmp_path / "cache",
            options=options, aliases=aliases, doxygen_executable="test-doxygen",
        )
    assert calls == ["module"] * len(values)


def test_interrupted_publication_invalidates_old_state(tmp_path, monkeypatch):
    """Regenerate mixed XML after a publication failure and source revert."""
    calls = _install_fake_doxygen(monkeypatch)
    source_file = tmp_path / "source" / "module.h"
    _write_file(source_file, "original\n")
    projects = {"module": ("source", ["module.h"])}

    def build():
        return DOXYGEN_CACHE.build_doxygen_projects(
            projects, tmp_path, tmp_path / "cache", doxygen_executable="test-doxygen"
        )

    project_paths = build()
    xml_root = Path(project_paths["module"])
    copyfile = shutil.copyfile

    def interrupted_copy(source, destination):
        copyfile(source, destination)
        raise OSError("Interrupted publication")

    _write_file(source_file, "changed\n")
    with monkeypatch.context() as publication_patch:
        publication_patch.setattr(shutil, "copyfile", interrupted_copy)
        with pytest.raises(OSError, match="Interrupted publication"):
            build()

    assert not (xml_root.parent / "cache-state.json").exists()
    assert (xml_root / "compound.xml").read_text(encoding="utf8") == "changed\n"
    _write_file(source_file, "original\n")
    build()
    assert calls == ["module", "module", "module"]
    assert (xml_root / "compound.xml").read_text(encoding="utf8") == "original\n"


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.skipif(shutil.which("doxygen") is None, reason="Requires Doxygen")
@pytest.mark.parametrize("options", [{}, {"SEARCH_INCLUDES": "NO"}])
@pytest.mark.parametrize("header_location", ["local", "configuration"])
def test_real_doxygen_warm_and_clean_builds_agree(tmp_path, capsys, options, header_location):
    """Compare warm and clean XML after local or configuration-header changes."""
    if header_location == "local":
        header = tmp_path / "source" / "shared" / "config.h"
        include_name = "../shared/config.h"
    else:
        header = tmp_path / "config.h"
        include_name = "config.h"
    _write_file(tmp_path / "source" / "module" / "module.h", f'''/** @file module.h */
#include "{include_name}"
#if FEATURE_ENABLED
/** Enabled feature. */
void enabled(void);
#else
/** Disabled feature. */
void disabled(void);
#endif
''')

    def build(cache_name, generated=True):
        paths = DOXYGEN_CACHE.build_doxygen_projects(
            {"module": ("source/module", ["module.h"])}, tmp_path, tmp_path / cache_name,
            options=options,
        )
        expected_summary = "1 generated, 0 reused" if generated else "0 generated, 1 reused"
        assert expected_summary in capsys.readouterr().out
        index = ET.parse(Path(paths["module"]) / "index.xml")
        return [node.text for node in index.findall(".//member/name")]

    for step, feature_enabled in enumerate((None, 1, 0, None, 1)):
        if feature_enabled is None:
            header.unlink(missing_ok=True)
        else:
            _write_file(header, f"#define FEATURE_ENABLED {feature_enabled}\n")
        expected = ["enabled" if feature_enabled else "disabled"]
        assert build("cache") == build(f"clean-cache-{step}") == expected
        assert build("cache", generated=False) == expected


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.skipif(shutil.which("doxygen") is None, reason="Requires Doxygen")
@pytest.mark.parametrize("setting", ["option", "alias"])
def test_real_doxygen_environment_changes_match_clean_build(tmp_path, monkeypatch, setting):
    """Compare real declarations and descriptions after environment-only changes."""
    _write_file(tmp_path / "source" / "module.h", '''/** @file module.h */
#if FEATURE_ENABLED
/** @brief @featureDescription */
void enabled(void);
#else
/** @brief Disabled feature. */
void disabled(void);
#endif
''')
    options = {"PREDEFINED": "FEATURE_ENABLED=1"}
    aliases = {"featureDescription": "Initial description."}
    if setting == "option":
        options["PREDEFINED"] = "FEATURE_ENABLED=$(BSK_DOXYGEN_TEST_VALUE)"
        initial_value, changed_value = "1", "0"
        expected = [("disabled", "Disabled feature.")]
    else:
        aliases["featureDescription"] = "$(BSK_DOXYGEN_TEST_VALUE)"
        initial_value, changed_value = "Initial description.", "Changed description."
        expected = [("enabled", "Changed description.")]

    def build(cache_name):
        paths = DOXYGEN_CACHE.build_doxygen_projects(
            {"module": ("source", ["module.h"])}, tmp_path, tmp_path / cache_name,
            options=options, aliases=aliases,
        )
        xml_root = Path(paths["module"])
        index = ET.parse(xml_root / "index.xml")
        file_id = index.find("compound[@kind='file']").get("refid")
        compound = ET.parse(xml_root / (file_id + ".xml"))
        return [
            (
                member.findtext("name"),
                "".join(member.find("briefdescription").itertext()).strip(),
            )
            for member in compound.findall(".//memberdef")
        ]

    monkeypatch.setenv("BSK_DOXYGEN_TEST_VALUE", initial_value)
    assert build("cache") == [("enabled", "Initial description.")]
    monkeypatch.setenv("BSK_DOXYGEN_TEST_VALUE", changed_value)
    assert build("cache") == build("clean-cache") == expected


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.skipif(shutil.which("doxygen") is None, reason="Requires Doxygen")
@pytest.mark.parametrize("override_after_include", [False, True])
def test_real_doxygen_configuration_include_order(tmp_path, override_after_include):
    """Resolve an included configuration and honor overrides on either side of it."""
    _write_file(tmp_path / "config" / "common.cfg", "PREDEFINED = FEATURE_ENABLED=1\n")
    _write_file(tmp_path / "source" / "module.h", '''/** @file module.h */
#if FEATURE_ENABLED
/** Enabled feature. */
void enabled(void);
#else
/** Disabled feature. */
void disabled(void);
#endif
''')
    includes = {"@INCLUDE_PATH": "config", "@INCLUDE": "common.cfg"}
    override = {"PREDEFINED": "FEATURE_ENABLED=0"}
    options = (
        {**includes, **override} if override_after_include else {**override, **includes}
    )
    paths = DOXYGEN_CACHE.build_doxygen_projects(
        {"module": ("source", ["module.h"])}, tmp_path, tmp_path / "cache", options=options,
    )
    index = ET.parse(Path(paths["module"]) / "index.xml")
    assert [node.text for node in index.findall(".//member/name")] == [
        "disabled" if override_after_include else "enabled"
    ]


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.skipif(shutil.which("doxygen") is None, reason="Requires Doxygen")
@pytest.mark.parametrize("preprocessing", ["YES", "NO"])
@pytest.mark.parametrize("use_alias", [False, True])
def test_real_doxygen_documentation_include_matches_clean_build(
    tmp_path, capsys, preprocessing, use_alias
):
    """Refresh included documentation after external-only edits, with or without aliases."""
    description_file = tmp_path / "description.dox"
    _write_file(description_file, "First external description.\n")
    include_command = r"\include{doc} " + description_file.as_posix()
    aliases = {"externalDescription": include_command} if use_alias else {}
    documentation = "@externalDescription" if use_alias else include_command
    _write_file(tmp_path / "source" / "module.h", f'''/** @file module.h */
/**
 * @brief Included documentation.
 *
 * {documentation}
 */
void example(void);
''')

    def build(cache_name):
        paths = DOXYGEN_CACHE.build_doxygen_projects(
            {"module": ("source", ["module.h"])}, tmp_path, tmp_path / cache_name,
            options={"ENABLE_PREPROCESSING": preprocessing}, aliases=aliases,
        )
        assert "1 generated, 0 reused" in capsys.readouterr().out
        xml_root = Path(paths["module"])
        index = ET.parse(xml_root / "index.xml")
        file_id = index.find("compound[@kind='file']").get("refid")
        compound = ET.parse(xml_root / (file_id + ".xml"))
        return "".join(compound.find(".//memberdef/detaileddescription").itertext()).strip()

    assert build("cache") == "First external description."
    _write_file(description_file, "Second external description.\n")
    assert build("cache") == build("clean-cache") == "Second external description."


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.skipif(shutil.which("doxygen") is None, reason="Requires Doxygen")
@pytest.mark.parametrize("setting", ["INCLUDE_PATH", "TAGFILES", "INPUT_FILTER"])
def test_real_doxygen_resolves_relative_settings(tmp_path, monkeypatch, capsys, setting):
    """Resolve relative inputs from conf.py's directory, including paths with spaces."""
    project_root = tmp_path / "project with spaces"
    configuration_root = project_root / "docs" / "source"
    configuration_root.mkdir(parents=True)
    module_file = project_root / "src" / "module" / "module.h"
    options = {}
    expected_references = []
    if setting == "INCLUDE_PATH":
        _write_file(project_root / "src" / "shared" / "config.h", "#define FEATURE_ENABLED 1\n")
        _write_file(module_file, '''/** @file module.h */
#include "config.h"
#if FEATURE_ENABLED
/** Expected declaration. */
void enabled(void);
#else
/** Declaration when the include path is lost. */
void disabled(void);
#endif
''')
        options[setting] = "../../src/shared"
        expected_members = ["enabled"]
    elif setting == "TAGFILES":
        _write_file(project_root / "external" / "tags.xml", '''<?xml version="1.0"?>
<tagfile>
  <compound kind="class">
    <name>External</name><filename>external.html</filename>
  </compound>
</tagfile>
''')
        _write_file(module_file, '''/** @file module.h */
/** @brief See @ref External. */
void example(void);
''')
        options[setting] = "../../external/tags.xml=https://example.invalid/api"
        expected_members = ["example"]
        expected_references = ["External"]
    else:
        _write_file(configuration_root / "filters" / "filter.py", '''import sys
from pathlib import Path
sys.stdout.buffer.write(Path(sys.argv[1]).read_bytes().replace(b"unfiltered", b"filtered"))
''')
        _write_file(module_file, '''/** @file module.h */
/** Filtered declaration. */
void unfiltered(void);
''')
        options[setting] = DOXYGEN_CACHE._quote_doxygen_value(
            f'"{sys.executable}" filters/filter.py'
        )
        expected_members = ["filtered"]

    # The caller's working directory must not affect Doxygen path resolution.
    monkeypatch.chdir(tmp_path)
    for cache_name in ("cache", "cache", "clean-cache"):
        paths = DOXYGEN_CACHE.build_doxygen_projects(
            {"module": ("../../src/module", ["module.h"])}, configuration_root,
            project_root / "build" / cache_name, options=options,
        )
        assert "1 generated, 0 reused" in capsys.readouterr().out
        xml_root = Path(paths["module"])
        index = ET.parse(xml_root / "index.xml")
        assert [node.text for node in index.findall(".//member/name")] == expected_members
        file_id = index.find("compound[@kind='file']").get("refid")
        compound = ET.parse(xml_root / (file_id + ".xml"))
        assert [node.text for node in compound.findall(".//ref[@external]")] == expected_references
    assert not (configuration_root / "xml").exists()


def test_parallel_merge_preserves_shared_xml_dependencies():
    """Merge only each worker's documents without losing shared XML consumers."""
    earlier_mtime = 1_600_000_000.0  # [s] Unix timestamp
    later_mtime = earlier_mtime + 1.0  # [s]
    environment = SimpleNamespace(breathe_file_state={
        "shared.xml": (earlier_mtime, {"unchanged"}),
    })
    worker = SimpleNamespace(breathe_file_state={
        "shared.xml": (later_mtime, {"first", "inherited"}),
        "first.xml": (later_mtime, {"first"}),
        "inherited.xml": (later_mtime, {"inherited"}),
    })
    DOXYGEN_CACHE._merge_breathe_file_state(None, environment, {"first"}, worker)
    other_worker = SimpleNamespace(breathe_file_state={
        "shared.xml": (later_mtime, {"second", "inherited"}),
    })
    DOXYGEN_CACHE._merge_breathe_file_state(None, environment, {"second"}, other_worker)
    assert environment.breathe_file_state == {
        "shared.xml": (earlier_mtime, {"unchanged", "first", "second"}),
        "first.xml": (later_mtime, {"first"}),
    }
    # Do not share mutable document sets with a worker environment.
    worker.breathe_file_state["first.xml"][1].clear()
    assert environment.breathe_file_state["first.xml"][1] == {"first"}


def test_parallel_merge_handles_missing_dependency_state():
    """Accept workers without Breathe directives and initialize parent state lazily."""
    environment = SimpleNamespace()
    DOXYGEN_CACHE._merge_breathe_file_state(None, environment, {"plain"}, SimpleNamespace())
    assert not hasattr(environment, "breathe_file_state")
    observed_mtime = 1_600_000_000.0  # [s] Unix timestamp
    worker = SimpleNamespace(breathe_file_state={"module.xml": (observed_mtime, {"module"})})
    DOXYGEN_CACHE._merge_breathe_file_state(None, environment, {"module"}, worker)
    assert environment.breathe_file_state == {"module.xml": (observed_mtime, {"module"})}


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.skipif(shutil.which("doxygen") is None, reason="Requires Doxygen")
@pytest.mark.skipif(os.name != "posix", reason="Sphinx parallel reading requires fork")
@pytest.mark.parametrize("legacy_environment", [False, True])
def test_parallel_sphinx_tracks_xml_changes(tmp_path, legacy_environment):
    """Refresh shared API pages after parallel reads and recover older saved environments."""
    pytest.importorskip("sphinx")
    pytest.importorskip("breathe")
    source_root = tmp_path / "source"
    _write_file(source_root / "conf.py", f'''import sys
sys.path.insert(0, {str(DOXYGEN_CACHE_PATH.parent)!r})
extensions = ["doxygen_cache"]
project = "Parallel cache test"
bsk_doxygen_projects_source = {{
    "module": (".", ["module.h"]), "unrelated": (".", ["unrelated.h"]),
}}
''')
    # Separate consumers of the same XML into different worker chunks.
    pages = ["a_first"] + [f"page{index}" for index in range(8)] + ["unrelated", "z_second"]
    _write_file(source_root / "index.rst", "Parallel cache test\n===================\n\n"
                ".. toctree::\n\n" + "".join(f"   {page}\n" for page in pages))
    for page in pages:
        title = page.replace("_", " ")
        contents = f"{title}\n{'=' * len(title)}\n\n"
        if page in ("a_first", "z_second"):
            function_name = page.split("_", 1)[1]
            contents += f".. doxygenfunction:: {function_name}\n   :project: module\n"
        elif page == "unrelated":
            contents += ".. doxygenfile:: unrelated.h\n   :project: unrelated\n"
        _write_file(source_root / f"{page}.rst", contents)
    _write_file(source_root / "unrelated.h", '''/** @file unrelated.h */
/** Unrelated documentation. */
void unrelated(void);
''')
    header = source_root / "module.h"
    _write_file(header, '''/** @file module.h */
/** InitialDescription */
void first(void);
/** InitialDescription */
void second(void);
''')
    build_root = tmp_path / "build"
    environment_path = build_root / "doctrees" / "environment.pickle"

    def build():
        result = subprocess.run(
            [sys.executable, "-m", "sphinx", "-M", "html", str(source_root),
             str(build_root), "-j", "2", "-W"], capture_output=True, text=True,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        return result.stdout

    def read_environment():
        with environment_path.open("rb") as stream:
            return pickle.load(stream)

    assert "2 generated, 0 reused" in build()
    if legacy_environment:
        # Reproduce a pre-fix environment: no extension environment version and
        # no Breathe dependency state survived the parallel workers.
        environment = read_environment()
        environment.version.pop("doxygen_cache", None)
        environment.breathe_file_state = {}
        with environment_path.open("wb") as stream:
            pickle.dump(environment, stream)
        assert "0 generated, 2 reused" in build()
    state = read_environment().breathe_file_state
    assert any(docnames == {"a_first", "z_second"} for _, docnames in state.values())
    unrelated_doctree = build_root / "doctrees" / "unrelated.doctree"
    unrelated_mtime = unrelated_doctree.stat().st_mtime_ns

    for description in ("UpdatedDescription", "FinalDescription"):
        _write_file(header, f'''/** @file module.h */
/** {description} */
void first(void);
/** {description} */
void second(void);
''')
        assert "1 generated, 1 reused" in build()
        for page in ("a_first", "z_second"):
            html = (build_root / "html" / f"{page}.html").read_text(encoding="utf8")
            assert description in html and "InitialDescription" not in html
        assert unrelated_doctree.stat().st_mtime_ns == unrelated_mtime
        unchanged_output = build()
        assert "0 generated, 2 reused" in unchanged_output
        assert "no targets are out of date" in unchanged_output


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.skipif(shutil.which("doxygen") is None, reason="Requires Doxygen")
@pytest.mark.parametrize("make_mode", [False, True])
def test_sphinx_honors_overrides_and_reuses_xml(tmp_path, make_mode):
    """Apply command-line options and aliases before caching or rendering XML."""
    pytest.importorskip("sphinx")
    pytest.importorskip("breathe")
    source_root = tmp_path / "source"
    _write_file(source_root / "conf.py", f'''import sys
sys.path.insert(0, {str(DOXYGEN_CACHE_PATH.parent)!r})
extensions = ["doxygen_cache"]
project = "Cache integration test"
bsk_doxygen_projects_source = {{"module": (".", ["module.h"])}}
breathe_doxygen_config_options = {{"PREDEFINED": "FEATURE_ENABLED=0"}}
breathe_doxygen_aliases = {{"featureDescription": "Default description."}}
''')
    _write_file(source_root / "module.h", '''/** @file module.h */
#if FEATURE_ENABLED
/** @brief @featureDescription */
void enabled(void);
#else
/** Disabled feature. */
void disabled(void);
#endif
''')
    _write_file(source_root / "index.rst", '''Cache integration test
======================

.. doxygenfile:: module.h
   :project: module
''')
    build_root = tmp_path / "build"
    html_root = build_root / "html"
    command = [sys.executable, "-m", "sphinx"]
    if make_mode:
        command += ["-M", "html", str(source_root), str(build_root), "-W"]
    else:
        command += ["-b", "html", "-W", str(source_root), str(html_root)]

    def build(overrides=()):
        result = subprocess.run(command + list(overrides), capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
        return result.stdout

    assert "1 generated, 0 reused" in build()
    assert "disabled" in (html_root / "index.html").read_text(encoding="utf8")
    overrides = (
        "-D", "breathe_doxygen_config_options.PREDEFINED=FEATURE_ENABLED=1",
        "-D", "breathe_doxygen_aliases.featureDescription=Overridden description.",
    )
    assert "1 generated, 0 reused" in build(overrides)
    html = (html_root / "index.html").read_text(encoding="utf8")
    assert "enabled" in html and "Overridden description." in html
    xml_files = list((build_root / "doxygen-cache").rglob("*.xml"))
    assert xml_files
    assert not (html_root / "doxygen-cache").exists()
    timestamps = {path: path.stat().st_mtime_ns for path in xml_files}
    assert "0 generated, 1 reused" in build(overrides)
    assert timestamps == {path: path.stat().st_mtime_ns for path in xml_files}


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.skipif(shutil.which("doxygen") is None, reason="Requires Doxygen")
@pytest.mark.skipif(
    os.name == "nt" or shutil.which("make") is None,
    reason="Exercises the POSIX Makefile's BUILDDIR override",
)
def test_alternate_build_cleanup_removes_its_xml_cache(tmp_path):
    """Clean an alternate build without retaining XML or touching another build."""
    pytest.importorskip("sphinx")
    pytest.importorskip("breathe")
    source_root = tmp_path / "source"
    _write_file(source_root / "conf.py", f'''import sys
sys.path.insert(0, {str(DOXYGEN_CACHE_PATH.parent)!r})
extensions = ["doxygen_cache"]
project = "Alternate build test"
bsk_doxygen_projects_source = {{"module": (".", ["module.h"])}}
''')
    _write_file(source_root / "module.h", '''/** @file module.h */
/** @brief Example declaration. */
void example(void);
''')
    _write_file(source_root / "index.rst", '''Alternate build test
====================

.. doxygenfile:: module.h
   :project: module
''')
    repository_root = DOXYGEN_CACHE_PATH.parents[3]
    shutil.copyfile(repository_root / "docs" / "Makefile", tmp_path / "Makefile")

    def build(directory_name):
        result = subprocess.run(
            [sys.executable, "-m", "sphinx", "-M", "html", str(source_root),
             str(tmp_path / directory_name), "-W"],
            cwd=tmp_path, capture_output=True, text=True,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        return result.stdout

    assert "1 generated, 0 reused" in build("build")
    default_xml = list((tmp_path / "build" / "doxygen-cache").rglob("*.xml"))
    assert default_xml
    default_timestamps = {path: path.stat().st_mtime_ns for path in default_xml}
    assert "1 generated, 0 reused" in build("custom-build")
    assert list((tmp_path / "custom-build" / "doxygen-cache").rglob("index.xml"))
    assert "0 generated, 1 reused" in build("custom-build")

    # Use the real cleanup recipe, but only against this test-owned fixture.
    result = subprocess.run(
        ["make", "clean", "BUILDDIR=custom-build"], cwd=tmp_path,
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert not (tmp_path / "custom-build").exists()
    assert default_timestamps == {path: path.stat().st_mtime_ns for path in default_xml}
    assert "1 generated, 0 reused" in build("custom-build")


def test_run_doxygen_retries_sigbus_only(tmp_path, monkeypatch):
    """Verify transient ``SIGBUS`` termination is retried from clean output."""
    sigbus = getattr(signal, "SIGBUS", 10)
    monkeypatch.setattr(DOXYGEN_CACHE.signal, "SIGBUS", sigbus, raising=False)
    return_codes = iter([-sigbus, 0])
    calls = []

    def fake_subprocess_run(command, cwd):
        calls.append((command, cwd))
        assert command == ["doxygen", str(config_path)]
        assert cwd == configuration_root
        return_code = next(return_codes)
        if return_code:
            _write_file(config_path.parent / "xml" / "partial.xml", "partial\n")
        else:
            assert not (config_path.parent / "xml").exists()
        return subprocess.CompletedProcess(command, return_code)

    monkeypatch.setattr(DOXYGEN_CACHE.subprocess, "run", fake_subprocess_run)
    configuration_root = tmp_path / "source"
    _write_file(configuration_root / "xml" / "keep.xml", "not staging output\n")
    config_path = tmp_path / "staging" / "Doxyfile"
    _write_file(config_path, "")

    DOXYGEN_CACHE._run_doxygen("doxygen", config_path, configuration_root)

    assert len(calls) == 2
    assert (configuration_root / "xml" / "keep.xml").is_file()


def test_run_doxygen_does_not_retry_regular_errors(tmp_path, monkeypatch):
    """Verify a normal Doxygen error is reported without retrying."""
    calls = []

    def fake_subprocess_run(command, cwd):
        calls.append((command, cwd))
        return subprocess.CompletedProcess(command, 1)

    monkeypatch.setattr(DOXYGEN_CACHE.subprocess, "run", fake_subprocess_run)
    config_path = tmp_path / "Doxyfile"
    config_path.touch()

    with pytest.raises(subprocess.CalledProcessError):
        DOXYGEN_CACHE._run_doxygen("doxygen", config_path, tmp_path)

    assert len(calls) == 1

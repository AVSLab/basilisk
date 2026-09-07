"""Tests for content-aware synchronization of generated documentation."""

import importlib.util
import os
from pathlib import Path
import re
import subprocess
import sys

import pytest


DOCUMENTATION_SYNC_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs"
    / "source"
    / "_ext"
    / "generated_documentation.py"
)
SPEC = importlib.util.spec_from_file_location(
    "generated_documentation", DOCUMENTATION_SYNC_PATH
)
GENERATED_DOCUMENTATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATED_DOCUMENTATION)


def _write_file(path, contents):
    """Write a test file, creating its parent directory first."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(contents, encoding="utf8")


def test_sync_generated_tree_preserves_unchanged_files(tmp_path):
    """Verify synchronization updates content without touching equal files."""
    staged_root = tmp_path / "staged"
    destination_root = tmp_path / "destination"

    _write_file(staged_root / "unchanged.rst", "unchanged\n")
    _write_file(staged_root / "updated.rst", "new contents\n")
    _write_file(staged_root / "new" / "added.rst", "added\n")
    _write_file(staged_root / "was-a-file" / "child.rst", "child\n")

    _write_file(destination_root / "unchanged.rst", "unchanged\n")
    _write_file(destination_root / "updated.rst", "old contents\n")
    _write_file(destination_root / "stale" / "obsolete.rst", "obsolete\n")
    _write_file(destination_root / "was-a-file", "obsolete blocker\n")

    unchanged_file = destination_root / "unchanged.rst"
    original_timestamp = 1_600_000_000_000_000_000  # [ns]
    os.utime(unchanged_file, ns=(original_timestamp, original_timestamp))

    counts = GENERATED_DOCUMENTATION.sync_generated_tree(
        staged_root, destination_root
    )

    assert unchanged_file.stat().st_mtime_ns == original_timestamp
    assert (destination_root / "updated.rst").read_text(encoding="utf8") == (
        "new contents\n"
    )
    assert (destination_root / "new" / "added.rst").is_file()
    assert (destination_root / "was-a-file" / "child.rst").is_file()
    assert not (destination_root / "stale").exists()
    assert counts == {"added": 2, "updated": 1, "unchanged": 1, "removed": 2}


def test_sync_generated_tree_replaces_directory_with_file(tmp_path):
    """Verify a generated file can replace an obsolete directory tree."""
    staged_root = tmp_path / "staged"
    destination_root = tmp_path / "destination"

    _write_file(staged_root / "replacement.rst", "replacement\n")
    _write_file(destination_root / "replacement.rst" / "obsolete.rst", "old\n")

    GENERATED_DOCUMENTATION.sync_generated_tree(staged_root, destination_root)

    replacement = destination_root / "replacement.rst"
    assert replacement.is_file()
    assert replacement.read_text(encoding="utf8") == "replacement\n"


def test_sync_generated_tree_creates_missing_destination(tmp_path):
    """Verify synchronization supports the first build after ``make clean``."""
    staged_root = tmp_path / "staged"
    destination_root = tmp_path / "missing-destination"
    _write_file(staged_root / "index.rst", "Generated documentation\n")

    counts = GENERATED_DOCUMENTATION.sync_generated_tree(
        staged_root, destination_root
    )

    assert (destination_root / "index.rst").is_file()
    assert counts == {"added": 1, "updated": 0, "unchanged": 0, "removed": 0}


def test_sync_generated_tree_can_preserve_stale_files(tmp_path):
    """Verify dependency files can remain until their consumer purges them."""
    staged_root = tmp_path / "staged"
    destination_root = tmp_path / "destination"
    _write_file(staged_root / "current.xml", "current\n")
    _write_file(destination_root / "current.xml", "old\n")
    _write_file(destination_root / "stale.xml", "stale\n")

    counts = GENERATED_DOCUMENTATION.sync_generated_tree(
        staged_root, destination_root, remove_stale=False
    )

    assert (destination_root / "current.xml").read_text(encoding="utf8") == "current\n"
    assert (destination_root / "stale.xml").is_file()
    assert counts == {"added": 0, "updated": 1, "unchanged": 0, "removed": 0}


# Tool-dependent checks run in docs CI after its dependencies are installed.
@pytest.mark.ciSkip
@pytest.mark.docsIntegration
def test_incremental_build_refreshes_sibling_navigation(tmp_path):
    """Update an unchanged sibling's sidebar when a module bypasses its folder."""
    pytest.importorskip("sphinx")
    source = tmp_path / "source"
    output = tmp_path / "html"
    cache = tmp_path / "doctrees"
    _write_file(source / "conf.py", (
        "import sys\n"
        f"sys.path.insert(0, {str(DOCUMENTATION_SYNC_PATH.parent)!r})\n"
        "extensions = ['generated_documentation']\n"
        "master_doc = 'index'\n"
        "html_theme = 'classic'\n"
        "html_sidebars = {'**': ['globaltoc.html']}\n"
    ))
    _write_file(source / "index.rst", (
        "Home\n====\n\n.. toctree::\n\n   folder\n   sibling\n"
    ))
    _write_file(source / "folder.rst", (
        "Folder\n======\n\n.. toctree::\n\n   module\n"
    ))
    _write_file(source / "module.rst", "Module\n======\n\nModule content.\n")
    _write_file(source / "sibling.rst", "Sibling\n=======\n\nUnchanged content.\n")
    sibling_timestamp = (source / "sibling.rst").stat().st_mtime_ns

    def build():
        """Build incrementally using the same cached Sphinx environment."""
        result = subprocess.run(
            [sys.executable, "-m", "sphinx", "-b", "html", "-W", "-q",
             "-d", str(cache), str(source), str(output)],
            capture_output=True, text=True, check=False,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        return (output / "sibling.html").read_text(encoding="utf8")

    assert 'href="folder.html"' in build()
    _write_file(source / "index.rst", (
        "Home\n====\n\n.. toctree::\n\n   Direct module <module>\n   sibling\n"
    ))
    _write_file(source / "folder.rst", (
        ":orphan:\n\nFolder\n======\n\n:doc:`Module documentation <module>`\n"
    ))
    html = build()
    assert 'href="folder.html"' not in html
    assert 'href="module.html"' in html
    assert "Direct module" in html
    assert (source / "sibling.rst").stat().st_mtime_ns == sibling_timestamp

    # A subsequent no-change build must not rewrite the page again.
    html_timestamp = (output / "sibling.html").stat().st_mtime_ns
    build()
    assert (output / "sibling.html").stat().st_mtime_ns == html_timestamp


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
def test_local_contents_are_separate_from_site_navigation(tmp_path):
    """Render local links only for long guides, with valid existing anchors."""
    pytest.importorskip("sphinx")
    pytest.importorskip("sphinx_rtd_theme")
    source = tmp_path / "source"
    output = tmp_path / "html"
    templates = DOCUMENTATION_SYNC_PATH.parent.parent / "_templates"
    _write_file(source / "conf.py", (
        "import sys\n"
        f"sys.path.insert(0, {str(DOCUMENTATION_SYNC_PATH.parent)!r})\n"
        "extensions = ['generated_documentation']\n"
        "master_doc = 'index'\n"
        "html_theme = 'sphinx_rtd_theme'\n"
        "html_theme_options = {'titles_only': True}\n"
        f"templates_path = [{str(templates)!r}]\n"
    ))
    _write_file(source / "index.rst", (
        "Home\n====\n\n.. toctree::\n\n   guide\n   short\n   Documentation/module\n"
    ))
    sections = "".join(
        f"Section {index}\n---------\n\nText.\n\n" for index in range(4)
    )
    guide = "Guide\n=====\n\nIntroduction.\n\n" + sections + (
        "Subsection & details\n~~~~~~~~~~~~~~~~~~~~\n\nDetails.\n"
    )
    _write_file(source / "guide.rst", guide)
    _write_file(source / "Documentation/module.rst", guide)
    _write_file(source / "short.rst", "Short\n=====\n\nOverview\n--------\n\nText.\n")
    result = subprocess.run(
        [sys.executable, "-m", "sphinx", "-b", "html", "-W", "-q",
         str(source), str(output)],
        capture_output=True, text=True, check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    html = (output / "guide.html").read_text(encoding="utf8")
    assert '<details class="bsk-page-navigation">' in html
    assert '<summary>On this page</summary>' in html
    assert html.index("</h1>") < html.index('<details class="bsk-page-navigation">')
    navigation = html.split('<nav aria-label="On this page">')[1].split("</nav>")[0]
    assert "Subsection &amp; details" in navigation
    anchors = re.findall(r'href="#([^"]+)"', navigation)
    assert len(anchors) == 5
    for anchor in anchors:
        assert f'id="{anchor}"' in html
    sidebar = html.split('aria-label="Navigation menu"')[1].split("</nav>")[0]
    assert "Section 0" not in sidebar
    assert "guide.html" in (output / "short.html").read_text(encoding="utf8")
    for page in ("short", "Documentation/module"):
        assert 'class="bsk-page-navigation"' not in (
            output / (page + ".html")
        ).read_text(encoding="utf8")

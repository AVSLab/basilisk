# ISC License
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

"""Check snippet compilation and incremental Sphinx rendering for both collections."""

from html.parser import HTMLParser
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


SOURCE = Path(__file__).resolve().parents[2] / "docs" / "source"
COMPILER_PATH = SOURCE / "_ext" / "rst_snippets.py"
SPEC = importlib.util.spec_from_file_location("rst_snippets", COMPILER_PATH)
COMPILER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COMPILER)


def _write(path, content):
    """Write a UTF-8 fixture, creating its directory if needed."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


@pytest.mark.parametrize("folder", COMPILER.SNIPPET_FOLDERS.values())
def test_compile_filters_sorts_and_preserves_rst(tmp_path, folder):
    """Keep multiline RST and separate files while excluding support artifacts."""
    snippets = tmp_path / folder
    first = "- First issue.\n\n  Workaround::\n\n      use_this()"
    second = "- Second issue with UTF-8: café."
    _write(snippets / "a-first.rst", first + "\n")
    _write(snippets / "B-second.RST", second)
    _write(snippets / "empty.rst", " \n\n")
    _write(snippets / "README.md", "Not RST")
    _write(snippets / "_sample.rst", "- Sample")
    _write(snippets / "_compiled_latest.rst", "- Stale generated text")
    _write(snippets / "nested.rst" / "hidden.rst", "- Not a top-level snippet")

    output = COMPILER.compile_snippets(snippets)

    assert output.read_text(encoding="utf-8") == first + "\n\n" + second + "\n"
    assert (snippets / "a-first.rst").read_text(encoding="utf-8") == first + "\n"


def test_compile_preserves_mtime_and_clears_consumed_entries(tmp_path):
    """Avoid unnecessary rebuilds and remove stale content after snippets are consumed."""
    snippet = tmp_path / "issue.rst"
    _write(snippet, "- An issue.\n")
    output = COMPILER.compile_snippets(tmp_path)
    timestamp = 1_600_000_000_000_000_000  # [ns]
    os.utime(output, ns=(timestamp, timestamp))

    COMPILER.compile_snippets(tmp_path)
    assert output.stat().st_mtime_ns == timestamp

    snippet.unlink()
    COMPILER.compile_snippets(tmp_path)
    assert output.read_text(encoding="utf-8") == ""


def test_cli_and_legacy_wrapper_from_another_directory(tmp_path):
    """Retain the original command and allow independent or combined compilation."""
    source = tmp_path / "checkout" / "docs" / "source"
    compiler = source / "_ext" / "rst_snippets.py"
    _write(compiler, COMPILER_PATH.read_text(encoding="utf-8"))
    for folder in COMPILER.SNIPPET_FOLDERS.values():
        _write(source / "Support" / folder / "entry.rst", f"- {folder}\n")
    wrapper_relative = (
        Path("Support") / "bskReleaseNotesSnippets" / "_compile_release_notes_snippets.py"
    )
    shutil.copyfile(SOURCE / wrapper_relative, source / wrapper_relative)
    release_output = source / "Support/bskReleaseNotesSnippets/_compiled_latest.rst"
    issues_output = source / "Support/bskKnownIssuesSnippets/_compiled_latest.rst"

    subprocess.run([sys.executable, str(source / wrapper_relative)], cwd=tmp_path, check=True)
    assert release_output.is_file()
    assert not issues_output.exists()

    release_output.unlink()
    subprocess.run([sys.executable, str(compiler), "known-issues"], cwd=tmp_path, check=True)
    assert issues_output.is_file()
    assert not release_output.exists()

    subprocess.run([sys.executable, str(compiler)], cwd=tmp_path, check=True)
    assert release_output.read_text(encoding="utf-8") == "- bskReleaseNotesSnippets\n"
    assert issues_output.read_text(encoding="utf-8") == "- bskKnownIssuesSnippets\n"


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.parametrize("builder, suffix", [("html", "html"), ("text", "txt")])
def test_sphinx_refreshes_both_collections_incrementally(tmp_path, builder, suffix):
    """Render references and structured RST, then detect additions, edits, and deletions."""
    pytest.importorskip("sphinx")
    source = tmp_path / "source"
    output = tmp_path / "output"
    release_folder = source / "Support" / "bskReleaseNotesSnippets"
    issues_folder = source / "Support" / "bskKnownIssuesSnippets"
    _write(source / "conf.py", (
        "import sys\n"
        f"sys.path.insert(0, {str(COMPILER_PATH.parent)!r})\n"
        "extensions = ['rst_snippets']\n"
        "exclude_patterns = ['Support/*Snippets/*']\n"
        "master_doc = 'index'\n"
    ))
    _write(source / "index.rst", (
        ".. _snippet-page:\n\n"
        "Snippets\n========\n\n"
        "Release notes\n-------------\n\n"
        ".. include:: Support/bskReleaseNotesSnippets/_compiled_latest.rst\n\n"
        "Known issues\n------------\n\n"
        ".. include:: Support/bskKnownIssuesSnippets/_compiled_latest.rst\n"
    ))
    _write(release_folder / "change.rst", "- Release entry with :ref:`snippet-page`.\n")
    _write(issues_folder / "a-issue.rst", (
        "- Original issue.\n\n"
        "  Workaround paragraph.\n\n"
        "  - Nested workaround step.\n\n"
        "  .. code-block:: python\n\n"
        "     use_this()\n"
    ))
    _write(issues_folder / "b-issue.rst", "- Removed issue.\n")

    def build():
        """Build directly with Sphinx, treating every warning as a failure."""
        result = subprocess.run(
            [sys.executable, "-m", "sphinx", "-b", builder, "-W", "--keep-going",
             str(source), str(output)],
            capture_output=True, text=True,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        rendered = (output / f"index.{suffix}").read_text(encoding="utf-8")
        if builder == "html":
            parser = HTMLParser()
            text_parts = []
            parser.handle_data = text_parts.append
            parser.feed(rendered)
            return "".join(text_parts)
        return rendered

    rendered = build()
    for expected in ("Release entry", "Original issue", "Workaround paragraph",
                     "Nested workaround step", "use_this()", "Removed issue"):
        assert expected in rendered

    _write(release_folder / "change.rst", "- Updated release entry.\n")
    _write(issues_folder / "a-issue.rst", "- Updated issue.\n")
    (issues_folder / "b-issue.rst").unlink()
    _write(issues_folder / "c-issue.rst", "- Added issue.\n")
    rendered = build()
    for expected in ("Updated release entry", "Updated issue", "Added issue"):
        assert expected in rendered
    assert "Original issue" not in rendered
    assert "Removed issue" not in rendered

    for folder in (release_folder, issues_folder):
        for snippet in folder.glob("*.rst"):
            if not snippet.name.startswith("_"):
                snippet.unlink()
    rendered = build()
    for removed in ("Updated release entry", "Updated issue", "Added issue"):
        assert removed not in rendered


@pytest.fixture
def snippet_project(tmp_path):
    """Create a minimal Sphinx project with both snippet collections."""
    pytest.importorskip("sphinx")
    source = tmp_path / "source"
    for folder in COMPILER.SNIPPET_FOLDERS.values():
        (source / "Support" / folder).mkdir(parents=True)
    _write(source / "conf.py", (
        "import sys\n"
        f"sys.path.insert(0, {str(COMPILER_PATH.parent)!r})\n"
        "extensions = ['rst_snippets']\n"
        "exclude_patterns = ['Support/*Snippets/*']\n"
        "release = '2.13.0b0'\n"
    ))
    return source, tmp_path / "html"


def _build_snippet_project(source, output):
    """Run a full incremental HTML build and retain diagnostics for assertions."""
    return subprocess.run(
        [sys.executable, "-m", "sphinx", "-b", "html", "-W", "-q",
         str(source), str(output)],
        capture_output=True, text=True,
    )


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.parametrize("folder", COMPILER.SNIPPET_FOLDERS.values())
def test_snippets_render_as_one_compact_list(snippet_project, folder):
    """Render adjacent snippets as one compact list in filename order."""
    source, output = snippet_project
    snippets = source / "Support" / folder
    _write(source / "index.rst", (
        "Entries\n=======\n\n"
        f".. include:: Support/{folder}/_compiled_latest.rst\n"
    ))
    for filename, entry in (
        ("a.rst", "First entry."),
        ("b.rst", "Second entry."),
        ("c.rst", "Third entry."),
    ):
        _write(snippets / filename, f"- {entry}\n")

    result = _build_snippet_project(source, output)
    assert result.returncode == 0, result.stdout + result.stderr
    rendered = (output / "index.html").read_text(encoding="utf-8")
    section = rendered.split('<section id="entries">', 1)[1].split('</section>', 1)[0]
    assert section.count('<ul class="simple">') == 1
    assert section.count("<li>") == 3
    assert section.index("First entry.") < section.index("Second entry.")
    assert section.index("Second entry.") < section.index("Third entry.")


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
@pytest.mark.parametrize("folder", COMPILER.SNIPPET_FOLDERS.values())
def test_sphinx_validates_snippets_across_release_cycles(snippet_project, folder):
    """Preserve bookmarks across tags, patches, and a new development cycle."""
    source, output = snippet_project
    snippets = source / "Support" / folder
    generated = snippets / "_compiled_latest.rst"
    include = f".. include:: ../{folder}/_compiled_latest.rst\n"
    prefix = (
        "bsk-release" if folder == "bskReleaseNotesSnippets" else "bsk-known-issues"
    )
    current_label = f".. _{prefix}-current:\n\n"
    title = "History\n=======\n\n"
    active = current_label + "Version |release|\n-----------------\n\n" + include + "\n"
    series_path = Path("Support") / folder.removesuffix("Snippets") / "2.13.rst"
    _write(source / "index.rst", (
        "Documentation\n=============\n\n.. toctree::\n\n"
        f"   {series_path.with_suffix('').as_posix()}\n\n"
        ".. container:: bsk-history-bookmarks\n\n"
        "   .. raw:: html\n\n      <div id=\"version-release\">\n\n"
        f"   :ref:`Current version <{prefix}-current>`\n\n"
        "   .. raw:: html\n\n      </div>\n"
    ))
    page = source / series_path
    _write(page, title + active)
    _write(snippets / "issue.rst", "- Archived issue.\n")
    result = _build_snippet_project(source, output)
    assert result.returncode == 0, result.stdout + result.stderr

    # Finalization removes the active include and consumes the authored snippet.
    # Examples and comments mentioning an include must not count as publication.
    archived = (
        f".. _{prefix}-2-13-0:\n\n"
        "Version 2.13.0\n--------------\n\n"
        + generated.read_text(encoding="utf-8")
        + "\n.. Reserved for the next development cycle:\n\n   " + include
        + "\n.. code-block:: rst\n\n   " + include
    )
    _write(page, title + current_label + archived)
    (snippets / "issue.rst").unlink()
    index = source / "index.rst"
    _write(index, index.read_text(encoding="utf-8") + (
        f"\n:ref:`Original release <{prefix}-2-13-0>`\n"
    ))
    result = _build_snippet_project(source, output)
    assert result.returncode == 0, result.stdout + result.stderr
    assert generated.read_text(encoding="utf-8") == ""
    assert "Archived issue." in (
        output / series_path.with_suffix(".html")
    ).read_text(encoding="utf-8")

    # A maintenance branch starts at the tag. Compile and consume only its
    # patch snippet, moving the current label and keeping the original label.
    patch_snippet = snippets / "patch.rst"
    _write(patch_snippet, "- Patch workaround.\n")
    COMPILER.compile_snippets(snippets)
    patch = (
        f".. _{prefix}-2-13-1:\n\n"
        "Version 2.13.1\n--------------\n\n"
        + generated.read_text(encoding="utf-8") + "\n"
    )
    _write(page, title + current_label + patch + archived)
    patch_snippet.unlink()
    result = _build_snippet_project(source, output)
    assert result.returncode == 0, result.stdout + result.stderr
    rendered = (output / series_path.with_suffix(".html")).read_text(encoding="utf-8")
    patch_section = rendered.split('<section id="version-2-13-1">')[1].split(
        '</section>'
    )[0]
    original_section = rendered.split('<section id="version-2-13-0">')[1].split(
        '</section>'
    )[0]
    assert f'id="{prefix}-current"' in patch_section
    assert f'id="{prefix}-2-13-1"' in patch_section
    assert "Patch workaround." in patch_section
    assert f'id="{prefix}-2-13-0"' in original_section
    assert f'id="{prefix}-current"' not in original_section
    index_html = (output / "index.html").read_text(encoding="utf-8")
    bookmark = index_html.split('<div id="version-release">')[1].split('</div>')[0]
    html_path = series_path.with_suffix(".html").as_posix()
    assert f'href="{html_path}#{prefix}-current"' in bookmark
    assert f'href="{html_path}#{prefix}-2-13-0"' in index_html
    assert generated.read_text(encoding="utf-8") == ""
    archived = patch + archived

    # Only an excluded snippet changes: validation must run even when no page
    # is outdated, and must keep failing on a repeated build with no changes.
    _write(snippets / "next-issue.rst", "- New beta issue.\n")
    for _ in range(2):
        result = _build_snippet_project(source, output)
        assert result.returncode != 0
        assert "Nonempty snippet collections are not included" in result.stderr
        assert f"Support/{folder}/_compiled_latest.rst" in result.stderr

    config = source / "conf.py"
    _write(config, config.read_text(encoding="utf-8").replace("2.13.0b0", "2.13.2b0"))
    _write(page, title + active + archived)
    result = _build_snippet_project(source, output)
    assert result.returncode == 0, result.stdout + result.stderr
    rendered = (output / series_path.with_suffix(".html")).read_text(encoding="utf-8")
    assert rendered.count("Archived issue.") == 1
    assert rendered.count("Patch workaround.") == 1
    assert rendered.count("New beta issue.") == 1
    beta_section = rendered.split('<section id="version-release">')[1].split(
        '</section>'
    )[0]
    assert "Version 2.13.2b0" in beta_section
    assert f'id="{prefix}-current"' in beta_section
    assert rendered.count(f'id="{prefix}-current"') == 1
    assert f'id="{prefix}-2-13-0"' in rendered
    assert f'id="{prefix}-2-13-1"' in rendered


@pytest.mark.ciSkip
@pytest.mark.docsIntegration
def test_single_page_preview_skips_collection_validation(snippet_project):
    """Compile both collections during a preview, then re-enable full-build validation."""
    source, output = snippet_project
    for folder in COMPILER.SNIPPET_FOLDERS.values():
        _write(source / "Support" / folder / "issue.rst", "- Unpublished issue.\n")
    _write(source / "preview.rst", "Preview\n=======\n\nAn unrelated page.\n")
    config = source / "conf.py"
    full_config = config.read_text(encoding="utf-8") + "root_doc = 'preview'\n"
    _write(config, full_config + (
        "include_patterns = ['preview.rst']\n"
        "bsk_snippets_check_includes = False\n"
    ))

    result = _build_snippet_project(source, output)
    assert result.returncode == 0, result.stdout + result.stderr
    for folder in COMPILER.SNIPPET_FOLDERS.values():
        generated = source / "Support" / folder / "_compiled_latest.rst"
        assert generated.read_text(encoding="utf-8") == "- Unpublished issue.\n"

    _write(config, full_config)
    result = _build_snippet_project(source, output)
    assert result.returncode != 0
    for folder in COMPILER.SNIPPET_FOLDERS.values():
        assert f"Support/{folder}/_compiled_latest.rst" in result.stderr

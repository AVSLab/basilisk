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

"""Compile RST snippets for release notes and known issues before Sphinx reads them."""

import argparse
from pathlib import Path


SNIPPET_FOLDERS = {
    "release-notes": "bskReleaseNotesSnippets",
    "known-issues": "bskKnownIssuesSnippets",
}


def compile_snippets(snippets_dir: Path) -> Path:
    """Join snippets with blank lines, preserving their internal RST structure.

    :param snippets_dir: Directory containing authored RST snippet files.
    :return: Path to the generated include, written only when its content changes.
    """
    snippet_files = sorted(
        (
            path for path in snippets_dir.iterdir()
            if path.is_file()
            and path.suffix.lower() == ".rst"
            and not path.name.startswith("_")
        ),
        key=lambda path: (path.name.lower(), path.name),
    )
    fragments = []
    for snippet in snippet_files:
        content = snippet.read_text(encoding="utf-8").strip()
        if content:
            fragments.append(content)

    output = "\n\n".join(fragments) + "\n" if fragments else ""
    output_file = snippets_dir / "_compiled_latest.rst"
    if output_file.is_file() and output_file.read_text(encoding="utf-8") == output:
        print(f"Unchanged {output_file}")
    else:
        output_file.write_text(output, encoding="utf-8")
        print(f"Wrote {output_file}")
    print(f"Included {len(fragments)} nonempty snippet file(s)")
    return output_file


def compile_collections(source_dir: Path, collection: str = "all") -> None:
    """Compile the selected collection, or both collections, below a Sphinx source root."""
    names = SNIPPET_FOLDERS if collection == "all" else (collection,)
    for name in names:
        compile_snippets(source_dir / "Support" / SNIPPET_FOLDERS[name])


def _compile_before_reading(app) -> None:
    """Refresh includes before Sphinx checks dependencies and reads source files."""
    compile_collections(Path(app.srcdir))


def _validate_collection_includes(app, env) -> None:
    """Reject full builds that leave nonempty snippet collections unpublished."""
    if not app.config.bsk_snippets_check_includes:
        return

    included = {
        included_doc
        for docname in env.found_docs
        for included_doc in env.included.get(docname, ())
    }
    missing = []
    for folder in SNIPPET_FOLDERS.values():
        relative_path = Path("Support") / folder / "_compiled_latest.rst"
        output_file = Path(app.srcdir) / relative_path
        if (
            output_file.read_text(encoding="utf-8").strip()
            and relative_path.with_suffix("").as_posix() not in included
        ):
            missing.append(relative_path.as_posix())

    if missing:
        # Keep the standalone compiler usable without Sphinx installed.
        from sphinx.errors import ExtensionError

        raise ExtensionError(
            "Nonempty snippet collections are not included in the documentation: "
            + ", ".join(missing)
            + ". Restore their active includes for the development cycle, or "
            "archive their reviewed content and remove only the consumed snippets."
        )


def setup(app):
    """Generate snippets for all builders, including direct and single-page builds."""
    app.add_config_value("bsk_snippets_check_includes", True, "env")
    app.connect("builder-inited", _compile_before_reading)
    # This runs even when adding an excluded snippet leaves every page unchanged.
    app.connect("env-updated", _validate_collection_includes)
    return {"parallel_read_safe": True, "parallel_write_safe": True}


def main() -> None:
    """Compile snippets without requiring Sphinx or a Basilisk installation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "collection", nargs="?", default="all",
        choices=("all", *SNIPPET_FOLDERS),
    )
    args = parser.parse_args()
    compile_collections(Path(__file__).resolve().parents[1], args.collection)


if __name__ == "__main__":
    main()

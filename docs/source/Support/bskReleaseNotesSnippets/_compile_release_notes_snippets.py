#!/usr/bin/env python3
"""Compile release note snippet files into _compiled_latest.rst."""

from pathlib import Path


SNIPPETS_DIR = Path(__file__).resolve().parent
OUTPUT_FILE = SNIPPETS_DIR / "_compiled_latest.rst"
SKIP_NAMES = {"README.md", OUTPUT_FILE.name}


def _is_snippet(path: Path) -> bool:
    """
        Return True if the path is a valid .rst snippet file.
    """
    try:
        return (
                path.is_file()
                and path.name not in SKIP_NAMES
                and not path.name.startswith("_")
                and path.suffix.lower() == ".rst"
        )
    except OSError:
        return False


def main() -> None:
    snippet_files = sorted(
        (p for p in SNIPPETS_DIR.iterdir() if _is_snippet(p)),
        key=lambda p: p.name.lower(),
    )
    compiled_lines = []
    for snippet in snippet_files:
        text = snippet.read_text(encoding="utf-8").strip()
        if not text:
            continue
        compiled_lines.extend(text.splitlines())

    output = "\n".join(compiled_lines).rstrip() if compiled_lines else ""
    OUTPUT_FILE.write_text(output, encoding="utf-8")
    print(f"Wrote {OUTPUT_FILE}")
    print(f"Included {len(snippet_files)} snippet file(s)")


if __name__ == "__main__":
    main()

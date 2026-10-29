#!/usr/bin/env python3
"""Keep the original release-note compilation command available."""

from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "_ext"))
from rst_snippets import compile_snippets


def main() -> None:
    """Compile only release notes using the shared RST snippet compiler."""
    compile_snippets(Path(__file__).resolve().parent)


if __name__ == "__main__":
    main()

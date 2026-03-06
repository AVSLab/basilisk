#!/usr/bin/env python3

#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

"""Validate commit message prefix for the ``commit-msg`` pre-commit hook.

The first non-empty, non-comment line in the commit message must start with
``[#<number>]``.
"""

from pathlib import Path
import re
import sys


COMMIT_PREFIX_PATTERN = re.compile(r"^\[#\d+\](?:\s|$)")


def get_commit_subject(commit_message_path: Path) -> str:
    """Return the first non-empty, non-comment line of a commit message."""
    with commit_message_path.open("r", encoding="utf-8") as commit_file:
        for raw_line in commit_file:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            return line
    return ""


def main() -> int:
    """Run the commit-message prefix validation."""
    if len(sys.argv) != 2:
        print("Error: expected a single commit message file argument.")
        return 2

    commit_message_path = Path(sys.argv[1])
    subject = get_commit_subject(commit_message_path)
    if not subject:
        print("Error: commit message is empty.")
        return 1

    if COMMIT_PREFIX_PATTERN.match(subject):
        return 0

    print("Error: commit message must start with [#xxxx].")
    print("Example: [#1234] Brief summary of the change")
    print(f"Found: {subject}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

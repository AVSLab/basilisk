#!/usr/bin/env python3
#
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

"""Generate the Rust third-party license report shipped with Basilisk.

The report is derived from the locked Cargo workspace dependency graph. Use
``--check`` in automation to verify that the committed report is current. A
normal local check skips cleanly when the pinned generator is unavailable;
CI uses ``--require-tool`` so that the same condition is an error there.
"""

from __future__ import annotations

import argparse
import difflib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


CARGO_ABOUT_VERSION = "0.9.1"
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
WORKSPACE_MANIFEST = REPOSITORY_ROOT / "src" / "Cargo.toml"
ABOUT_CONFIG = Path(__file__).resolve().parent / "licenses" / "about.toml"
LICENSE_REPORT = REPOSITORY_ROOT / "LICENSES" / "RUST-THIRD-PARTY.txt"


def find_cargo_about(require_tool: bool) -> str | None:
    """Return the pinned ``cargo-about`` executable, if available."""
    executable = shutil.which("cargo-about")
    if executable is None:
        message = (
            "cargo-about is unavailable; install it with "
            "`cargo install cargo-about "
            f"--version '={CARGO_ABOUT_VERSION}' --locked --features cli`"
        )
        if require_tool:
            raise RuntimeError(message)
        print(f"Skipping Rust license check: {message}.")
        return None

    result = subprocess.run(
        [executable, "--version"],
        check=True,
        capture_output=True,
        text=True,
    )
    expected = f"cargo-about {CARGO_ABOUT_VERSION}"
    if result.stdout.strip() != expected:
        raise RuntimeError(
            f"expected {expected}, found {result.stdout.strip() or 'unknown version'}"
        )
    return executable


def cargo_about_data(executable: str) -> dict[str, Any]:
    """Return cargo-about's JSON report for the locked workspace."""
    with tempfile.TemporaryDirectory(prefix="bsk-rust-licenses-") as directory:
        output_path = Path(directory) / "licenses.json"
        subprocess.run(
            [
                executable,
                "generate",
                "--workspace",
                "--locked",
                "--offline",
                "--fail",
                "--manifest-path",
                str(WORKSPACE_MANIFEST),
                "--config",
                str(ABOUT_CONFIG),
                "--format",
                "json",
                "--output-file",
                str(output_path),
            ],
            check=True,
            cwd=REPOSITORY_ROOT,
        )
        return json.loads(output_path.read_text(encoding="utf-8"))


def third_party_licenses(data: dict[str, Any]) -> list[dict[str, Any]]:
    """Remove Basilisk workspace crates from cargo-about license entries."""
    entries = []
    for license_entry in data["licenses"]:
        entry = dict(license_entry)
        entry["used_by"] = [
            item for item in entry["used_by"] if item["crate"].get("source") is not None
        ]
        if entry["used_by"]:
            entries.append(entry)
    return entries


def dependency_notices(entries: list[dict[str, Any]]) -> list[tuple[str, str]]:
    """Collect top-level third-party ``NOTICE`` files from reported crates."""
    packages: dict[str, dict[str, Any]] = {}
    for entry in entries:
        for item in entry["used_by"]:
            package = item["crate"]
            packages[f"{package['name']} {package['version']}"] = package

    notices = []
    for package_name, package in sorted(packages.items()):
        package_root = Path(package["manifest_path"]).parent
        for path in sorted(package_root.iterdir()):
            if path.is_file() and path.name.upper().startswith("NOTICE"):
                notices.append((f"{package_name}: {path.name}", path.read_text(encoding="utf-8")))
    return notices


def normalize_newlines(text: str) -> str:
    """Return ``text`` with platform-independent line endings."""
    return text.replace("\r\n", "\n").replace("\r", "\n")


def render_report(data: dict[str, Any]) -> str:
    """Render a deterministic, human-readable third-party license report."""
    entries = third_party_licenses(data)
    lines = [
        "Basilisk Rust Third-Party Licenses",
        "===================================",
        "",
        "This file is generated from src/Cargo.lock by",
        "src/architecture/rust/generate_rust_licenses.py. Do not edit it by hand.",
        "",
        "It covers non-development Rust dependencies used to build or included in",
        "Basilisk's native Rust modules. Basilisk itself remains licensed under the",
        "ISC License in the repository's LICENSE file.",
        "",
    ]

    for entry in entries:
        heading = f"{entry['name']} ({entry['id']})"
        lines.extend([heading, "-" * len(heading), "", "Used by:"])
        for item in entry["used_by"]:
            package = item["crate"]
            lines.append(f"- {package['name']} {package['version']}")
        lines.extend(["", normalize_newlines(entry["text"]).rstrip(), ""])

    notices = dependency_notices(entries)
    if notices:
        lines.extend(["Third-Party Notices", "-------------------", ""])
        for heading, notice in notices:
            lines.extend(
                [heading, "~" * len(heading), "", normalize_newlines(notice).rstrip(), ""]
            )

    return "\n".join(lines).rstrip() + "\n"


def check_report(expected: str) -> bool:
    """Return whether the committed report matches ``expected``."""
    actual = LICENSE_REPORT.read_text(encoding="utf-8") if LICENSE_REPORT.exists() else ""
    if actual == expected:
        print(f"Rust third-party license report is current: {LICENSE_REPORT}")
        return True

    difference = difflib.unified_diff(
        actual.splitlines(),
        expected.splitlines(),
        fromfile=str(LICENSE_REPORT),
        tofile="generated report",
        lineterm="",
    )
    print("\n".join(difference))
    print(
        "Rust third-party license report is stale; run "
        "`python src/architecture/rust/generate_rust_licenses.py`.",
        file=sys.stderr,
    )
    return False


def main() -> int:
    """Generate or check the committed Rust third-party license report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify rather than update the report")
    parser.add_argument(
        "--require-tool",
        action="store_true",
        help="fail instead of skipping when cargo-about is unavailable",
    )
    arguments = parser.parse_args()

    try:
        executable = find_cargo_about(arguments.require_tool)
        if executable is None:
            return 0
        report = render_report(cargo_about_data(executable))
    except (
        OSError,
        RuntimeError,
        UnicodeError,
        subprocess.CalledProcessError,
        json.JSONDecodeError,
    ) as error:
        print(f"Rust license generation failed: {error}", file=sys.stderr)
        return 1

    if arguments.check:
        return 0 if check_report(report) else 1

    LICENSE_REPORT.parent.mkdir(parents=True, exist_ok=True)
    LICENSE_REPORT.write_text(report, encoding="utf-8")
    print(f"Generated {LICENSE_REPORT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

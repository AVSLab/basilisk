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

"""Generate a deterministic Rust third-party license report.

Defaults produce the report shipped with Basilisk. SDK extension projects can
override the manifest, policy, output, project name, and project-license text.
The report is derived from the locked Cargo workspace dependency graph. Use
``--check`` in automation to verify that the committed report is current. A
normal local check skips cleanly when the pinned generator is unavailable; CI
uses ``--require-tool`` so that the same condition is an error there.
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
SCRIPT_DIRECTORY = Path(__file__).resolve().parent
ABOUT_CONFIG = SCRIPT_DIRECTORY / "licenses" / "about.toml"
if not ABOUT_CONFIG.is_file():
    # bsk-sdk installs the synchronized generator beside its policy file.
    ABOUT_CONFIG = SCRIPT_DIRECTORY / "about.toml"
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


def cargo_about_data(
    executable: str, workspace_manifest: Path, about_config: Path
) -> dict[str, Any]:
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
                str(workspace_manifest),
                "--config",
                str(about_config),
                "--format",
                "json",
                "--output-file",
                str(output_path),
            ],
            check=True,
            cwd=workspace_manifest.parent,
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


def render_report(
    data: dict[str, Any],
    project_name: str = "Basilisk",
    manifest_label: str = "src/Cargo.lock",
    generator_label: str = "src/architecture/rust/generate_rust_licenses.py",
    project_license: str = "ISC License in the repository's LICENSE file.",
) -> str:
    """Render a deterministic, human-readable third-party license report."""
    entries = third_party_licenses(data)
    title = f"{project_name} Rust Third-Party Licenses"
    underline_length = 35 if project_name == "Basilisk" else len(title)
    lines = [
        title,
        "=" * underline_length,
        "",
        f"This file is generated from {manifest_label} by",
        f"{generator_label}. Do not edit it by hand.",
        "",
        "It covers non-development Rust dependencies used to build or included in",
        (
            f"{project_name}'s native Rust modules. {project_name} itself remains "
            "licensed under the"
        ),
        project_license,
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


def check_report(expected: str, license_report: Path = LICENSE_REPORT) -> bool:
    """Return whether the committed report matches ``expected``."""
    actual = license_report.read_text(encoding="utf-8") if license_report.exists() else ""
    if actual == expected:
        print(f"Rust third-party license report is current: {license_report}")
        return True

    difference = difflib.unified_diff(
        actual.splitlines(),
        expected.splitlines(),
        fromfile=str(license_report),
        tofile="generated report",
        lineterm="",
    )
    print("\n".join(difference))
    print(
        "Rust third-party license report is stale; run "
        "the Rust license generator without `--check`.",
        file=sys.stderr,
    )
    return False


def main() -> int:
    """Generate or check the committed Rust third-party license report."""
    parser = argparse.ArgumentParser(description=__doc__)
    has_basilisk_defaults = WORKSPACE_MANIFEST.is_file()
    parser.add_argument("--check", action="store_true", help="verify rather than update the report")
    parser.add_argument(
        "--require-tool",
        action="store_true",
        help="fail instead of skipping when cargo-about is unavailable",
    )
    parser.add_argument(
        "--manifest-path",
        type=Path,
        default=WORKSPACE_MANIFEST if has_basilisk_defaults else None,
        required=not has_basilisk_defaults,
        help=(
            "Cargo workspace manifest to inspect; required outside the "
            "Basilisk source tree"
        ),
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=ABOUT_CONFIG,
        help="cargo-about policy file",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=LICENSE_REPORT if has_basilisk_defaults else None,
        required=not has_basilisk_defaults,
        help=(
            "third-party license report path; required outside the Basilisk "
            "source tree"
        ),
    )
    parser.add_argument(
        "--project-name",
        default="Basilisk",
        help="project name displayed in the report",
    )
    parser.add_argument(
        "--project-license",
        default="ISC License in the repository's LICENSE file.",
        help="project-license sentence displayed after the dependency overview",
    )
    parser.add_argument(
        "--manifest-label",
        default=None,
        help="stable Cargo.lock label written into the report",
    )
    parser.add_argument(
        "--generator-label",
        default=None,
        help="stable generator label written into the report",
    )
    arguments = parser.parse_args()

    try:
        executable = find_cargo_about(arguments.require_tool)
        if executable is None:
            return 0
        workspace_manifest = arguments.manifest_path.resolve()
        about_config = arguments.config.resolve()
        output = arguments.output.resolve()
        using_basilisk_defaults = (
            workspace_manifest == WORKSPACE_MANIFEST.resolve()
            and about_config == ABOUT_CONFIG.resolve()
            and output == LICENSE_REPORT.resolve()
            and arguments.project_name == "Basilisk"
        )
        report = render_report(
            cargo_about_data(executable, workspace_manifest, about_config),
            project_name=arguments.project_name,
            project_license=arguments.project_license,
            manifest_label=(
                arguments.manifest_label
                or ("src/Cargo.lock" if using_basilisk_defaults else "Cargo.lock")
            ),
            generator_label=(
                arguments.generator_label
                or (
                    "src/architecture/rust/generate_rust_licenses.py"
                    if using_basilisk_defaults
                    else "the bsk-sdk Rust license generator"
                )
            ),
        )
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
        return 0 if check_report(report, output) else 1

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(report, encoding="utf-8")
    print(f"Generated {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

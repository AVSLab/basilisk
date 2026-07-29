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
#

"""Test reusable Rust third-party license report generation."""

import importlib.util
from pathlib import Path
import sys

import pytest


GENERATOR_PATH = Path(__file__).resolve().parents[1] / "generate_rust_licenses.py"
GENERATOR_SPEC = importlib.util.spec_from_file_location(
    "generate_rust_licenses",
    GENERATOR_PATH,
)
assert GENERATOR_SPEC is not None
assert GENERATOR_SPEC.loader is not None
generate_rust_licenses = importlib.util.module_from_spec(GENERATOR_SPEC)
GENERATOR_SPEC.loader.exec_module(generate_rust_licenses)


def test_custom_cli_arguments_render_extension_report(tmp_path, monkeypatch) -> None:
    """Apply custom input paths and project labels to a generated report."""
    workspace_manifest = tmp_path / "workspace" / "Cargo.toml"
    workspace_manifest.parent.mkdir()
    workspace_manifest.write_text("[workspace]\n", encoding="utf-8")
    about_config = tmp_path / "about.toml"
    about_config.write_text("accepted = []\n", encoding="utf-8")
    dependency_manifest = tmp_path / "dependency" / "Cargo.toml"
    dependency_manifest.parent.mkdir()
    dependency_manifest.write_text("[package]\n", encoding="utf-8")
    output = tmp_path / "package" / "RUST-THIRD-PARTY.txt"

    license_data = {
        "licenses": [
            {
                "id": "MIT",
                "name": "MIT License",
                "text": "Example license text.\n",
                "used_by": [
                    {
                        "crate": {
                            "name": "example-dependency",
                            "version": "1.2.3",
                            "source": "registry+https://example.invalid/index",
                            "manifest_path": str(dependency_manifest),
                        }
                    }
                ],
            }
        ]
    }
    observed = {}

    def fake_find_cargo_about(require_tool):
        observed["require_tool"] = require_tool
        return "/test/bin/cargo-about"

    def fake_cargo_about_data(executable, manifest, config):
        observed["cargo_about"] = (executable, manifest, config)
        return license_data

    monkeypatch.setattr(
        generate_rust_licenses,
        "find_cargo_about",
        fake_find_cargo_about,
    )
    monkeypatch.setattr(
        generate_rust_licenses,
        "cargo_about_data",
        fake_cargo_about_data,
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            str(GENERATOR_PATH),
            "--require-tool",
            "--manifest-path",
            str(workspace_manifest),
            "--config",
            str(about_config),
            "--output",
            str(output),
            "--project-name",
            "Example Extension",
            "--project-license",
            "BSD-3-Clause License in the package LICENSE file.",
            "--manifest-label",
            "extension/Cargo.lock",
            "--generator-label",
            "tools/generate_licenses.py",
        ],
    )

    assert generate_rust_licenses.main() == 0
    assert observed["require_tool"] is True
    assert observed["cargo_about"] == (
        "/test/bin/cargo-about",
        workspace_manifest.resolve(),
        about_config.resolve(),
    )

    report = output.read_text(encoding="utf-8")
    assert "Example Extension Rust Third-Party Licenses\n" in report
    assert "This file is generated from extension/Cargo.lock by\n" in report
    assert "tools/generate_licenses.py. Do not edit it by hand.\n" in report
    assert "BSD-3-Clause License in the package LICENSE file.\n" in report
    assert "- example-dependency 1.2.3\n" in report
    assert "Example license text.\n" in report


def test_relocated_generator_requires_project_paths(
    tmp_path, monkeypatch, capsys
) -> None:
    """Reject ambiguous defaults when the generator is installed by bsk-sdk."""
    monkeypatch.setattr(
        generate_rust_licenses,
        "WORKSPACE_MANIFEST",
        tmp_path / "missing" / "Cargo.toml",
    )
    monkeypatch.setattr(sys, "argv", [str(GENERATOR_PATH)])

    with pytest.raises(SystemExit, match="2"):
        generate_rust_licenses.main()

    error = capsys.readouterr().err
    assert "--manifest-path" in error
    assert "--output" in error

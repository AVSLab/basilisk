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

"""Verify the pinned Rust utility-binding generator dependency."""

import importlib.util
from pathlib import Path
import subprocess

import pytest


GENERATOR_PATH = Path(__file__).resolve().parents[1] / "gen_rust_utilities.py"
GENERATOR_SPEC = importlib.util.spec_from_file_location(
    "gen_rust_utilities",
    GENERATOR_PATH,
)
assert GENERATOR_SPEC is not None
assert GENERATOR_SPEC.loader is not None
gen_rust_utilities = importlib.util.module_from_spec(GENERATOR_SPEC)
GENERATOR_SPEC.loader.exec_module(gen_rust_utilities)


def test_bindgen_pin_matches_library_and_ci_installation():
    """Keep the generator, Rust library, and CI command on one bindgen version."""
    message_manifest = (
        gen_rust_utilities.RUST_SUPPORT_ROOT / "bsk_messages" / "Cargo.toml"
    ).read_text(encoding="utf-8")
    expected_dependency = (
        f'bindgen = {{ version = "{gen_rust_utilities.BINDGEN_CLI_VERSION}"'
    )
    assert expected_dependency in message_manifest

    pull_request_workflow = (
        gen_rust_utilities.REPO_ROOT / ".github" / "workflows" / "pull-request.yml"
    ).read_text(encoding="utf-8")
    assert gen_rust_utilities.BINDGEN_INSTALL_COMMAND in pull_request_workflow


def _mock_version_result(version):
    """Return a successful ``bindgen --version`` subprocess result."""
    return subprocess.CompletedProcess(
        args=["bindgen", "--version"],
        returncode=0,
        stdout=f"bindgen {version}\n",
        stderr="",
    )


def test_find_bindgen_accepts_only_pinned_version(monkeypatch):
    """Accept the executable when its reported version matches the pin."""
    executable = "/test/bin/bindgen"
    monkeypatch.setattr(gen_rust_utilities.shutil, "which", lambda _name: executable)
    monkeypatch.setattr(
        gen_rust_utilities.subprocess,
        "run",
        lambda *_args, **_kwargs: _mock_version_result(
            gen_rust_utilities.BINDGEN_CLI_VERSION
        ),
    )

    assert gen_rust_utilities.find_bindgen() == executable


def test_find_bindgen_rejects_mismatched_version(monkeypatch):
    """Reject an installed CLI whose version differs from the required pin."""
    monkeypatch.setattr(
        gen_rust_utilities.shutil,
        "which",
        lambda _name: "/test/bin/bindgen",
    )
    monkeypatch.setattr(
        gen_rust_utilities.subprocess,
        "run",
        lambda *_args, **_kwargs: _mock_version_result("0.71.1"),
    )

    with pytest.raises(SystemExit, match="expected bindgen 0.72.1"):
        gen_rust_utilities.find_bindgen()


def test_find_bindgen_reports_pinned_install_command(monkeypatch):
    """Report the exact pinned installation command when the CLI is missing."""
    monkeypatch.setattr(gen_rust_utilities.shutil, "which", lambda _name: None)

    with pytest.raises(SystemExit, match="bindgen-cli --version '=0.72.1' --locked"):
        gen_rust_utilities.find_bindgen()

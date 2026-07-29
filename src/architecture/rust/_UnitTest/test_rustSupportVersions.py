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

"""Check that independently consumed Rust support versions remain aligned."""

from __future__ import annotations

import re
from pathlib import Path


SRC_ROOT = Path(__file__).resolve().parents[3]


def _required_match(pattern: str, text: str, description: str) -> str:
    """Return the first capture from ``pattern`` or fail with useful context."""
    match = re.search(pattern, text, re.MULTILINE)
    assert match is not None, f"Could not find {description}"
    return match.group(1)


def test_rust_support_versions_are_consistent() -> None:
    """Keep the workspace, support crates, and generated ABI expectation aligned."""
    version_file = (
        SRC_ROOT / "cmake" / "bskRustSupportVersions.cmake"
    ).read_text(encoding="utf-8")
    workspace = (SRC_ROOT / "Cargo.toml").read_text(encoding="utf-8")

    minimum_version = _required_match(
        r'set\(BSK_RUST_MIN_VERSION "([^"]+)"\)',
        version_file,
        "the synchronized Rust minimum version",
    )
    support_version = _required_match(
        r'set\(BSK_RUST_SUPPORT_CRATE_VERSION "([^"]+)"\)',
        version_file,
        "the synchronized Rust support-crate version",
    )
    workspace_minimum = _required_match(
        r'^rust-version\s*=\s*"([^"]+)"',
        workspace,
        "workspace.package.rust-version",
    )
    assert workspace_minimum == minimum_version

    crate_manifests = {}
    for crate in ("bsk_build", "bsk_macros", "bsk_messages", "bsk_utilities"):
        manifest = (
            SRC_ROOT / "architecture" / "rust" / crate / "Cargo.toml"
        ).read_text(encoding="utf-8")
        crate_manifests[crate] = manifest
        crate_version = _required_match(
            r'^version\s*=\s*"([^"]+)"', manifest, f"the {crate} package version"
        )
        assert crate_version == support_version

    for crate, dependency in (
        ("bsk_build", "bsk-macros"),
        ("bsk_messages", "bsk-build"),
    ):
        dependency_version = _required_match(
            rf'^{dependency}\s*=\s*\{{[^\n]*version\s*=\s*"=([^"]+)"',
            crate_manifests[crate],
            f"the exact {dependency} dependency in {crate}",
        )
        assert dependency_version == support_version

    runtime_header = (
        SRC_ROOT / "architecture" / "_GeneralModuleFiles" / "bsk_rust_module.h"
    ).read_text(encoding="utf-8")
    code_generator = (
        SRC_ROOT / "architecture" / "rust" / "bsk_build" / "src" / "codegen" / "mod.rs"
    ).read_text(encoding="utf-8")
    runtime_abi = _required_match(
        r'^#define BSK_RUST_MODULE_ABI_VERSION\s+(\d+)',
        runtime_header,
        "the runtime Rust-module ABI version",
    )
    generator_abi = _required_match(
        r'^const RUST_MODULE_ABI_VERSION: u32 = (\d+);',
        code_generator,
        "the bsk-build ABI expectation",
    )
    assert generator_abi == runtime_abi

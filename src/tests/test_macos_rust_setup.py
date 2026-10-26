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

"""Test macOS Rust setup without downloading or modifying user toolchains."""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[2] / ".github/scripts/setup_macos_rust.py"


@pytest.fixture
def rust_setup(tmp_path, monkeypatch):
    """Model an incomplete toolchain and retain every subprocess invocation."""
    spec = importlib.util.spec_from_file_location("setup_macos_rust", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(sys, "platform", "darwin")
    sysroot = tmp_path / "toolchain with spaces"
    compiler = sysroot / "bin" / "rustc"
    state = {"installed": False, "calls": [], "failure": None}

    def fake_run(command, **kwargs):
        state["calls"].append((command, kwargs))
        stdout = ""
        stderr = ""
        status = 0
        toolchain = os.environ.get("RUSTUP_TOOLCHAIN", "stable-aarch64-apple-darwin")
        if command[:3] == ["rustup", "show", "active-toolchain"]:
            stdout = toolchain + " (default)\n"
        elif command[:2] == ["rustup", "which"]:
            assert command[2:] == ["--toolchain", toolchain, "rustc"]
            stdout = str(compiler) + "\n"
        elif command[:3] == ["rustup", "component", "add"]:
            assert command[3:] == ["llvm-tools-preview", "--toolchain", toolchain]
            if state["failure"] == "install":
                status, stderr = 1, "component download failed"
            else:
                state["installed"] = True
        elif command == [str(compiler), "--print", "sysroot"]:
            stdout = str(sysroot) + "\n"
        elif command == [str(compiler), "--version", "--verbose"]:
            stdout = "rustc 1.99.1\nhost: aarch64-apple-darwin\n"
        elif Path(command[0]).name == "rust-objcopy":
            if not state["installed"]:
                status, stderr = -6, "Library not loaded: @rpath/libLLVM.dylib"
        elif command[0] == str(compiler):
            if state["failure"] == "strip":
                # The real compiler returns success after this warning.
                stderr = (
                    "warning: stripping debug info with `rust-objcopy` failed: "
                    "signal: 6 (SIGABRT)\n"
                )
            elif state["failure"] == "compile":
                status, stderr = 1, "error: linking failed"
        elif Path(command[0]).name == "probe":
            if state["failure"] == "execute":
                status, stderr = 1, "probe execution failed"
        else:
            pytest.fail(f"Unexpected command: {command}")
        if status and kwargs["check"]:
            raise subprocess.CalledProcessError(status, command, output=stdout, stderr=stderr)
        return subprocess.CompletedProcess(command, status, stdout, stderr)

    monkeypatch.setattr(module.subprocess, "run", fake_run)
    monkeypatch.delenv("RUSTUP_TOOLCHAIN", raising=False)
    return module, state, compiler


@pytest.mark.parametrize("toolchain", [None, "1.99.1-aarch64-apple-darwin"])
def test_setup_preserves_selected_toolchain(rust_setup, monkeypatch, toolchain):
    """Prepare the active default or wheel version without changing the selection."""
    module, state, compiler = rust_setup
    if toolchain:
        monkeypatch.setenv("RUSTUP_TOOLCHAIN", toolchain)
    monkeypatch.setenv("DYLD_LIBRARY_PATH", "/unrelated/library")
    monkeypatch.setenv("DYLD_FALLBACK_LIBRARY_PATH", "/rustup/proxy/library")
    environment_before = os.environ.copy()
    for _ in range(2):
        assert module.main([]) == 0
    assert os.environ == environment_before
    assert state["installed"]
    assert not any(command[:2] == ["rustup", "default"] for command, _ in state["calls"])
    compiles = [(command, options) for command, options in state["calls"]
                if command[0] == str(compiler) and "-Cstrip=debuginfo" in command]
    assert len(compiles) == 2
    for command, options in compiles:
        assert "-Copt-level=3" in command
        assert options["env"].get("RUSTUP_TOOLCHAIN") == toolchain
    for command, options in state["calls"]:
        if options["env"] is not None:
            assert "DYLD_LIBRARY_PATH" not in options["env"]
            assert "DYLD_FALLBACK_LIBRARY_PATH" not in options["env"]


def test_check_only_detects_incomplete_toolchain(rust_setup, capsys):
    """Checking must report the missing library without installing a workaround."""
    module, state, _ = rust_setup
    assert module.main(["--check-only"]) == 1
    assert "libLLVM.dylib" in capsys.readouterr().err
    assert not state["installed"]
    assert not any(command[:3] == ["rustup", "component", "add"] for command, _ in state["calls"])
    state["installed"] = True
    assert module.main(["--check-only"]) == 0


@pytest.mark.parametrize("failure, diagnostic", [
    ("install", "component download failed"),
    ("strip", "stripping debug info"),
    ("compile", "linking failed"),
    ("execute", "probe execution failed"),
])
def test_setup_fails_on_toolchain_errors(rust_setup, capsys, failure, diagnostic):
    """Propagate download, link, execution, and zero-status stripping failures."""
    module, state, _ = rust_setup
    state["failure"] = failure
    assert module.main([]) == 1
    captured = capsys.readouterr()
    assert diagnostic in captured.err
    assert "Verified Rust debug stripping" not in captured.out
    if failure == "install":
        assert not state["installed"]
        assert all(command[0] == "rustup" for command, _ in state["calls"])


@pytest.mark.parametrize("platform", ["linux", "win32"])
def test_other_platforms_do_not_modify_toolchains(rust_setup, monkeypatch, platform):
    """Linux and Windows need no component installation or macOS probe."""
    module, state, _ = rust_setup
    monkeypatch.setattr(sys, "platform", platform)
    assert module.main([]) == 0
    assert state["calls"] == []


@pytest.mark.parametrize("error", [
    FileNotFoundError("rustup is missing"),
    subprocess.TimeoutExpired(["rustup"], 600),  # [s]
])
def test_unavailable_toolchain_reports_failure(rust_setup, monkeypatch, capsys, error):
    """An unavailable or unresponsive tool must produce an actionable failure."""
    module, _, _ = rust_setup

    def fail(command, **kwargs):
        raise error

    monkeypatch.setattr(module.subprocess, "run", fail)
    assert module.main([]) == 1
    assert "Rust toolchain check failed" in capsys.readouterr().err

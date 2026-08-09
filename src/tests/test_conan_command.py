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
"""Tests for the Conan command used by Basilisk's build script."""

import argparse
import importlib
from pathlib import Path

import pytest


pytest.importorskip("conan")


@pytest.fixture(scope="module")
def conanfile_module(monkeypatch_module):
    """Import the repository's Conan recipe for command tests."""
    repo_root = Path(__file__).resolve().parents[2]
    monkeypatch_module.chdir(repo_root)
    monkeypatch_module.syspath_prepend(str(repo_root))
    return importlib.import_module("conanfile")


@pytest.fixture(scope="module")
def monkeypatch_module(request):
    """Provide module-scoped monkeypatching for the shared recipe import."""
    patch = pytest.MonkeyPatch()
    request.addfinalizer(patch.undo)
    return patch


def make_arguments(conanfile_module, **overrides):
    """Create build-script arguments using the recipe defaults."""
    values = {
        "buildType": "Release",
        "generator": None,
    }
    for option_group in (
            conanfile_module.bskModuleOptionsBool,
            conanfile_module.bskModuleOptionsString,
            conanfile_module.bskModuleOptionsFlag,
    ):
        values.update({name: definition[1] for name, definition in option_group.items()})
    values.update(overrides)
    return argparse.Namespace(**values)


def option_values(command, flag):
    """Return values paired with a repeated command-line flag."""
    return [command[index + 1] for index, value in enumerate(command) if value == flag]


def test_single_command_installs_missing_dependencies_and_builds(conanfile_module):
    """Use one Conan build command while retaining settings and Boolean options."""
    arguments = make_arguments(
        conanfile_module,
        buildType="Debug",
        generator="Ninja",
        clean=True,
        rustModules=True,
    )

    command = conanfile_module.create_conan_build_command(arguments, platform_name="posix")
    settings = option_values(command, "-s")
    build_settings = option_values(command, "-s:b")
    options = option_values(command, "-o")

    assert command[3:6] == ["build", ".", "--build=missing"]
    assert "install" not in command
    assert "build_type=Debug" in settings
    assert "compiler.cppstd=17" in settings
    assert "compiler.cppstd=17" in build_settings
    assert "compiler.cstd=gnu17" in settings
    assert "&:generator=Ninja" in options
    assert "&:clean=True" in options
    assert "&:rustModules=True" in options
    assert "&:buildTesting=True" in options


def test_offline_command_uses_only_cached_binary_packages(conanfile_module):
    """Disable Conan remotes and dependency builds in strict offline mode."""
    arguments = make_arguments(conanfile_module, offline=True)

    command = conanfile_module.create_conan_build_command(arguments)

    assert command[3:6] == ["build", ".", "--build=never"]
    assert "--build=missing" not in command
    assert "--no-remote" in command
    assert (
        f"{conanfile_module.OFFLINE_CONAN_CONF}=True"
        in option_values(command, "-c")
    )


def test_offline_environment_disables_cargo_network_access(conanfile_module):
    """Force every Cargo command spawned by Conan or CMake to remain offline."""
    original_environment = {"EXISTING_VARIABLE": "unchanged"}

    build_environment = conanfile_module.create_conan_build_environment(
        offline=True,
        environment=original_environment,
    )

    assert build_environment["CARGO_NET_OFFLINE"] == "true"
    assert build_environment["EXISTING_VARIABLE"] == "unchanged"
    assert "CARGO_NET_OFFLINE" not in original_environment


def test_online_environment_preserves_existing_cargo_policy(conanfile_module):
    """Do not override a developer's Cargo policy during a regular build."""
    original_environment = {"CARGO_NET_OFFLINE": "custom"}

    build_environment = conanfile_module.create_conan_build_environment(
        offline=False,
        environment=original_environment,
    )

    assert build_environment == original_environment


def test_native_tests_can_be_disabled(conanfile_module):
    """Forward the wheel-build setting that excludes native test targets."""
    arguments = make_arguments(conanfile_module, buildTesting=False)

    command = conanfile_module.create_conan_build_command(arguments)

    assert "&:buildTesting=False" in option_values(command, "-o")


def test_legacy_python_environment_options_are_removed(conanfile_module):
    """Keep Python package management outside the native build recipe."""
    option_names = {
        *conanfile_module.bskModuleOptionsBool,
        *conanfile_module.bskModuleOptionsString,
        *conanfile_module.bskModuleOptionsFlag,
    }

    assert option_names.isdisjoint({
        "managePipEnvironment",
        "autoKey",
        "allOptPkg",
        "pyPkgCanary",
        "examples",
    })


def test_windows_command_omits_c_language_standard(conanfile_module):
    """Retain the existing Windows command-line settings."""
    arguments = make_arguments(conanfile_module)

    command = conanfile_module.create_conan_build_command(arguments, platform_name="nt")

    assert "compiler.cstd=gnu17" not in option_values(command, "-s")


def test_external_module_path_is_normalized(conanfile_module, tmp_path):
    """Forward a validated absolute external-module path to Conan."""
    external_modules = tmp_path / "external-modules"
    external_modules.mkdir()
    arguments = make_arguments(
        conanfile_module,
        pathToExternalModules=str(external_modules),
    )

    command = conanfile_module.create_conan_build_command(arguments)

    assert f"&:pathToExternalModules={external_modules.resolve()}" in option_values(command, "-o")


def test_missing_external_module_path_is_rejected(conanfile_module, tmp_path):
    """Reject an external-module path before invoking Conan."""
    missing_path = tmp_path / "missing"
    arguments = make_arguments(
        conanfile_module,
        pathToExternalModules=str(missing_path),
    )

    with pytest.raises(ValueError, match="does not exist"):
        conanfile_module.create_conan_build_command(arguments)


def test_offline_mujoco_requires_a_cached_package(conanfile_module, monkeypatch):
    """Fail without launching a network-capable MuJoCo package build offline."""
    create_calls = []
    monkeypatch.setattr(
        conanfile_module,
        "is_conan_package_available",
        lambda reference: False,
    )
    monkeypatch.setattr(
        conanfile_module.subprocess,
        "run",
        lambda *args, **kwargs: create_calls.append((args, kwargs)),
    )

    with pytest.raises(RuntimeError, match=r"mujoco/.*local Conan cache"):
        conanfile_module.conan_create_mujoco(offline=True, print_fn=None)

    assert create_calls == []

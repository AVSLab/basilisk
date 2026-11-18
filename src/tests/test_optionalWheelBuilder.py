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

import importlib.util
import io
import os
import struct
import subprocess
import sys
import venv
import zipfile
from contextlib import contextmanager
from functools import cache
from importlib.metadata import distributions
from pathlib import Path
from types import ModuleType

import pytest

import Basilisk._buildInfo as buildInfoFormatter


REPO_ROOT = Path(__file__).parents[2]
WHEEL_BUILDER_PATH = REPO_ROOT / ".github/scripts/build_optional_bsk_wheel.py"
WHEEL_TESTER_PATH = REPO_ROOT / ".github/scripts/test_optional_bsk_wheels.py"


def _loadWheelBuilder():
    moduleName = "bskOptionalWheelBuilderUnderTest"
    spec = importlib.util.spec_from_file_location(moduleName, WHEEL_BUILDER_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[moduleName] = module
    spec.loader.exec_module(module)
    return module


def _loadWheelTester():
    moduleName = "bskOptionalWheelTesterUnderTest"
    spec = importlib.util.spec_from_file_location(moduleName, WHEEL_TESTER_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[moduleName] = module
    spec.loader.exec_module(module)
    return module


WHEEL_BUILDER = _loadWheelBuilder()
WHEEL_TESTER = _loadWheelTester()


@pytest.mark.parametrize("suffix", ["so", "pyd"])
def test_grouped_opnav_wheel_delta(suffix, monkeypatch):
    """Package grouped OpenCV bindings without accepting incomplete or mixed layouts.

    :param suffix: Native extension suffix for Unix or Windows wheels.
    :param monkeypatch: Pytest helper for reusing the unchanged source inventory.
    """
    # Discover the real modules once; each missing-file case uses the same tree.
    monkeypatch.setattr(
        WHEEL_BUILDER, "discover_cmake_include_modules",
        cache(WHEEL_BUILDER.discover_cmake_include_modules),
    )
    component = WHEEL_BUILDER.COMPONENTS["opnav"]
    base = {"Basilisk/__init__.py", "Basilisk/simulation/_simulationCoreNative." + suffix}
    delta = {"Basilisk/_opNavNative." + suffix, "Basilisk/_load_opnav.py"}
    for package, name in (("simulation", "camera"), ("fswAlgorithms", "centerRadiusCNN"),
                          ("fswAlgorithms", "houghCircles"), ("fswAlgorithms", "limbFinding")):
        delta.update({f"Basilisk/{package}/{name}.py", f"Basilisk/{package}/_{name}.py"})
    assert WHEEL_BUILDER.validate_delta(base, base | delta, component) == sorted(delta)
    for required in delta:
        with pytest.raises(ValueError):
            WHEEL_BUILDER.validate_delta(base, base | (delta - {required}), component)
    with pytest.raises(ValueError, match="unexpected"):
        WHEEL_BUILDER.validate_delta(base, base | delta | {"Basilisk/simulation/_camera." + suffix}, component)
    with pytest.raises(ValueError, match="Base wheel"):
        WHEEL_BUILDER.validate_delta(base | {"Basilisk/_load_opnav.py"}, base | delta, component)


def _buildInfoData(opNavEnabled, *, schemaVersion=4, diagnostics=None):
    buildInfo = {
        "schemaVersion": schemaVersion,
        "artifact": {"basiliskVersion": "test-version"},
        "features": {
            "vizInterface": True,
            "opNav": opNavEnabled,
            "mujoco": True,
        },
    }
    if diagnostics is not None:
        buildInfo["diagnostics"] = diagnostics
    return f"buildInfoData = {buildInfo!r}\n".encode("utf-8")


def _windowsBuildDiagnostics(
    msvcVersion,
    rustVersion,
    cargoVersion,
    compilerLauncher="",
):
    return {
        "target": {"system": "Windows", "processor": "AMD64"},
        "build": {"configuration": "Release", "generator": "Ninja"},
        "compilers": {
            "c": {
                "id": "MSVC",
                "version": msvcVersion,
                "launcher": compilerLauncher,
            },
            "cxx": {
                "id": "MSVC",
                "version": msvcVersion,
                "launcher": compilerLauncher,
                "standard": "17",
            },
            "rust": {
                "id": "rustc",
                "version": rustVersion,
                "target": "x86_64-pc-windows-msvc",
            },
        },
        "tools": {"cargo": cargoVersion, "cmake": "4.3.2"},
    }


def _windowsBuildAbiData(msvcFullVersion, patchVersion=0, stlUpdate=202604):
    compilerVersion = str(msvcFullVersion)
    msvcVersion = msvcFullVersion // 100000
    return (
        "buildAbiData = "
        + repr({
            "capture": "compiled",
            "build": {"configuration": "Release", "debugRuntime": False},
            "target": {"system": "Windows", "architecture": "x86_64"},
            "c": {
                "compiler": {
                    "id": "MSVC",
                    "version": compilerVersion,
                    "majorVersion": msvcVersion // 100,
                    "minorVersion": msvcVersion % 100,
                    "patchVersion": patchVersion,
                    "msvcVersion": msvcVersion,
                },
                "layoutCanaries": {"packingCanarySize": 12},
            },
            "cxx": {
                "compiler": {
                    "id": "MSVC",
                    "version": compilerVersion,
                    "majorVersion": msvcVersion // 100,
                    "minorVersion": msvcVersion % 100,
                    "patchVersion": patchVersion,
                    "abiFamily": "msvc",
                    "abiVersion": msvcVersion,
                    "msvcFullVersion": msvcFullVersion,
                    "languageStandard": 201703,
                },
                "standardLibrary": {
                    "family": "msvc", "version": 145, "abiVersion": 145,
                    "msvcStlUpdate": stlUpdate, "iteratorDebugLevel": 0,
                },
                "runtime": {
                    "linkage": "dynamic", "debug": False,
                    "exceptions": True, "rtti": True,
                },
                "layoutCanaries": {"stdStringSize": 32},
            },
            "dependencies": {
                "eigen": {"maxAlignBytes": 16},
                "python": {"limitedApi": 50921472},
                "swig": {"runtimeVersion": "5"},
            },
        })
        + "\n"
    ).encode("utf-8")


@contextmanager
def _buildInfoArchive(data, abiData=None):
    with io.BytesIO() as buffer:
        with zipfile.ZipFile(buffer, "w") as archive:
            archive.writestr(WHEEL_BUILDER.BUILD_INFO_DATA_PATH, data)
            if abiData is not None:
                archive.writestr(WHEEL_BUILDER.BUILD_ABI_DATA_PATH, abiData)
        buffer.seek(0)
        with zipfile.ZipFile(buffer) as archive:
            yield archive


def test_expected_build_feature_metadata_difference():
    """Verify the optional-wheel builder accepts only the expected feature change."""
    payload = {WHEEL_BUILDER.BUILD_INFO_DATA_PATH}
    component = WHEEL_BUILDER.COMPONENTS["opnav"]

    with _buildInfoArchive(_buildInfoData(False)) as baseArchive:
        with _buildInfoArchive(_buildInfoData(True)) as componentArchive:
            WHEEL_BUILDER.validate_common_payloads(
                baseArchive,
                componentArchive,
                payload,
                payload,
                component,
            )


def test_patch_build_tool_version_differences_warn(capsys):
    """Compatible runner-image version drift emits a warning and continues."""
    payload = {WHEEL_BUILDER.BUILD_INFO_DATA_PATH}
    component = WHEEL_BUILDER.COMPONENTS["opnav"]
    baseData = _buildInfoData(
        False,
        diagnostics=_windowsBuildDiagnostics(
            "19.51.36252.0",
            "1.97.1",
            "1.97.1",
        ),
    )
    componentData = _buildInfoData(
        True,
        diagnostics=_windowsBuildDiagnostics(
            "19.51.36248.0",
            "1.97.0",
            "1.97.0",
        ),
    )

    with _buildInfoArchive(baseData) as baseArchive:
        with _buildInfoArchive(componentData) as componentArchive:
            WHEEL_BUILDER.validate_common_payloads(
                baseArchive,
                componentArchive,
                payload,
                payload,
                component,
            )

    warning = capsys.readouterr().err
    assert "compatible build-metadata differences" in warning
    assert "19.51.36252.0" in warning
    assert "19.51.36248.0" in warning
    assert "1.97.1" in warning
    assert "1.97.0" in warning


def test_patch_abi_compiler_version_differences_warn(capsys):
    """Equivalent compiler ABI metadata permits raw build-number drift."""
    payload = {
        WHEEL_BUILDER.BUILD_ABI_DATA_PATH,
        WHEEL_BUILDER.BUILD_INFO_DATA_PATH,
    }
    component = WHEEL_BUILDER.COMPONENTS["opnav"]
    baseAbiData = _windowsBuildAbiData(195136252)
    componentAbiData = _windowsBuildAbiData(195136248, patchVersion=1)

    with _buildInfoArchive(
        _buildInfoData(False),
        baseAbiData,
    ) as baseArchive:
        with _buildInfoArchive(
            _buildInfoData(True),
            componentAbiData,
        ) as componentArchive:
            WHEEL_BUILDER.validate_common_payloads(
                baseArchive,
                componentArchive,
                payload,
                payload,
                component,
            )

    warning = capsys.readouterr().err
    assert "compatible build-metadata differences" in warning
    assert "195136252" in warning
    assert "195136248" in warning
    assert "patchVersion: 0 versus 1" in warning


def test_abi_layout_difference_fails():
    """A changed ABI layout canary remains incompatible."""
    payload = {
        WHEEL_BUILDER.BUILD_ABI_DATA_PATH,
        WHEEL_BUILDER.BUILD_INFO_DATA_PATH,
    }
    component = WHEEL_BUILDER.COMPONENTS["opnav"]
    baseAbiData = _windowsBuildAbiData(195136252)
    componentAbiData = _windowsBuildAbiData(195136248).replace(
        b"'stdStringSize': 32",
        b"'stdStringSize': 40",
    )

    with _buildInfoArchive(
        _buildInfoData(False),
        baseAbiData,
    ) as baseArchive:
        with _buildInfoArchive(
            _buildInfoData(True),
            componentAbiData,
        ) as componentArchive:
            with pytest.raises(ValueError, match="beyond compatible compiler"):
                WHEEL_BUILDER.validate_common_payloads(
                    baseArchive,
                    componentArchive,
                    payload,
                    payload,
                    component,
                )


def _mixedMsvcMetadata():
    """Represent the compiler pair from nightly run 44332824441."""
    baseInfo = _buildInfoData(
        False,
        diagnostics=_windowsBuildDiagnostics("19.51.36260.0", "1.100.0", "1.100.0"),
    )
    componentInfo = _buildInfoData(
        True,
        diagnostics=_windowsBuildDiagnostics("19.52.37029.0", "1.100.0", "1.100.0"),
    )
    return (
        baseInfo, componentInfo,
        _windowsBuildAbiData(195136260),
        _windowsBuildAbiData(195237029, stlUpdate=202609),
    )


@pytest.mark.parametrize("reverse", [False, True])
def test_msvc_minor_version_difference_warns(reverse, capsys):
    """Accept supported MSVC DLL combinations without changing stored metadata.

    :param reverse: Exercise both old-core/new-component and the reverse pairing.
    :param capsys: Capture warnings describing the preserved build provenance.
    """
    baseInfo, componentInfo, baseAbi, componentAbi = _mixedMsvcMetadata()
    if reverse:
        baseAbi, componentAbi = componentAbi, baseAbi
        baseInfo, componentInfo = (
            componentInfo.replace(b"'opNav': True", b"'opNav': False"),
            baseInfo.replace(b"'opNav': False", b"'opNav': True"),
        )
    payload = {WHEEL_BUILDER.BUILD_INFO_DATA_PATH, WHEEL_BUILDER.BUILD_ABI_DATA_PATH}
    with _buildInfoArchive(baseInfo, baseAbi) as baseArchive:
        with _buildInfoArchive(componentInfo, componentAbi) as componentArchive:
            WHEEL_BUILDER.validate_common_payloads(
                baseArchive, componentArchive, payload, payload,
                WHEEL_BUILDER.COMPONENTS["opnav"],
            )
            assert baseArchive.read(WHEEL_BUILDER.BUILD_ABI_DATA_PATH) == baseAbi
            assert componentArchive.read(WHEEL_BUILDER.BUILD_ABI_DATA_PATH) == componentAbi
    warning = capsys.readouterr().err
    assert "compatible build-metadata differences" in warning
    for version in ("19.51.36260.0", "19.52.37029.0", "1951", "1952", "202604", "202609"):
        assert version in warning


@pytest.mark.parametrize(
    "path, value",
    [
        (("capture",), "inferred"),
        (("target", "system"), "Linux"),
        (("target", "architecture"), "arm64"),
        (("build", "debugRuntime"), True),
        (("c", "compiler", "id"), "Clang"),
        (("c", "compiler", "msvcVersion"), 2000),
        (("c", "layoutCanaries", "packingCanarySize"), 16),
        (("cxx", "compiler", "abiFamily"), "itanium"),
        (("cxx", "compiler", "languageStandard"), 202004),
        (("cxx", "standardLibrary", "abiVersion"), 146),
        (("cxx", "standardLibrary", "iteratorDebugLevel"), 2),
        (("cxx", "runtime", "linkage"), "static"),
        (("cxx", "runtime", "debug"), True),
        (("cxx", "runtime", "exceptions"), False),
        (("cxx", "runtime", "rtti"), False),
        (("cxx", "layoutCanaries", "stdStringSize"), 40),
        (("dependencies", "eigen", "maxAlignBytes"), 32),
        (("dependencies", "python", "limitedApi"), 0),
        (("dependencies", "swig", "runtimeVersion"), "6"),
    ],
)
def test_mixed_msvc_rejects_abi_changes(path, value):
    """Preserve ABI checks even when the compiler pair is normally compatible.

    :param path: ABI metadata field to change in the component wheel.
    :param value: Incompatible replacement value.
    """
    baseInfo, componentInfo, baseAbi, componentAbi = _mixedMsvcMetadata()
    abi = WHEEL_BUILDER.parse_build_abi_data(componentAbi)
    section = abi
    for key in path[:-1]:
        section = section[key]
    section[path[-1]] = value
    componentAbi = f"buildAbiData = {abi!r}\n".encode()
    payload = {WHEEL_BUILDER.BUILD_INFO_DATA_PATH, WHEEL_BUILDER.BUILD_ABI_DATA_PATH}
    with _buildInfoArchive(baseInfo, baseAbi) as baseArchive:
        with _buildInfoArchive(componentInfo, componentAbi) as componentArchive:
            with pytest.raises(ValueError, match="beyond compatible compiler"):
                WHEEL_BUILDER.validate_common_payloads(
                    baseArchive, componentArchive, payload, payload,
                    WHEEL_BUILDER.COMPONENTS["opnav"],
                )


def test_msvc_minor_version_requires_compiled_abi():
    """Diagnostic compiler versions alone must not enable the MSVC exception."""
    baseInfo, componentInfo, _, _ = _mixedMsvcMetadata()
    payload = {WHEEL_BUILDER.BUILD_INFO_DATA_PATH}
    with _buildInfoArchive(baseInfo) as baseArchive:
        with _buildInfoArchive(componentInfo) as componentArchive:
            with pytest.raises(ValueError, match="beyond the expected 'opNav'"):
                WHEEL_BUILDER.validate_common_payloads(
                    baseArchive, componentArchive, payload, payload,
                    WHEEL_BUILDER.COMPONENTS["opnav"],
                )


@pytest.mark.parametrize(
    "path, value",
    [
        (("capture",), "inferred"),
        (("target", "system"), "Linux"),
        (("cxx", "compiler", "id"), "Clang"),
        (("cxx", "compiler", "languageStandard"), 202004),
        (("cxx", "standardLibrary", "family"), "unknown"),
        (("cxx", "runtime", "linkage"), "static"),
        (("cxx", "runtime", "debug"), True),
    ],
)
def test_msvc_exception_requires_supported_build_context(path, value):
    """Matching unsupported settings cannot authorize cross-toolset mixing.

    :param path: Context field to change identically in both input wheels.
    :param value: Setting outside the supported release-DLL compatibility rule.
    """
    baseInfo, componentInfo, baseAbi, componentAbi = _mixedMsvcMetadata()
    changed = []
    for source in (baseAbi, componentAbi):
        abi = WHEEL_BUILDER.parse_build_abi_data(source)
        section = abi
        for key in path[:-1]:
            section = section[key]
        section[path[-1]] = value
        changed.append(f"buildAbiData = {abi!r}\n".encode())
    payload = {WHEEL_BUILDER.BUILD_INFO_DATA_PATH, WHEEL_BUILDER.BUILD_ABI_DATA_PATH}
    with _buildInfoArchive(baseInfo, changed[0]) as baseArchive:
        with _buildInfoArchive(componentInfo, changed[1]) as componentArchive:
            with pytest.raises(ValueError, match="beyond compatible compiler"):
                WHEEL_BUILDER.validate_common_payloads(
                    baseArchive, componentArchive, payload, payload,
                    WHEEL_BUILDER.COMPONENTS["opnav"],
                )


def _runtimeDll(version, *, machine=0x8664, flags=0):
    """Build a minimal PE fixture with a real, nested version resource."""
    data = bytearray(1024)
    data[:2] = b"MZ"
    struct.pack_into("<I", data, 0x3c, 128)
    data[128:132] = b"PE\0\0"
    optionalSize = 224 if machine == 0x14c else 240
    struct.pack_into("<HH", data, 132, machine, 1)
    struct.pack_into("<HH", data, 148, optionalSize, 0x2000)
    struct.pack_into("<H", data, 152, 0x10b if machine == 0x14c else 0x20b)
    directoryOffset = 248 if machine == 0x14c else 264
    struct.pack_into("<I", data, directoryOffset - 4, 16)
    struct.pack_into("<II", data, directoryOffset + 16, 4096, 256)
    sectionOffset = 152 + optionalSize
    data[sectionOffset:sectionOffset + 8] = b".rsrc\0\0\0"
    struct.pack_into("<IIII", data, sectionOffset + 8, 512, 4096, 512, 512)
    # Type -> name -> language -> data entry. Offsets are relative to .rsrc.
    for relative, identifier, child in ((0, 16, 0x80000018), (24, 1, 0x80000030), (48, 1033, 72)):
        struct.pack_into("<HH", data, 512 + relative + 12, 0, 1)
        struct.pack_into("<II", data, 512 + relative + 16, identifier, child)
    struct.pack_into("<IIII", data, 584, 4096 + 88, 92, 0, 0)
    valueOffset = 600
    struct.pack_into("<HHH", data, valueOffset, 92, 52, 0)
    key = "VS_VERSION_INFO\0".encode("utf-16-le")
    data[valueOffset + 6:valueOffset + 6 + len(key)] = key
    versionMS = version[0] << 16 | version[1]
    versionLS = version[2] << 16 | version[3]
    struct.pack_into(
        "<13I", data, valueOffset + 40,
        0xfeef04bd, 0x10000, versionMS, versionLS, versionMS, versionLS,
        0x3f, flags, 0x40004, 2, 0, 0, 0,
    )
    return bytes(data)


def _validateRuntimePair(baseRuntime, componentRuntime, *, sameCompiler=False):
    """Validate shared runtime replacement through the complete wheel check."""
    baseInfo, componentInfo, baseAbi, componentAbi = _mixedMsvcMetadata()
    if sameCompiler:
        componentInfo = baseInfo.replace(b"'opNav': False", b"'opNav': True")
        componentAbi = baseAbi
    runtimePath = "bsk.libs/msvcp140.dll"
    payload = {
        WHEEL_BUILDER.BUILD_INFO_DATA_PATH, WHEEL_BUILDER.BUILD_ABI_DATA_PATH,
        runtimePath,
    }
    buffers = [io.BytesIO(), io.BytesIO()]
    for buffer, info, abi, runtime in zip(
        buffers, (baseInfo, componentInfo), (baseAbi, componentAbi), (baseRuntime, componentRuntime),
    ):
        with zipfile.ZipFile(buffer, "w") as archive:
            archive.writestr(WHEEL_BUILDER.BUILD_INFO_DATA_PATH, info)
            archive.writestr(WHEEL_BUILDER.BUILD_ABI_DATA_PATH, abi)
            archive.writestr(runtimePath, runtime)
        buffer.seek(0)
    with zipfile.ZipFile(buffers[0]) as baseArchive:
        with zipfile.ZipFile(buffers[1]) as componentArchive:
            WHEEL_BUILDER.validate_common_payloads(
                baseArchive, componentArchive, payload, payload,
                WHEEL_BUILDER.COMPONENTS["opnav"],
            )
            assert baseArchive.read(runtimePath) == baseRuntime
            assert componentArchive.read(runtimePath) == componentRuntime


@pytest.mark.parametrize("sameCompiler", [False, True])
def test_newer_core_msvc_runtime_accepts_older_component(sameCompiler):
    """Retain a newer core runtime without requiring identical DLL bytes.

    :param sameCompiler: Also cover runtime updates without compiler drift.
    """
    _validateRuntimePair(
        _runtimeDll((14, 52, 37029, 0)), _runtimeDll((14, 51, 36260, 0)),
        sameCompiler=sameCompiler,
    )


def test_equal_msvc_runtime_version_accepts_nonsemantic_bytes():
    """An unused PE overlay must not make an equal runtime incompatible."""
    runtime = _runtimeDll((14, 51, 36260, 0))
    _validateRuntimePair(runtime, runtime + b"\0", sameCompiler=True)


@pytest.mark.parametrize("sameCompiler", [False, True])
def test_older_core_msvc_runtime_rejects_newer_component(sameCompiler):
    """Do not silently downgrade the component's shared runtime.

    :param sameCompiler: Also reject runtime downgrades without compiler drift.
    """
    with pytest.raises(ValueError, match="equal or newer runtime"):
        _validateRuntimePair(
            _runtimeDll((14, 51, 36260, 0)), _runtimeDll((14, 52, 37029, 0)),
            sameCompiler=sameCompiler,
        )


def test_msvc_runtime_rejects_machine_mismatch():
    """A higher runtime version must not conceal a different PE architecture."""
    with pytest.raises(ValueError, match="same architecture"):
        _validateRuntimePair(
            _runtimeDll((14, 52, 37029, 0), machine=0xaa64),
            _runtimeDll((14, 51, 36260, 0)),
        )


@pytest.mark.parametrize("machine", [0x14c, 0x8664, 0xaa64])
def test_read_msvc_runtime_version(machine):
    """Read numeric versions from PE32 and PE32+ resource directories.

    :param machine: Windows x86, x64, or ARM64 machine identifier.
    """
    version = (14, 52, 37029, 7)
    assert WHEEL_BUILDER.read_msvc_runtime_version(_runtimeDll(version, machine=machine)) == (machine, version)


@pytest.mark.parametrize("runtime", [
    b"unreadable",
    _runtimeDll((14, 52, 37029, 0))[:650],
    _runtimeDll((15, 0, 1, 0)),
    _runtimeDll((14, 52, 37029, 0), flags=1),
    _runtimeDll((14, 52, 37029, 0), flags=2),
])
def test_msvc_runtime_rejects_unverified_replacement(runtime):
    """Fail closed for malformed, unknown-family, debug, or prerelease DLLs.

    :param runtime: Core runtime outside the supported release-DLL policy.
    """
    with pytest.raises(ValueError, match="Cannot establish shared MSVC runtime compatibility"):
        _validateRuntimePair(runtime, _runtimeDll((14, 51, 36260, 0)))


def test_msvc_runtime_ignores_overlay_version_resource():
    """Do not accept a version signature outside the PE resource directory."""
    runtime = bytearray(_runtimeDll((14, 52, 37029, 0)))
    struct.pack_into("<HH", runtime, 524, 0, 0)
    with pytest.raises(ValueError, match="Missing or conflicting"):
        WHEEL_BUILDER.read_msvc_runtime_version(bytes(runtime))


def test_compiler_launcher_differences_warn(capsys):
    """Compiler-cache availability differs without changing compatibility."""
    payload = {WHEEL_BUILDER.BUILD_INFO_DATA_PATH}
    component = WHEEL_BUILDER.COMPONENTS["opnav"]
    baseData = _buildInfoData(
        False,
        diagnostics=_windowsBuildDiagnostics("19.51.1", "1.97.1", "1.97.1"),
    )
    componentData = _buildInfoData(
        True,
        diagnostics=_windowsBuildDiagnostics(
            "19.51.1",
            "1.97.1",
            "1.97.1",
            compilerLauncher="sccache",
        ),
    )

    with _buildInfoArchive(baseData) as baseArchive:
        with _buildInfoArchive(componentData) as componentArchive:
            WHEEL_BUILDER.validate_common_payloads(
                baseArchive,
                componentArchive,
                payload,
                payload,
                component,
            )

    warning = capsys.readouterr().err
    assert "compatible build-metadata differences" in warning
    assert "diagnostics.compilers.c.launcher: '' versus 'sccache'" in warning
    assert "diagnostics.compilers.cxx.launcher: '' versus 'sccache'" in warning


def test_build_tool_minor_version_difference_fails():
    """A different build-tool major/minor release remains incompatible."""
    payload = {WHEEL_BUILDER.BUILD_INFO_DATA_PATH}
    component = WHEEL_BUILDER.COMPONENTS["opnav"]
    baseData = _buildInfoData(
        False,
        diagnostics=_windowsBuildDiagnostics("19.51.1", "1.97.1", "1.97.1"),
    )
    componentData = _buildInfoData(
        True,
        diagnostics=_windowsBuildDiagnostics("19.51.2", "1.98.0", "1.98.0"),
    )

    with _buildInfoArchive(baseData) as baseArchive:
        with _buildInfoArchive(componentData) as componentArchive:
            with pytest.raises(ValueError, match="beyond the expected 'opNav'"):
                WHEEL_BUILDER.validate_common_payloads(
                    baseArchive,
                    componentArchive,
                    payload,
                    payload,
                    component,
                )


@pytest.mark.parametrize(
    "componentData, errorText",
    [
        pytest.param(
            _buildInfoData(False),
            "disabled in the base build and enabled in the component build",
            id="feature-not-enabled",
        ),
        pytest.param(
            _buildInfoData(True, schemaVersion=5),
            "beyond the expected 'opNav' feature value",
            id="unrelated-metadata-drift",
        ),
        pytest.param(
            (
                b'buildInfoData = {"schemaVersion": 4, "artifact": '
                b'{"basiliskVersion": "test-version"}, "features": '
                b'{"vizInterface": True, "mujoco": True}}\n'
            ),
            "Missing build feature 'opNav'",
            id="feature-missing",
        ),
    ],
)
def test_invalid_build_feature_metadata_difference(componentData, errorText):
    """Verify invalid component build metadata is rejected.

    :param componentData: Generated component build-information source.
    :param errorText: Expected validation-error text.
    """
    payload = {WHEEL_BUILDER.BUILD_INFO_DATA_PATH}
    component = WHEEL_BUILDER.COMPONENTS["opnav"]

    with _buildInfoArchive(_buildInfoData(False)) as baseArchive:
        with _buildInfoArchive(componentData) as componentArchive:
            with pytest.raises(ValueError, match=errorText):
                WHEEL_BUILDER.validate_common_payloads(
                    baseArchive,
                    componentArchive,
                    payload,
                    payload,
                    component,
                )


def test_missing_or_malformed_build_feature_metadata():
    """Verify missing and non-literal build metadata is rejected."""
    component = WHEEL_BUILDER.COMPONENTS["opnav"]
    buildInfoPath = WHEEL_BUILDER.BUILD_INFO_DATA_PATH

    with _buildInfoArchive(_buildInfoData(False)) as baseArchive:
        with _buildInfoArchive(_buildInfoData(True)) as componentArchive:
            with pytest.raises(ValueError, match="Both wheel build inputs must contain"):
                WHEEL_BUILDER.validate_common_payloads(
                    baseArchive,
                    componentArchive,
                    set(),
                    {buildInfoPath},
                    component,
                )

    malformedData = b"buildInfoData = dict(features={})\n"
    with pytest.raises(ValueError):
        WHEEL_BUILDER.parse_build_info_data(malformedData)


def test_optional_wheel_declares_build_feature(tmp_path, monkeypatch):
    """Verify an optional wheel declares a discoverable build feature."""
    component = WHEEL_BUILDER.COMPONENTS["opnav"]
    assert (
        WHEEL_BUILDER.BUILD_FEATURE_ENTRY_POINT_GROUP
        == buildInfoFormatter._buildFeatureEntryPointGroup
    )
    sourceWheel = tmp_path / "bsk-full.whl"
    outputWheel = tmp_path / "bsk-opnav.whl"
    payloadName = "Basilisk/fswAlgorithms/example.py"
    coreVersion = buildInfoFormatter._buildInfoData["artifact"]["basiliskVersion"]
    sourceDistInfo = f"bsk-{coreVersion}.dist-info"

    with zipfile.ZipFile(sourceWheel, "w") as archive:
        archive.writestr(payloadName, b"# optional payload\n")
        archive.writestr(
            f"{sourceDistInfo}/WHEEL",
            "Wheel-Version: 1.0\nGenerator: test\nRoot-Is-Purelib: false\n",
        )
        archive.writestr(f"{sourceDistInfo}/licenses/LICENSE", b"Basilisk license\n")
        archive.writestr(
            f"{sourceDistInfo}/licenses/LICENSES/RUST-THIRD-PARTY.txt",
            b"Rust dependency licenses\n",
        )

    WHEEL_BUILDER.build_wheel_file(
        sourceWheel,
        outputWheel,
        component,
        {},
        [payloadName],
        coreVersion,
    )

    optionalDistInfo = f"bsk_opnav-{coreVersion}.dist-info"
    entryPointPath = f"{optionalDistInfo}/entry_points.txt"
    with zipfile.ZipFile(outputWheel) as archive:
        entryPointData = archive.read(entryPointPath).decode("utf-8")
        metadataData = archive.read(f"{optionalDistInfo}/METADATA").decode("utf-8")
        recordData = archive.read(f"{optionalDistInfo}/RECORD").decode("utf-8")
        licenseData = archive.read(f"{optionalDistInfo}/licenses/LICENSE")
        rustLicenseData = archive.read(
            f"{optionalDistInfo}/licenses/LICENSES/RUST-THIRD-PARTY.txt"
        )
        installPath = tmp_path / "installed"
        archive.extractall(installPath)

    assert entryPointData == "[basilisk.build_features]\nopNav = Basilisk\n"
    assert "License-File: LICENSE\n" in metadataData
    assert "License-File: LICENSES/RUST-THIRD-PARTY.txt\n" in metadataData
    assert licenseData == b"Basilisk license\n"
    assert rustLicenseData == b"Rust dependency licenses\n"
    assert entryPointPath in recordData

    installedDistributions = list(distributions(path=[installPath]))
    monkeypatch.setitem(buildInfoFormatter._buildInfoData["features"], "opNav", False)
    monkeypatch.setattr(
        buildInfoFormatter,
        "_distributions",
        lambda: installedDistributions,
    )
    assert buildInfoFormatter.hasBuildFeature("opNav") is True


@pytest.mark.parametrize(
    "build_version, runtime_version, installed_version",
    [
        pytest.param("2.12.0", "2.12.0", "2.12.0", id="matching-versions"),
        pytest.param("2.12.0b0", "2.12.0", "2.12.0", id="stale-build"),
        pytest.param("2.12.0", "2.12.0b0", "2.12.0b0", id="stale-metadata"),
        pytest.param("2.12.0", "2.12.0b0", "2.12.0", id="stale-runtime"),
        pytest.param("2.12.0", "2.12.0", "2.12.0b0", id="stale-distribution"),
        pytest.param("2.12.0", "0.0.0", "2.12.0", id="unknown-runtime"),
    ],
)
@pytest.mark.parametrize(
    "expectedFeatures",
    [
        pytest.param(WHEEL_TESTER.CORE_FEATURES, id="core"),
        pytest.param(WHEEL_TESTER.OPNAV_FEATURES, id="opnav"),
    ],
)
def test_optional_wheel_import_check_validates_features_and_version(
    monkeypatch,
    expectedFeatures,
    build_version,
    runtime_version,
    installed_version,
):
    """Verify wheel checks validate public features and version consistency.

    :param monkeypatch: Pytest fixture used to install isolated fake modules.
    :param expectedFeatures: Expected build-feature values for the wheel variant.
    :param build_version: Version recorded in the compiled build metadata.
    :param runtime_version: Version exposed by the imported Basilisk package.
    :param installed_version: Version recorded in the installed distribution.
    """
    queriedFeatures = []

    class FakeRustModule:
        pass

    def hasBuildFeature(name):
        queriedFeatures.append(name)
        return expectedFeatures[name]

    fakeBasilisk = ModuleType("Basilisk")
    fakeBasilisk.__file__ = "fake/Basilisk/__init__.py"
    fakeBasilisk.__version__ = runtime_version
    fakeBasilisk.hasBuildFeature = hasBuildFeature
    fakeBasilisk.getBuildInfo = lambda: {
        "artifact": {"basiliskVersion": build_version},
        "diagnostics": {"tools": {"corrosion": "test-version"}}
    }

    def distribution_version(name):
        assert name == "bsk"
        return installed_version

    monkeypatch.setattr("importlib.metadata.version", distribution_version)

    fakeRustModuleApi = ModuleType("Basilisk.moduleTemplates.rustModuleTemplate")
    fakeRustModuleApi.rustModuleTemplate = FakeRustModule

    monkeypatch.setitem(sys.modules, "Basilisk", fakeBasilisk)
    monkeypatch.setitem(
        sys.modules,
        "Basilisk.moduleTemplates.rustModuleTemplate",
        fakeRustModuleApi,
    )

    script = WHEEL_TESTER.import_check_script([], [], expectedFeatures)
    if build_version == runtime_version == installed_version:
        exec(script, {})
    else:
        with pytest.raises(SystemExit, match="wheel version mismatch"):
            exec(script, {})

    assert queriedFeatures == list(expectedFeatures)


def test_optional_wheel_checks_ignore_checkout_metadata(tmp_path, monkeypatch):
    """Wheel subprocesses ignore checkout metadata and ``PYTHONPATH`` packages."""
    environment = tmp_path / "environment"
    venv.EnvBuilder(with_pip=False).create(environment)
    python = WHEEL_TESTER.venv_python(environment)
    site_packages = Path(subprocess.check_output(
        [python, "-I", "-c", 'import sysconfig; print(sysconfig.get_path("purelib"))'],
        text=True,
    ).strip())

    package = site_packages / "Basilisk"
    package.mkdir()
    (package / "__init__.py").write_text(
        'from importlib.metadata import version\n'
        '__version__ = version("bsk")\n'
        'def getBuildInfo():\n'
        '    return {"artifact": {"basiliskVersion": "2.12.0"},\n'
        '            "diagnostics": {"tools": {"corrosion": "test-version"}}}\n',
        encoding="utf-8",
    )
    module_templates = package / "moduleTemplates"
    module_templates.mkdir()
    (module_templates / "__init__.py").write_text("", encoding="utf-8")
    (module_templates / "rustModuleTemplate.py").write_text(
        "class rustModuleTemplate:\n    pass\n", encoding="utf-8"
    )
    metadata = site_packages / "bsk-2.12.0.dist-info"
    metadata.mkdir()
    (metadata / "METADATA").write_text(
        "Metadata-Version: 2.4\nName: bsk\nVersion: 2.12.0\n", encoding="utf-8"
    )

    stale_metadata = tmp_path / "bsk.egg-info"
    stale_metadata.mkdir()
    (stale_metadata / "PKG-INFO").write_text(
        "Metadata-Version: 2.4\nName: bsk\nVersion: 2.12.0b0\n", encoding="utf-8"
    )
    shadow_package = tmp_path / "shadow" / "Basilisk"
    shadow_package.mkdir(parents=True)
    (shadow_package / "__init__.py").write_text(
        'raise RuntimeError("Imported a package from PYTHONPATH")\n', encoding="utf-8"
    )
    monkeypatch.chdir(tmp_path)
    subprocess_environment = {**os.environ, "PYTHONPATH": str(shadow_package.parent)}
    module_name = "Basilisk.moduleTemplates.rustModuleTemplate"
    WHEEL_TESTER.run_import_check(
        python,
        required=[module_name],
        missing=[],
        expected_features={},
        env=subprocess_environment,
    )

    # The shutdown checks must use the installed package as well.
    monkeypatch.setattr(
        WHEEL_TESTER, "PROTOBUF_CONSUMERS", ((module_name, "rustModuleTemplate"),)
    )
    WHEEL_TESTER.run_protobuf_consumer_checks(python, subprocess_environment)


def test_optional_wheel_rejects_unsafe_license_path():
    """Verify legal files cannot escape the wheel license directory."""
    with io.BytesIO() as buffer:
        with zipfile.ZipFile(buffer, "w") as archive:
            archive.writestr("example.dist-info/licenses/../../payload", b"invalid\n")
        buffer.seek(0)
        with zipfile.ZipFile(buffer) as archive:
            with pytest.raises(ValueError, match="Invalid wheel license path"):
                WHEEL_BUILDER.read_license_files(
                    archive,
                    "example.dist-info/licenses/",
                )

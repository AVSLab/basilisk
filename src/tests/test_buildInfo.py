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

import ctypes
import platform
import re
from pathlib import Path

import pytest

import Basilisk
import Basilisk._buildInfo as buildInfoFormatter


ABI_CONTRACT_HEADER = (
    Path(__file__).parents[1] / "architecture" / "utilities" / "bskAbiDescriptor.h"
)


def _integerDefine(name):
    headerText = ABI_CONTRACT_HEADER.read_text()
    match = re.search(rf"^#define\s+{name}\s+(\d+)\s*$", headerText, re.MULTILINE)
    assert match is not None
    return int(match.group(1))


class _MsgHeader(ctypes.Structure):
    _fields_ = [
        ("isLinked", ctypes.c_int64),
        ("isWritten", ctypes.c_int64),
        ("timeWritten", ctypes.c_uint64),
        ("moduleID", ctypes.c_int64),
    ]


class _PackingCanary(ctypes.Structure):
    _fields_ = [
        ("first", ctypes.c_uint8),
        ("second", ctypes.c_uint32),
        ("third", ctypes.c_uint16),
    ]


def _makeBuildInfo(
    system,
    processor,
    compilerId,
    compilerVersion,
    compilerExecutable,
    generator,
    *,
    multiConfig=False,
    generatorPlatform="",
    generatorToolset="",
    standardLibrary="",
    runtime="",
    runtimeType="",
):
    standardLibraryFamily = standardLibrary
    if standardLibrary.startswith("libstdc++"):
        standardLibraryFamily = "libstdc++"
    abiFamily = "msvc" if system == "Windows" else "itanium"
    return {
        "schemaVersion": 3,
        "artifact": {
            "basiliskVersion": "2.12.0",
            "sourceRevision": "0123456789abcdef",
            "sourceDirty": False,
            "extensionAbiVersion": 1,
        },
        "diagnostics": {
            "target": {
                "system": system,
                "processor": processor,
                "pointerSizeBytes": 8,
                "osxArchitectures": processor if system == "Darwin" else "",
                "osxDeploymentTarget": "",
            },
            "build": {
                "configuration": "" if multiConfig else "Release",
                "multiConfig": multiConfig,
                "availableConfigurations": ["Debug", "Release"] if multiConfig else [],
                "generator": generator,
                "generatorPlatform": generatorPlatform,
                "generatorToolset": generatorToolset,
                "pythonLimitedApi": "0x03090000",
            },
            "compilers": {
                "c": {
                    "id": compilerId,
                    "version": compilerVersion,
                    "executable": compilerExecutable,
                    "launcher": "",
                    "frontendVariant": "",
                    "simulatedId": "",
                    "simulatedVersion": "",
                    "target": "",
                    "standard": "",
                    "extensions": "",
                },
                "cxx": {
                    "id": compilerId,
                    "version": compilerVersion,
                    "executable": compilerExecutable,
                    "launcher": "",
                    "frontendVariant": "",
                    "simulatedId": "",
                    "simulatedVersion": "",
                    "target": "",
                    "standard": "17",
                    "extensions": "OFF",
                },
            },
            "conanSettings": {
                "buildType": "Release",
                "cxxStandard": "17",
                "cxxStandardLibrary": standardLibrary,
                "compilerRuntime": runtime,
                "compilerRuntimeType": runtimeType,
            },
            "tools": {
                "cmake": "4.2.0",
                "conan": "2.23.0",
                "swig": "4.4.1",
                "python": "3.14.6",
            },
        },
        "abi": {
            "descriptorVersion": 1,
            "capture": "compiled",
            "contractHeader": "architecture/utilities/bskAbiDescriptor.h",
            "build": {
                "configuration": "Release",
                "debugRuntime": False,
                "assertionsDisabled": True,
            },
            "target": {
                "system": system,
                "architecture": processor,
                "pointerSizeBytes": 8,
                "endianness": "little",
                "instructionSet": "baseline",
            },
            "c": {
                "compiler": {
                    "id": compilerId,
                    "version": compilerVersion,
                    "languageStandard": 201710,
                    "msvcVersion": 1944 if system == "Windows" else 0,
                },
                "fundamentalTypes": {},
                "layoutCanaries": {},
            },
            "cxx": {
                "compiler": {
                    "id": compilerId,
                    "version": compilerVersion,
                    "frontend": "MSVC" if system == "Windows" else "GNU",
                    "languageStandard": 201703,
                    "abiFamily": abiFamily,
                    "abiVersion": 1944 if system == "Windows" else 1002,
                    "msvcFullVersion": 194400000 if system == "Windows" else 0,
                },
                "standardLibrary": {
                    "family": standardLibraryFamily,
                    "version": 1,
                    "abiVersion": 1,
                    "libstdcxxRelease": 15 if standardLibraryFamily == "libstdc++" else 0,
                    "useCxx11Abi": 1 if standardLibraryFamily == "libstdc++" else None,
                    "iteratorDebugLevel": 0 if system == "Windows" else None,
                    "debugMode": False,
                    "assertionsEnabled": False,
                },
                "runtime": {
                    "linkage": runtime or "system",
                    "debug": False,
                    "exceptions": True,
                    "rtti": True,
                },
                "layoutCanaries": {},
            },
            "dependencies": {
                "eigen": {
                    "version": "3.4.0",
                    "maxAlignBytes": 16,
                    "maxStaticAlignBytes": 16,
                    "vectorization": "none",
                },
                "swig": {"runtimeVersion": "5"},
                "python": {
                    "implementation": "CPython",
                    "limitedApi": 0x03090000,
                    "soabi": "",
                    "debug": False,
                    "freeThreaded": False,
                },
            },
        },
    }


def test_build_info():
    """Verify that Basilisk exposes compiled and diagnostic build metadata."""
    buildInfo = Basilisk.getBuildInfo()
    diagnostics = buildInfo["diagnostics"]
    abi = buildInfo["abi"]

    assert buildInfo["schemaVersion"] == 3
    assert buildInfo["artifact"]["basiliskVersion"]
    if Basilisk.__version__ != "0.0.0":
        assert buildInfo["artifact"]["basiliskVersion"] == Basilisk.__version__
    assert buildInfo["artifact"]["extensionAbiVersion"] == _integerDefine(
        "BSK_EXTENSION_ABI_VERSION"
    )
    assert buildInfo["artifact"]["sourceDirty"] in (True, False, None)
    assert abi["descriptorVersion"] == _integerDefine("BSK_ABI_DESCRIPTOR_VERSION")
    assert abi["capture"] == "compiled"
    assert abi["contractHeader"] == "architecture/utilities/bskAbiDescriptor.h"
    assert abi["build"]["configuration"]
    assert abi["target"]["system"] == platform.system()
    assert abi["target"]["pointerSizeBytes"] == ctypes.sizeof(ctypes.c_void_p)
    assert abi["target"]["endianness"] in ("little", "big")
    assert abi["target"]["architecture"]

    assert diagnostics["target"]["system"] == platform.system()
    assert isinstance(diagnostics["build"]["multiConfig"], bool)
    if diagnostics["build"]["multiConfig"]:
        assert not diagnostics["build"]["configuration"]
        assert diagnostics["build"]["availableConfigurations"]
    else:
        assert diagnostics["build"]["configuration"]
    assert diagnostics["build"]["generator"]

    for language in ("c", "cxx"):
        compilerInfo = diagnostics["compilers"][language]
        assert compilerInfo["id"]
        assert compilerInfo["version"]
        assert compilerInfo["executable"]
        assert abi[language]["compiler"]["id"] == compilerInfo["id"]
        assert abi[language]["compiler"]["version"]
        assert abi[language]["compiler"]["majorVersion"] > 0

    assert abi["c"]["runtime"]["family"]
    assert abi["cxx"]["compiler"]["languageStandard"] >= 201703
    assert abi["cxx"]["compiler"]["abiFamily"] in ("itanium", "msvc")
    assert abi["cxx"]["standardLibrary"]["family"] in ("libc++", "libstdc++", "msvc")
    assert abi["cxx"]["runtime"]["exceptions"]
    assert abi["cxx"]["runtime"]["rtti"]
    assert abi["dependencies"]["eigen"]["version"] == "3.4.0"
    assert abi["dependencies"]["eigen"]["maxAlignBytes"] >= 0
    assert abi["dependencies"]["swig"]["runtimeVersion"]
    assert abi["dependencies"]["python"]["limitedApi"] > 0
    assert "soabi" in abi["dependencies"]["python"]

    assert "buildType" in diagnostics["conanSettings"]
    assert diagnostics["tools"]["cmake"]
    assert diagnostics["tools"]["swig"]
    assert diagnostics["tools"]["python"]

    buildInfo["abi"]["target"]["system"] = "modified"
    assert Basilisk.getBuildInfo()["abi"]["target"]["system"] == platform.system()


def test_shared_abi_contract_header():
    """Verify that the SDK-syncable header owns the ABI contract definitions."""
    headerText = ABI_CONTRACT_HEADER.read_text()

    assert "BskAbiPackingCanary" in headerText
    assert "BskAbiPayloadCanary" in headerText
    assert "bskCreateCAbiInfo" in headerText
    assert "createCxxAbiInfo" in headerText


def test_compiled_c_layout_canaries():
    """Verify compiled C layout canaries against the running platform."""
    cAbi = Basilisk.getBuildInfo()["abi"]["c"]
    fundamentalTypes = cAbi["fundamentalTypes"]
    layoutCanaries = cAbi["layoutCanaries"]

    assert fundamentalTypes["shortSize"] == ctypes.sizeof(ctypes.c_short)
    assert fundamentalTypes["intSize"] == ctypes.sizeof(ctypes.c_int)
    assert fundamentalTypes["longSize"] == ctypes.sizeof(ctypes.c_long)
    assert fundamentalTypes["longLongSize"] == ctypes.sizeof(ctypes.c_longlong)
    assert fundamentalTypes["sizeTSize"] == ctypes.sizeof(ctypes.c_size_t)
    assert fundamentalTypes["pointerSize"] == ctypes.sizeof(ctypes.c_void_p)

    assert layoutCanaries["packingCanarySize"] == ctypes.sizeof(_PackingCanary)
    assert layoutCanaries["packingCanaryAlignment"] == ctypes.alignment(_PackingCanary)
    assert layoutCanaries["packingCanaryUint32Offset"] == _PackingCanary.second.offset
    assert layoutCanaries["packingCanaryUint16Offset"] == _PackingCanary.third.offset
    assert layoutCanaries["msgHeaderSize"] == ctypes.sizeof(_MsgHeader)
    assert layoutCanaries["msgHeaderAlignment"] == ctypes.alignment(_MsgHeader)
    assert layoutCanaries["msgHeaderIsWrittenOffset"] == _MsgHeader.isWritten.offset
    assert layoutCanaries["msgHeaderTimeWrittenOffset"] == _MsgHeader.timeWritten.offset
    assert layoutCanaries["msgHeaderModuleIdOffset"] == _MsgHeader.moduleID.offset


def test_compiled_platform_abi_fields():
    """Verify the ABI fields required by each supported platform."""
    abi = Basilisk.getBuildInfo()["abi"]
    cxx = abi["cxx"]
    standardLibrary = cxx["standardLibrary"]

    if platform.system() == "Windows":
        assert cxx["compiler"]["abiFamily"] == "msvc"
        assert cxx["compiler"]["msvcFullVersion"] > 0
        assert standardLibrary["family"] == "msvc"
        assert standardLibrary["msvcStlUpdate"] > 0
        assert standardLibrary["iteratorDebugLevel"] in (0, 1, 2)
        assert cxx["runtime"]["linkage"] in ("dynamic", "static")
    elif platform.system() == "Darwin":
        assert cxx["compiler"]["abiFamily"] == "itanium"
        assert standardLibrary["family"] == "libc++"
        assert standardLibrary["abiVersion"] > 0
    elif platform.system() == "Linux":
        assert cxx["compiler"]["abiFamily"] == "itanium"
        assert standardLibrary["family"] in ("libc++", "libstdc++")
        if standardLibrary["family"] == "libstdc++":
            assert standardLibrary["useCxx11Abi"] in (0, 1)
            assert standardLibrary["libstdcxxRelease"] > 0


def test_print_build_info(capsys):
    """Verify that the human-readable summary is concise and complete."""
    buildInfo = Basilisk.getBuildInfo()
    diagnostics = buildInfo["diagnostics"]
    standardLibraryFamily = buildInfo["abi"]["cxx"]["standardLibrary"]["family"]
    standardLibraryLabel = {"msvc": "MSVC STL"}.get(standardLibraryFamily, standardLibraryFamily)

    Basilisk.printBuildInfo()
    output = capsys.readouterr().out

    assert "Basilisk Build Information" in output
    assert "Build Tools" in output
    assert buildInfo["artifact"]["basiliskVersion"] in output
    assert diagnostics["compilers"]["c"]["id"] in output
    assert diagnostics["compilers"]["cxx"]["id"] in output
    assert "C++17" in output
    assert standardLibraryLabel in output
    assert buildInfo["abi"]["dependencies"]["eigen"]["version"] in output
    assert buildInfo["abi"]["dependencies"]["swig"]["runtimeVersion"] in output
    for toolVersion in diagnostics["tools"].values():
        if toolVersion:
            assert toolVersion in output
    assert "''" not in output


@pytest.mark.parametrize(
    "buildInfo, expectedText",
    [
        (
            _makeBuildInfo(
                "Darwin",
                "arm64",
                "AppleClang",
                "21.0.0",
                "c++",
                "Unix Makefiles",
                standardLibrary="libc++",
            ),
            (
                "Target:         macOS arm64, 64-bit",
                "Build:          Release, Unix Makefiles",
                "C++ ABI:        libc++",
                "Conan profile:  Release, C++17, libc++",
            ),
        ),
        (
            _makeBuildInfo(
                "Linux",
                "x86_64",
                "GNU",
                "15.1.0",
                "g++",
                "Ninja",
                standardLibrary="libstdc++11",
            ),
            (
                "Target:         Linux x86_64, 64-bit",
                "Build:          Release, Ninja",
                "C++ ABI:        libstdc++",
                "Conan profile:  Release, C++17, libstdc++11",
            ),
        ),
        (
            _makeBuildInfo(
                "Windows",
                "AMD64",
                "MSVC",
                "19.44",
                "cl.exe",
                "Visual Studio 17 2022",
                multiConfig=True,
                generatorPlatform="x64",
                generatorToolset="v143",
                standardLibrary="msvc",
                runtime="dynamic",
                runtimeType="Release",
            ),
            (
                "Target:         Windows AMD64, 64-bit",
                "Build:          Release, Visual Studio 17 2022 (x64, v143)",
                "C++ ABI:        MSVC STL",
                "Conan profile:  Release, C++17, msvc, dynamic (Release)",
            ),
        ),
    ],
)
def test_format_build_info_for_supported_platforms(buildInfo, expectedText):
    """Verify formatting for representative macOS, Linux, and Windows builds."""
    output = buildInfoFormatter._formatBuildInfo(buildInfo)

    for text in expectedText:
        assert text in output
    assert "''" not in output

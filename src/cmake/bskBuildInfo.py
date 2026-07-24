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

from copy import deepcopy as _deepcopy

from Basilisk._buildAbiData import buildAbiData as _buildAbiData
from Basilisk._buildInfoData import buildInfoData as _buildInfoData

_buildInfoData = {**_buildInfoData, "abi": _buildAbiData}


def getBuildInfo() -> dict[str, object]:
    """Return metadata describing the Basilisk binary and its build.

    Values under ``abi`` are captured by compiled C and C++ probes. Values
    under ``diagnostics`` describe CMake observations, build tools, and the
    requested Conan profile.

    :return: A copy of the build metadata.
    :rtype: dict[str, object]
    """
    return _deepcopy(_buildInfoData)


def _appendField(lines: list[str], label: str, value: str) -> None:
    if value:
        lines.append(f"  {label + ':':<16}{value}")


def _compilerDescription(compilerInfo: dict[str, str]) -> str:
    description = " ".join(
        value for value in (compilerInfo["id"], compilerInfo["version"]) if value
    )
    details = []
    if compilerInfo["executable"]:
        details.append(compilerInfo["executable"])
    if compilerInfo["launcher"]:
        details.append(f"launcher: {compilerInfo['launcher']}")
    if details:
        description += f" ({'; '.join(details)})"
    return description


def _languageStandard(prefix: str, standard: str) -> str:
    if standard.isdigit():
        return f"{prefix}{standard}"
    return standard


def _compiledLanguageStandard(prefix: str, standard: int) -> str:
    standards = {
        "C": {199409: "C95", 199901: "C99", 201112: "C11", 201710: "C17", 202311: "C23"},
        "C++": {199711: "C++98", 201103: "C++11", 201402: "C++14", 201703: "C++17", 202002: "C++20"},
    }
    return standards.get(prefix, {}).get(standard, f"{prefix} ({standard})" if standard else "")


def _cxxAbiDescription(abi: dict[str, object]) -> str:
    cxx = abi["cxx"]
    standardLibrary = cxx["standardLibrary"]
    details = []
    if standardLibrary["version"]:
        details.append(str(standardLibrary["version"]))
    if standardLibrary["abiVersion"]:
        details.append(f"ABI {standardLibrary['abiVersion']}")
    if standardLibrary["useCxx11Abi"] is not None:
        details.append(f"C++11 ABI {standardLibrary['useCxx11Abi']}")
    description = {"msvc": "MSVC STL"}.get(
        standardLibrary["family"], standardLibrary["family"]
    )
    if details:
        description += f" ({', '.join(details)})"
    abiFamily = {"itanium": "Itanium", "msvc": "MSVC"}.get(
        cxx["compiler"]["abiFamily"], cxx["compiler"]["abiFamily"]
    )
    return f"{description}, {abiFamily} ABI"


def _runtimeDescription(abi: dict[str, object]) -> str:
    runtime = abi["cxx"]["runtime"]
    details = [runtime["linkage"]]
    if abi["cxx"]["compiler"]["abiFamily"] == "msvc":
        details.append("debug" if runtime["debug"] else "release")
    if runtime["exceptions"]:
        details.append("exceptions")
    if runtime["rtti"]:
        details.append("RTTI")
    return ", ".join(value for value in details if value)


def _formatBuildInfo(buildInfo: dict[str, object]) -> str:
    artifact = buildInfo["artifact"]
    diagnostics = buildInfo["diagnostics"]
    abi = buildInfo["abi"]
    targetInfo = diagnostics["target"]
    compiledTarget = abi["target"]
    build = diagnostics["build"]
    cCompiler = diagnostics["compilers"]["c"]
    cxxCompiler = diagnostics["compilers"]["cxx"]
    conanSettings = diagnostics["conanSettings"]

    system = {"Darwin": "macOS"}.get(compiledTarget["system"], compiledTarget["system"])
    architectures = compiledTarget["architecture"]
    architectures = architectures.replace(";", ", ")
    target = " ".join(value for value in (system, architectures) if value)
    if compiledTarget["pointerSizeBytes"]:
        target += f", {compiledTarget['pointerSizeBytes'] * 8}-bit"

    configuration = abi["build"]["configuration"]
    if not configuration:
        configuration = "Multi-config" if build["multiConfig"] else build["configuration"]
    generator = build["generator"]
    generatorDetails = [
        value for value in (build["generatorPlatform"], build["generatorToolset"]) if value
    ]
    if generatorDetails:
        generator += f" ({', '.join(generatorDetails)})"
    buildDescription = ", ".join(value for value in (configuration, generator) if value)

    lines = ["Basilisk Build Information"]
    _appendField(
        lines,
        "Version",
        f"{artifact['basiliskVersion']} (extension ABI {artifact['extensionAbiVersion']})",
    )
    _appendField(lines, "Target", target)
    _appendField(lines, "Build", buildDescription)
    _appendField(lines, "C compiler", _compilerDescription(cCompiler))
    _appendField(lines, "C++ compiler", _compilerDescription(cxxCompiler))
    _appendField(lines, "C standard", _compiledLanguageStandard("C", abi["c"]["compiler"]["languageStandard"]))
    _appendField(lines, "C++ standard", _compiledLanguageStandard("C++", abi["cxx"]["compiler"]["languageStandard"]))
    _appendField(lines, "C++ ABI", _cxxAbiDescription(abi))
    _appendField(lines, "C++ runtime", _runtimeDescription(abi))
    _appendField(lines, "Python API", build["pythonLimitedApi"])
    _appendField(lines, "macOS target", targetInfo["osxDeploymentTarget"])

    eigen = abi["dependencies"]["eigen"]
    eigenDescription = f"{eigen['version']}, {eigen['maxAlignBytes']}-byte max alignment"
    if eigen["vectorization"] != "none":
        eigenDescription += f", {eigen['vectorization']}"
    _appendField(lines, "Eigen", eigenDescription)
    _appendField(lines, "SWIG runtime", abi["dependencies"]["swig"]["runtimeVersion"])

    conanProfile = []
    if conanSettings["buildType"]:
        conanProfile.append(conanSettings["buildType"])
    if conanSettings["cxxStandard"]:
        conanProfile.append(_languageStandard("C++", conanSettings["cxxStandard"]))
    if conanSettings["cxxStandardLibrary"]:
        conanProfile.append(conanSettings["cxxStandardLibrary"])
    runtime = conanSettings["compilerRuntime"]
    if runtime and conanSettings["compilerRuntimeType"]:
        runtime += f" ({conanSettings['compilerRuntimeType']})"
    if runtime:
        conanProfile.append(runtime)
    _appendField(lines, "Conan profile", ", ".join(conanProfile))

    toolLines = []
    for label, key in (("CMake", "cmake"), ("Conan", "conan"), ("SWIG", "swig"), ("Python", "python")):
        _appendField(toolLines, label, diagnostics["tools"][key])
    if toolLines:
        lines.extend(("", "Build Tools", *toolLines))

    return "\n".join(lines)


def printBuildInfo() -> None:
    """Print a concise summary of the Basilisk binary and its build.

    Empty fields that do not apply to the current platform are omitted.

    :return: None.
    """
    print(_formatBuildInfo(_buildInfoData))


__all__ = ["getBuildInfo", "printBuildInfo"]

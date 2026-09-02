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
import sys
import zipfile
from contextlib import contextmanager
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


def _windowsBuildAbiData(msvcFullVersion, patchVersion=0):
    compilerVersion = str(msvcFullVersion)
    return (
        "buildAbiData = "
        + repr({
            "target": {"system": "Windows", "architecture": "x86_64"},
            "c": {
                "compiler": {
                    "id": "MSVC",
                    "version": compilerVersion,
                    "majorVersion": 19,
                    "minorVersion": 51,
                    "patchVersion": patchVersion,
                    "msvcVersion": 1951,
                },
            },
            "cxx": {
                "compiler": {
                    "id": "MSVC",
                    "version": compilerVersion,
                    "majorVersion": 19,
                    "minorVersion": 51,
                    "patchVersion": patchVersion,
                    "abiFamily": "msvc",
                    "abiVersion": 1951,
                    "msvcFullVersion": msvcFullVersion,
                },
                "layoutCanaries": {"stdStringSize": 32},
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
    "expectedFeatures",
    [
        pytest.param(WHEEL_TESTER.CORE_FEATURES, id="core"),
        pytest.param(WHEEL_TESTER.OPNAV_FEATURES, id="opnav"),
    ],
)
def test_optional_wheel_import_check_uses_public_feature_api(
    monkeypatch,
    expectedFeatures,
):
    """Verify the generated import check queries the public feature API.

    :param monkeypatch: Pytest fixture used to install isolated fake modules.
    :param expectedFeatures: Expected build-feature values for the wheel variant.
    """
    queriedFeatures = []

    class FakeRustModule:
        pass

    def hasBuildFeature(name):
        queriedFeatures.append(name)
        return expectedFeatures[name]

    fakeBasilisk = ModuleType("Basilisk")
    fakeBasilisk.__file__ = "fake/Basilisk/__init__.py"
    fakeBasilisk.hasBuildFeature = hasBuildFeature
    fakeBasilisk.getBuildInfo = lambda: {
        "diagnostics": {"tools": {"corrosion": "test-version"}}
    }

    fakeRustModuleApi = ModuleType("Basilisk.moduleTemplates.rustModuleTemplate")
    fakeRustModuleApi.rustModuleTemplate = FakeRustModule

    monkeypatch.setitem(sys.modules, "Basilisk", fakeBasilisk)
    monkeypatch.setitem(
        sys.modules,
        "Basilisk.moduleTemplates.rustModuleTemplate",
        fakeRustModuleApi,
    )

    script = WHEEL_TESTER.import_check_script([], [], expectedFeatures)
    exec(script, {})

    assert queriedFeatures == list(expectedFeatures)


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

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

import pytest

import Basilisk._buildInfo as buildInfoFormatter


REPO_ROOT = Path(__file__).parents[2]
WHEEL_BUILDER_PATH = REPO_ROOT / ".github/scripts/build_optional_bsk_wheel.py"


def _loadWheelBuilder():
    moduleName = "bskOptionalWheelBuilderUnderTest"
    spec = importlib.util.spec_from_file_location(moduleName, WHEEL_BUILDER_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[moduleName] = module
    spec.loader.exec_module(module)
    return module


WHEEL_BUILDER = _loadWheelBuilder()


def _buildInfoData(opNavEnabled, *, schemaVersion=3):
    buildInfo = {
        "schemaVersion": schemaVersion,
        "artifact": {"basiliskVersion": "test-version"},
        "features": {
            "vizInterface": True,
            "opNav": opNavEnabled,
            "mujoco": True,
        },
    }
    return f"buildInfoData = {buildInfo!r}\n".encode("utf-8")


@contextmanager
def _buildInfoArchive(data):
    with io.BytesIO() as buffer:
        with zipfile.ZipFile(buffer, "w") as archive:
            archive.writestr(WHEEL_BUILDER.BUILD_INFO_DATA_PATH, data)
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


@pytest.mark.parametrize(
    "componentData, errorText",
    [
        pytest.param(
            _buildInfoData(False),
            "disabled in the base build and enabled in the component build",
            id="feature-not-enabled",
        ),
        pytest.param(
            _buildInfoData(True, schemaVersion=4),
            "beyond the expected 'opNav' feature value",
            id="unrelated-metadata-drift",
        ),
        pytest.param(
            (
                b'buildInfoData = {"schemaVersion": 3, "artifact": '
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
        recordData = archive.read(f"{optionalDistInfo}/RECORD").decode("utf-8")
        installPath = tmp_path / "installed"
        archive.extractall(installPath)

    assert entryPointData == "[basilisk.build_features]\nopNav = Basilisk\n"
    assert entryPointPath in recordData

    installedDistributions = list(distributions(path=[installPath]))
    monkeypatch.setitem(buildInfoFormatter._buildInfoData["features"], "opNav", False)
    monkeypatch.setattr(
        buildInfoFormatter,
        "_distributions",
        lambda: installedDistributions,
    )
    buildInfoFormatter._installedBuildFeatureProviders.cache_clear()
    try:
        assert buildInfoFormatter.hasBuildFeature("opNav") is True
    finally:
        buildInfoFormatter._installedBuildFeatureProviders.cache_clear()

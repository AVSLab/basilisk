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

"""Exercise the guards that prevent publishing unrelated recovery artifacts."""

import importlib.util
import io
from pathlib import Path
import tarfile
import zipfile

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "recover_wheel_release.py"
SPEC = importlib.util.spec_from_file_location("recover_wheel_release", SCRIPT)
recovery = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recovery)


@pytest.fixture
def original():
    """Represent a failed source check after successful wheel packaging."""
    run = {"head_branch": "v2.12.1", "head_sha": "abc123", "event": "push",
           "status": "completed", "path": ".github/workflows/publish-wheels.yml"}
    jobs = [{"name": "Make SDist", "conclusion": "success"},
            {"name": "Test SDist", "conclusion": "failure"},
            {"name": "Publish", "conclusion": "skipped"}]
    for platform in recovery.PLATFORMS:
        jobs.append({"name": f"Test and Package Wheels ({platform})", "conclusion": "success"})
        for kind in ("core", "opnav-full"):
            jobs.append({"name": f"Build Wheel Input ({kind}, {platform})", "conclusion": "success"})
    artifacts = [{"name": name, "id": index, "expired": False}
                 for index, name in enumerate(sorted(recovery.WHEEL_ARTIFACTS | {"cibw-sdist"}), 1)]
    return run, "abc123", "2.12.1\n", jobs, artifacts


def test_accepts_original_tested_artifacts(original):
    """Select only final distribution artifacts from the validated run."""
    outputs = recovery.validate_run(*original)
    assert outputs["tag"] == "v2.12.1"
    assert outputs["version"] == "2.12.1"
    assert len(outputs["wheel_artifact_ids"].split(",")) == 4


@pytest.mark.parametrize("field,value", [
    ("head_branch", "develop"), ("head_branch", "v2.12.1rc1"),
    ("head_sha", "wrong-commit"), ("event", "pull_request"),
    ("status", "in_progress"), ("path", ".github/workflows/unrelated.yml"),
])
def test_rejects_unrelated_or_moved_tag_run(original, field, value):
    """Never recover a different commit, prerelease, or untrusted workflow."""
    original[0][field] = value
    with pytest.raises(ValueError):
        recovery.validate_run(*original)


def test_rejects_wrong_source_version(original):
    """A final-looking tag on beta source must not reach publication."""
    with pytest.raises(ValueError, match="source version"):
        recovery.validate_run(original[0], original[1], "2.13.0b0", original[3], original[4])


@pytest.mark.parametrize("job_name,conclusion", [
    ("Publish", "success"), ("Publish", "failure"),
    ("Test SDist", "success"), ("Make SDist", "failure"),
    ("Build Wheel Input (core, macos-26)", "failure"),
    ("Test and Package Wheels (ubuntu-24.04)", "skipped"),
])
def test_requires_successful_wheel_checks_and_no_prior_publish(original, job_name, conclusion):
    """Reject partial validation and any run that already attempted publication."""
    next(job for job in original[3] if job["name"] == job_name)["conclusion"] = conclusion
    with pytest.raises(ValueError):
        recovery.validate_run(*original)


@pytest.mark.parametrize("change", ["missing", "expired", "unexpected"])
def test_rejects_unusable_artifacts(original, change):
    """Require the complete, unexpired platform artifact inventory."""
    if change == "missing":
        original[4].pop()
    elif change == "expired":
        original[4][0]["expired"] = True
    else:
        original[4].append({"name": "cibw-wheels-unexpected", "id": 99, "expired": False})
    with pytest.raises(ValueError):
        recovery.validate_run(*original)


@pytest.fixture
def distributions(tmp_path):
    """Create minimal wheel metadata and a source archive for all release platforms."""
    for platform in ("macosx_11_0_arm64", "manylinux_2_28_x86_64", "manylinux_2_28_aarch64", "win_amd64"):
        for name in ("bsk", "bsk_opnav"):
            path = tmp_path / f"{name}-2.12.1-cp39-abi3-{platform}.whl"
            with zipfile.ZipFile(path, "w") as wheel:
                wheel.writestr(f"{name}-2.12.1.dist-info/METADATA",
                               f"Name: {name}\nVersion: 2.12.1\n")
    content = b"Name: bsk\nVersion: 2.12.1\n"
    with tarfile.open(tmp_path / "bsk-2.12.1.tar.gz", "w:gz") as source:
        member = tarfile.TarInfo("bsk-2.12.1/PKG-INFO")
        member.size = len(content)
        source.addfile(member, io.BytesIO(content))
    return tmp_path


def test_accepts_complete_distribution_set(distributions):
    """The expected package pairs and matching embedded versions may be published."""
    recovery.validate_files(distributions, "2.12.1")


@pytest.mark.parametrize("change", ["missing-wheel", "extra-file", "wrong-version", "wrong-metadata"])
def test_rejects_incomplete_or_mislabeled_distributions(distributions, change):
    """Filenames alone must not hide missing packages or a different release."""
    wheel = next(distributions.glob("bsk-*.whl"))
    if change == "missing-wheel":
        wheel.unlink()
    elif change == "extra-file":
        (distributions / "unreviewed.whl").write_bytes(b"")
    elif change == "wrong-version":
        wheel.rename(wheel.with_name(wheel.name.replace("2.12.1", "2.12.0")))
    else:
        with zipfile.ZipFile(wheel, "w") as archive:
            archive.writestr("bsk-2.12.1.dist-info/METADATA", "Name: bsk\nVersion: 2.13.0b0\n")
    with pytest.raises(ValueError):
        recovery.validate_files(distributions, "2.12.1")

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

"""Validate existing release artifacts before recovering a failed SDist check."""

import argparse
import base64
from email.parser import BytesParser
import json
import os
from pathlib import Path
import re
import subprocess
import tarfile
import zipfile

from packaging.utils import canonicalize_name, parse_sdist_filename, parse_wheel_filename
from packaging.version import Version


PLATFORMS = ("macos-26", "ubuntu-24.04", "ubuntu-24.04-arm", "windows-2025-vs2026")
WHEEL_ARTIFACTS = {f"cibw-wheels-{platform}" for platform in PLATFORMS}


def require(condition: bool, message: str) -> None:
    """Reject recovery before publication if an invariant is not satisfied."""
    if not condition:
        raise ValueError(message)


def validate_run(run: dict, tag_sha: str, source_version: str,
                 jobs: list[dict], artifacts: list[dict]) -> dict[str, str]:
    """Require an unchanged final tag and successful wheel validation jobs."""
    tag = run["head_branch"]
    require(re.fullmatch(r"v\d+\.\d+\.\d+", tag) is not None,
            "Recovery requires a final version tag, such as v2.12.1")
    require(run["event"] == "push" and run["status"] == "completed",
            "The original run must be a completed tag-push run")
    require(run["path"] == ".github/workflows/publish-wheels.yml",
            "Artifacts must come from the Publish Wheels workflow")
    require(run["head_sha"] == tag_sha, "The release tag no longer matches the original run")
    require(source_version.strip() == tag[1:], "The tagged source version does not match the tag")

    by_name = {job["name"]: job for job in jobs}
    require(len(by_name) == len(jobs), "Unexpected duplicate job names")
    expected_jobs = {"Make SDist"}
    for platform in PLATFORMS:
        expected_jobs.add(f"Test and Package Wheels ({platform})")
        for kind in ("core", "opnav-full"):
            expected_jobs.add(f"Build Wheel Input ({kind}, {platform})")
    for name in expected_jobs:
        require(by_name.get(name, {}).get("conclusion") == "success",
                f"Original job did not succeed: {name}")
    require(by_name.get("Test SDist", {}).get("conclusion") == "failure",
            "Recovery is limited to failed source-distribution validation")
    require(by_name.get("Publish", {}).get("conclusion") == "skipped",
            "The original publishing job must not have run")

    expected_artifacts = WHEEL_ARTIFACTS | {"cibw-sdist"}
    selected = {artifact["name"]: artifact for artifact in artifacts
                if artifact["name"] in expected_artifacts}
    require(set(selected) == expected_artifacts, "Required release artifacts are missing")
    require({artifact["name"] for artifact in artifacts
             if artifact["name"].startswith("cibw-wheels-")} == WHEEL_ARTIFACTS,
            "Unexpected wheel artifact inventory")
    require(all(not artifact["expired"] for artifact in selected.values()),
            "Release artifacts have expired")
    return {
        "tag": tag,
        "version": tag[1:],
        "wheel_artifact_ids": ",".join(str(selected[name]["id"]) for name in sorted(WHEEL_ARTIFACTS)),
        "sdist_artifact_id": str(selected["cibw-sdist"]["id"]),
    }


def github_api(endpoint: str, *, paginated: bool = False):
    """Read GitHub metadata using the workflow's scoped token through gh."""
    command = ["gh", "api", endpoint]
    if paginated:
        command.extend(["--paginate", "--slurp"])
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    return json.loads(result.stdout)


def check_original_run(run_id: str, repository: str) -> dict[str, str]:
    """Resolve the remote tag and source version without changing the checkout."""
    require(run_id.isdigit(), "The original run ID must be numeric")
    require(re.fullmatch(r"[\w.-]+/[\w.-]+", repository) is not None, "Invalid repository")
    root = f"repos/{repository}"
    run = github_api(f"{root}/actions/runs/{run_id}")
    require(run["repository"]["full_name"] == repository
            and run["head_repository"]["full_name"] == repository,
            "The original run must belong to this repository")
    tag = run["head_branch"]
    require(re.fullmatch(r"v\d+\.\d+\.\d+", tag) is not None, "Expected a final version tag")
    target = github_api(f"{root}/git/ref/tags/{tag}")["object"]
    for _ in range(8):
        if target["type"] != "tag":
            break
        target = github_api(f"{root}/git/tags/{target['sha']}")["object"]
    require(target["type"] == "commit", "The release tag must resolve to a commit")
    version_file = github_api(f"{root}/contents/docs/source/bskVersion.txt?ref={target['sha']}")
    require(version_file["encoding"] == "base64", "Unexpected version-file encoding")
    version = base64.b64decode(version_file["content"]).decode("utf-8")
    jobs = [job for page in github_api(f"{root}/actions/runs/{run_id}/jobs?per_page=100", paginated=True)
            for job in page["jobs"]]
    artifacts = [artifact for page in github_api(f"{root}/actions/runs/{run_id}/artifacts?per_page=100",
                                                paginated=True) for artifact in page["artifacts"]]
    outputs = validate_run(run, target["sha"], version, jobs, artifacts)
    print(f"Validated {tag} at {target['sha']} against original run {run_id}")
    return outputs


def check_metadata(data: bytes, name: str, version: Version) -> None:
    """Require embedded package metadata to match the selected release."""
    metadata = BytesParser().parsebytes(data)
    require(canonicalize_name(metadata.get("Name", "")) == name, "Package metadata name mismatch")
    require(Version(metadata.get("Version", "0")) == version, "Package metadata version mismatch")


def validate_files(directory: Path, version_string: str) -> None:
    """Check both packages for every platform and the source archive's metadata."""
    version = Version(version_string)
    require(not version.is_prerelease and not version.is_devrelease and version.local is None,
            "Recovery requires a final release version")
    files = list(directory.iterdir())
    wheels = list(directory.glob("*.whl"))
    sources = list(directory.glob("*.tar.gz"))
    require(len(wheels) == 2 * len(PLATFORMS) and len(sources) == 1
            and set(files) == set(wheels + sources), "Unexpected distribution file inventory")
    wheel_groups = {}
    for wheel in wheels:
        name, wheel_version, _, tags = parse_wheel_filename(wheel.name)
        require(name in {"bsk", "bsk-opnav"} and wheel_version == version,
                f"Unexpected wheel: {wheel.name}")
        group = wheel_groups.setdefault(tags, set())
        require(name not in group, f"Duplicate wheel platform: {wheel.name}")
        group.add(name)
        with zipfile.ZipFile(wheel) as archive:
            metadata = [name for name in archive.namelist() if name.endswith(".dist-info/METADATA")]
            require(len(metadata) == 1, f"Missing or ambiguous metadata in {wheel.name}")
            check_metadata(archive.read(metadata[0]), name, version)
    require(len(wheel_groups) == len(PLATFORMS)
            and all(names == {"bsk", "bsk-opnav"} for names in wheel_groups.values()),
            "Every platform must contain matching bsk and bsk-opnav wheels")
    name, source_version = parse_sdist_filename(sources[0].name)
    require(name == "bsk" and source_version == version, "Source archive version mismatch")
    with tarfile.open(sources[0]) as archive:
        metadata = [member for member in archive.getmembers()
                    if len(Path(member.name).parts) == 2 and member.name.endswith("/PKG-INFO")
                    and member.isfile()]
        require(len(metadata) == 1, "Missing or ambiguous source archive metadata")
        check_metadata(archive.extractfile(metadata[0]).read(), "bsk", version)
    print(f"Validated {len(wheels)} original wheels and the bsk {version} source archive")


def main() -> None:
    """Validate provenance before downloading, then validate downloaded distributions."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("run_id")
    run.add_argument("--repository", default=os.environ.get("GITHUB_REPOSITORY", ""))
    files = commands.add_parser("files")
    files.add_argument("directory", type=Path)
    files.add_argument("version")
    args = parser.parse_args()
    if args.command == "run":
        outputs = check_original_run(args.run_id, args.repository)
        if os.environ.get("GITHUB_OUTPUT"):
            with Path(os.environ["GITHUB_OUTPUT"]).open("a", encoding="utf-8") as stream:
                for key, value in outputs.items():
                    stream.write(f"{key}={value}\n")
        else:
            print(json.dumps(outputs, indent=2))
    else:
        validate_files(args.directory, args.version)


if __name__ == "__main__":
    main()

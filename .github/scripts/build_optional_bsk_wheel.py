#!/usr/bin/env python3
"""Build optional Basilisk component wheels from full-wheel deltas.

 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""

from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import io
import re
import sys
import zipfile
from dataclasses import dataclass
from email.parser import Parser
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VERSION_FILE = REPO_ROOT / "docs/source/bskVersion.txt"
BASE_DISTRIBUTION = "bsk"
NATIVE_PAYLOAD_SUFFIXES = (".a", ".dll", ".dylib", ".lib", ".pyd", ".so")
NATIVE_PAYLOAD_PREFIXES = ("bsk.libs/",)
PRE_RELEASE_ALIASES = {
    "a": "a",
    "alpha": "a",
    "b": "b",
    "beta": "b",
    "c": "rc",
    "pre": "rc",
    "preview": "rc",
    "rc": "rc",
}
PRE_RELEASE_VERSION_PATTERN = re.compile(
    r"^(?P<release>\d+(?:\.\d+)*)(?:[-_.]?"
    r"(?P<label>a|alpha|b|beta|c|pre|preview|rc)(?P<number>\d*))$",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class OptionalComponent:
    distribution: str
    summary: str
    description: str
    cmake_include: str
    support_files: tuple[tuple[str, re.Pattern[str]], ...] = ()


COMPONENTS = {
    "opnav": OptionalComponent(
        distribution="bsk-opnav",
        summary="Basilisk optical navigation extension modules",
        description="Optional optical navigation extension modules for Basilisk.",
        # Basilisk treats OpenCV-backed modules as optical navigation modules.
        cmake_include="usingOpenCV",
        support_files=(
            (
                "Windows dependency DLL",
                re.compile(r"bsk\.libs/[^/]+-[0-9a-f]{32}\.dll"),
            ),
        ),
    ),
}

MODULE_PACKAGE_ROOTS = {
    REPO_ROOT / "src/fswAlgorithms": "Basilisk/fswAlgorithms",
    REPO_ROOT / "src/simulation": "Basilisk/simulation",
}


def normalize_distribution_name(name: str) -> str:
    return re.sub(r"[-_.]+", "_", name).lower()


def canonicalize_version(version: str) -> str:
    """Return the canonical spelling for supported PEP 440 prereleases."""
    stripped = version.strip()
    match = PRE_RELEASE_VERSION_PATTERN.fullmatch(stripped)
    if match is None:
        return stripped

    label = PRE_RELEASE_ALIASES[match.group("label").lower()]
    number = int(match.group("number") or "0")
    return f"{match.group('release')}{label}{number}"


def read_version(version_file: Path) -> str:
    version = version_file.read_text(encoding="utf-8").strip()
    if not version:
        raise ValueError(f"Version file is empty: {version_file}")
    return canonicalize_version(version)


def package_root_for_module(module_dir: Path) -> str:
    for source_root, package_root in MODULE_PACKAGE_ROOTS.items():
        try:
            module_dir.relative_to(source_root)
        except ValueError:
            continue
        return package_root

    raise ValueError(f"Do not know Python package root for optional module: {module_dir}")


def discover_cmake_include_modules(cmake_include: str) -> list[Path]:
    include_pattern = re.compile(rf"include\s*\(\s*{re.escape(cmake_include)}\s*\)")
    modules = []
    for custom_cmake in (REPO_ROOT / "src").rglob("Custom.cmake"):
        if include_pattern.search(custom_cmake.read_text(encoding="utf-8")):
            modules.append(custom_cmake.parent)

    if not modules:
        raise ValueError(f"No Basilisk modules include {cmake_include!r}.")

    return sorted(modules)


def module_files_for_component(
    component: OptionalComponent,
    *,
    include_libraries: bool = False,
) -> tuple[tuple[str, re.Pattern[str]], ...]:
    module_files = []
    for module_dir in discover_cmake_include_modules(component.cmake_include):
        target = module_dir.name
        package_root = package_root_for_module(module_dir)
        module_files.extend([
            (
                f"{target}.py",
                re.compile(rf"{re.escape(package_root)}/{re.escape(target)}\.py"),
            ),
            (
                f"_{target}",
                re.compile(rf"{re.escape(package_root)}/_{re.escape(target)}\.(so|pyd)"),
            ),
        ])
        if include_libraries:
            module_files.append((
                f"{target}.lib",
                re.compile(rf"{re.escape(package_root)}/{re.escape(target)}\.lib"),
            ))

    return tuple(module_files)


def expected_files_for_component(component: OptionalComponent) -> tuple[tuple[str, re.Pattern[str]], ...]:
    return module_files_for_component(component)


def allowed_files_for_component(component: OptionalComponent) -> tuple[tuple[str, re.Pattern[str]], ...]:
    allowed_files = list(module_files_for_component(component, include_libraries=True))
    allowed_files.extend(component.support_files)
    return tuple(allowed_files)


def optional_module_files_for_component(
    component: OptionalComponent,
) -> tuple[tuple[str, re.Pattern[str]], ...]:
    return module_files_for_component(component, include_libraries=True)


def find_dist_info_file(names: list[str], suffix: str) -> str:
    matches = [name for name in names if name.endswith(f".dist-info/{suffix}")]
    if len(matches) != 1:
        raise ValueError(f"Expected one .dist-info/{suffix} file, found {len(matches)}.")
    return matches[0]


def read_metadata(archive: zipfile.ZipFile) -> dict[str, list[str]]:
    metadata_name = find_dist_info_file(archive.namelist(), "METADATA")
    raw_metadata = archive.read(metadata_name).decode("utf-8")
    message = Parser().parsestr(raw_metadata)

    metadata: dict[str, list[str]] = {}
    for key, value in message.items():
        metadata.setdefault(key, []).append(value)
    return metadata


def first_metadata_value(metadata: dict[str, list[str]], key: str) -> str | None:
    values = metadata.get(key)
    if not values:
        return None
    return values[0]


def wheel_tag_suffix(wheel: Path, distribution: str, version: str) -> str:
    prefix = f"{normalize_distribution_name(distribution)}-{version}-"
    suffix = wheel.name.removesuffix(".whl")
    if not suffix.startswith(prefix):
        raise ValueError(
            f"Wheel filename {wheel.name!r} does not start with expected prefix {prefix!r}."
        )
    return suffix[len(prefix):]


def payload_names(archive: zipfile.ZipFile) -> set[str]:
    return {
        info.filename
        for info in archive.infolist()
        if not info.filename.endswith("/") and ".dist-info/" not in info.filename
    }


def archive_payload_digest(archive: zipfile.ZipFile, name: str) -> str:
    return hashlib.sha256(archive.read(name)).hexdigest()


def is_native_payload(name: str) -> bool:
    normalized = name.lower()
    return normalized.startswith(NATIVE_PAYLOAD_PREFIXES) or normalized.endswith(
        NATIVE_PAYLOAD_SUFFIXES
    )


def warn_changed_native_payloads(changed: list[str]) -> None:
    if not changed:
        return

    shown = changed[:10]
    details = "\n  ".join(shown)
    remaining = len(changed) - len(shown)
    if remaining:
        details += f"\n  ... {remaining} more"

    print(
        "warning: common native payload files differ between the base and "
        f"component builds; continuing with {len(changed)} changed native files:\n"
        f"  {details}",
        file=sys.stderr,
    )


def validate_common_payloads(
    base_archive: zipfile.ZipFile,
    component_archive: zipfile.ZipFile,
    base_payload: set[str],
    component_payload: set[str],
) -> None:
    changed = []
    for name in sorted(base_payload & component_payload):
        base_digest = archive_payload_digest(base_archive, name)
        component_digest = archive_payload_digest(component_archive, name)
        if base_digest != component_digest:
            changed.append(name)

    native_changed = [name for name in changed if is_native_payload(name)]
    non_native_changed = [name for name in changed if not is_native_payload(name)]
    warn_changed_native_payloads(native_changed)

    if non_native_changed:
        details = "\n  ".join(non_native_changed)
        raise ValueError(
            "Component build changes non-native files that already exist in the base wheel:\n"
            f"  {details}"
        )


def validate_delta(
    base_payload: set[str],
    component_payload: set[str],
    component: OptionalComponent,
) -> list[str]:
    expected_files = expected_files_for_component(component)
    allowed_files = allowed_files_for_component(component)
    optional_module_files = optional_module_files_for_component(component)
    optional_in_base = sorted(
        name
        for name in base_payload
        if any(pattern.fullmatch(name) for _, pattern in optional_module_files)
    )
    if optional_in_base:
        details = "\n  ".join(optional_in_base)
        raise ValueError(f"Base wheel already contains optional files:\n  {details}")

    delta = sorted(component_payload - base_payload)
    unexpected = sorted(
        name
        for name in delta
        if not any(pattern.fullmatch(name) for _, pattern in allowed_files)
    )
    if unexpected:
        details = "\n  ".join(unexpected)
        raise ValueError(f"Component wheel delta contains unexpected files:\n  {details}")

    missing = [
        label
        for label, pattern in expected_files
        if not any(pattern.fullmatch(name) for name in delta)
    ]
    if missing:
        details = "\n  ".join(missing)
        raise ValueError(f"Component wheel delta is missing expected files:\n  {details}")

    return delta


def digest_record(data: bytes) -> tuple[str, str]:
    digest = hashlib.sha256(data).digest()
    encoded = base64.urlsafe_b64encode(digest).rstrip(b"=").decode("ascii")
    return f"sha256={encoded}", str(len(data))


def add_bytes(
    archive: zipfile.ZipFile,
    records: list[tuple[str, str, str]],
    name: str,
    data: bytes,
) -> None:
    archive.writestr(name, data, compress_type=zipfile.ZIP_DEFLATED)
    records.append((name, *digest_record(data)))


def build_metadata(
    base_metadata: dict[str, list[str]],
    component: OptionalComponent,
    version: str,
    include_license_file: bool,
) -> bytes:
    lines = [
        "Metadata-Version: 2.4",
        f"Name: {component.distribution}",
        f"Version: {version}",
        f"Summary: {component.summary}",
    ]

    for field in ("Home-page", "License-Expression", "Requires-Python"):
        value = first_metadata_value(base_metadata, field)
        if value:
            lines.append(f"{field}: {value}")

    for value in base_metadata.get("Project-URL", []):
        lines.append(f"Project-URL: {value}")
    for value in base_metadata.get("Classifier", []):
        lines.append(f"Classifier: {value}")

    if include_license_file:
        lines.append("License-File: LICENSE")

    lines.extend([
        f"Requires-Dist: {BASE_DISTRIBUTION}=={version}",
        "Description-Content-Type: text/plain",
        "",
        component.description,
        "",
    ])
    return "\n".join(lines).encode("utf-8")


def build_wheel_file(
    component_wheel: Path,
    output_wheel: Path,
    component: OptionalComponent,
    base_metadata: dict[str, list[str]],
    payload: list[str],
    version: str,
) -> None:
    distribution = normalize_distribution_name(component.distribution)
    dist_info = f"{distribution}-{version}.dist-info"
    records: list[tuple[str, str, str]] = []

    with zipfile.ZipFile(component_wheel) as source:
        wheel_name = find_dist_info_file(source.namelist(), "WHEEL")
        wheel_lines = source.read(wheel_name).decode("utf-8").splitlines()
        wheel_metadata = [
            line if not line.startswith("Generator: ") else "Generator: build_optional_bsk_wheel.py"
            for line in wheel_lines
        ]
        wheel_data = ("\n".join(wheel_metadata) + "\n").encode("utf-8")

        license_name = next(
            (name for name in source.namelist() if name.endswith(".dist-info/licenses/LICENSE")),
            None,
        )
        license_data = source.read(license_name) if license_name else None
        metadata = build_metadata(
            base_metadata,
            component,
            version,
            include_license_file=license_data is not None,
        )

        with zipfile.ZipFile(output_wheel, "w", compression=zipfile.ZIP_DEFLATED) as output:
            for name in payload:
                add_bytes(output, records, name, source.read(name))

            if license_data is not None:
                add_bytes(output, records, f"{dist_info}/licenses/LICENSE", license_data)

            add_bytes(output, records, f"{dist_info}/METADATA", metadata)
            add_bytes(output, records, f"{dist_info}/WHEEL", wheel_data)
            add_bytes(output, records, f"{dist_info}/top_level.txt", b"Basilisk\n")

            record_name = f"{dist_info}/RECORD"
            output_text = io.StringIO()
            writer = csv.writer(output_text, lineterminator="\n")
            for record in records:
                writer.writerow(record)
            writer.writerow((record_name, "", ""))
            output.writestr(record_name, output_text.getvalue(), compress_type=zipfile.ZIP_DEFLATED)


def build_optional_wheel(
    component_name: str,
    base_wheel: Path,
    component_wheel: Path,
    dest_dir: Path,
    version_file: Path,
) -> Path:
    component = COMPONENTS[component_name]
    version = read_version(version_file)

    with zipfile.ZipFile(base_wheel) as base_archive, zipfile.ZipFile(component_wheel) as component_archive:
        base_metadata = read_metadata(base_archive)
        component_metadata = read_metadata(component_archive)

        for metadata_name, metadata in (
            ("base", base_metadata),
            ("component", component_metadata),
        ):
            wheel_version = first_metadata_value(metadata, "Version")
            canonical_wheel_version = (
                canonicalize_version(wheel_version) if wheel_version is not None else None
            )
            if canonical_wheel_version != version:
                raise ValueError(
                    f"The {metadata_name} wheel version {wheel_version!r} does not match "
                    f"{version_file}: {version!r}."
                )

        base_payload = payload_names(base_archive)
        component_payload = payload_names(component_archive)
        validate_common_payloads(
            base_archive,
            component_archive,
            base_payload,
            component_payload,
        )
        payload = validate_delta(base_payload, component_payload, component)

    tag_suffix = wheel_tag_suffix(component_wheel, BASE_DISTRIBUTION, version)
    output_name = f"{normalize_distribution_name(component.distribution)}-{version}-{tag_suffix}.whl"
    dest_dir.mkdir(parents=True, exist_ok=True)
    output_wheel = dest_dir / output_name
    build_wheel_file(component_wheel, output_wheel, component, base_metadata, payload, version)
    return output_wheel


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build an optional Basilisk component wheel from a full-wheel delta.",
    )
    parser.add_argument("--component", choices=sorted(COMPONENTS), required=True)
    parser.add_argument("--base-wheel", type=Path, required=True)
    parser.add_argument("--component-wheel", type=Path, required=True)
    parser.add_argument("--dest-dir", type=Path, required=True)
    parser.add_argument("--version-file", type=Path, default=DEFAULT_VERSION_FILE)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        output_wheel = build_optional_wheel(
            args.component,
            args.base_wheel,
            args.component_wheel,
            args.dest_dir,
            args.version_file,
        )
    except Exception as err:
        print(f"error: {err}", file=sys.stderr)
        return 1

    print(output_wheel)
    return 0


if __name__ == "__main__":
    sys.exit(main())

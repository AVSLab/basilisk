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

import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Optional


PROFILE_CONF_SETTINGS = {
    "tools.system.package_manager:mode": "install",
    "tools.system.package_manager:sudo": "True",
}


def _run_conan(*args: str, capture_output: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "-m", "conans.conan", *args],
        check=True,
        capture_output=capture_output,
        text=True,
    )


def _profile_setting_key(line: str) -> Optional[str]:
    stripped = line.strip()
    if not stripped or stripped.startswith("#") or "=" not in stripped:
        return None
    return stripped.split("=", 1)[0].strip()


def update_profile_conf(profile_text: str) -> str:
    """Set the required package-manager configuration in a Conan profile."""
    lines = profile_text.splitlines()
    updated_lines = []
    seen_keys = set()
    in_conf_section = False
    found_conf_section = False

    def append_missing_settings() -> None:
        for key, value in PROFILE_CONF_SETTINGS.items():
            if key not in seen_keys:
                updated_lines.append(f"{key}={value}")
                seen_keys.add(key)

    for line in lines:
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            if in_conf_section:
                append_missing_settings()
            in_conf_section = stripped == "[conf]"
            found_conf_section = found_conf_section or in_conf_section
            updated_lines.append(line)
            continue

        if in_conf_section:
            key = _profile_setting_key(line)
            if key in PROFILE_CONF_SETTINGS:
                if key not in seen_keys:
                    updated_lines.append(f"{key}={PROFILE_CONF_SETTINGS[key]}")
                    seen_keys.add(key)
                continue
        updated_lines.append(line)

    if in_conf_section:
        append_missing_settings()
    elif not found_conf_section:
        if updated_lines and updated_lines[-1].strip():
            updated_lines.append("")
        updated_lines.append("[conf]")
        append_missing_settings()

    return "\n".join(updated_lines) + "\n"


def configure_default_profile() -> Path:
    """Detect and configure the default Conan profile for the current runner."""
    _run_conan("profile", "detect", "--force")
    result = _run_conan("profile", "path", "default", capture_output=True)
    output_lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if not output_lines:
        raise RuntimeError("Conan did not report the default profile path")

    profile_path = Path(output_lines[-1])
    profile_text = profile_path.read_text(encoding="utf-8")
    profile_path.write_text(update_profile_conf(profile_text), encoding="utf-8")
    print(f"Configured Conan profile at {profile_path}")
    return profile_path


def cache_fingerprint(
    profile_text: str,
    conan_version: str,
    conan_args: str,
    runner_arch: str,
) -> str:
    """Return a stable cache fingerprint for a Conan build configuration."""
    context = {
        "conanArgs": shlex.split(conan_args),
        "conanProfile": profile_text,
        "conanVersion": conan_version,
        "runnerArch": runner_arch,
    }
    serialized_context = json.dumps(context, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(serialized_context.encode("utf-8")).hexdigest()


def _conan_version() -> str:
    result = _run_conan("--version", capture_output=True)
    return result.stdout.strip()


def _write_github_output(name: str, value: str) -> None:
    output_path = os.environ.get("GITHUB_OUTPUT")
    if not output_path:
        raise RuntimeError("GITHUB_OUTPUT is required when emitting the cache fingerprint")
    with Path(output_path).open("a", encoding="utf-8") as output_file:
        output_file.write(f"{name}={value}\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure Conan for a Basilisk CI build")
    parser.add_argument(
        "--emit-cache-fingerprint",
        action="store_true",
        help="write the current Conan build fingerprint to GITHUB_OUTPUT",
    )
    args = parser.parse_args()

    profile_path = configure_default_profile()
    if args.emit_cache_fingerprint:
        fingerprint = cache_fingerprint(
            profile_path.read_text(encoding="utf-8"),
            _conan_version(),
            os.environ.get("CONAN_ARGS", ""),
            os.environ.get("RUNNER_ARCH", ""),
        )
        _write_github_output("fingerprint", fingerprint)
        print(f"Conan cache fingerprint: {fingerprint}")


if __name__ == "__main__":
    main()

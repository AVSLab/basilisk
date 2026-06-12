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

from pathlib import Path
from typing import Optional


CONAN_DOWNLOAD_RETRIES = 1
CONAN_HTTP_RETRIES = 1
CONAN_RETRY_WAIT = 5  # [s]
CONAN_HTTP_TIMEOUT = 30  # [s]
CONAN_SOURCE_BACKUP_URL = "https://hanspeterschaub.info/bskFiles/backup/conan-sources"

CONAN_RETRY_SETTINGS = {
    "tools.files.download:retry": CONAN_DOWNLOAD_RETRIES,
    "tools.files.download:retry_wait": CONAN_RETRY_WAIT,
    "core.net.http:max_retries": CONAN_HTTP_RETRIES,
    "core.net.http:timeout": CONAN_HTTP_TIMEOUT,
    "core.sources:download_urls": ["origin", CONAN_SOURCE_BACKUP_URL],
}

GLOBAL_CONF_PATH = Path.home() / ".conan2" / "global.conf"


def _conf_key(line: str) -> Optional[str]:
    stripped = line.strip()
    if not stripped or stripped.startswith("#") or "=" not in stripped:
        return None
    return stripped.split("=", 1)[0].strip()


def update_global_conf(path: Path = GLOBAL_CONF_PATH) -> None:
    """Set Conan retry and source-backup configuration in ``global.conf``."""
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = path.read_text(encoding="utf-8").splitlines() if path.exists() else []

    updated_lines = []
    seen_keys = set()
    for line in lines:
        key = _conf_key(line)
        if key in CONAN_RETRY_SETTINGS:
            if key in seen_keys:
                continue
            updated_lines.append(f"{key}={CONAN_RETRY_SETTINGS[key]}")
            seen_keys.add(key)
        else:
            updated_lines.append(line)

    for key, value in CONAN_RETRY_SETTINGS.items():
        if key not in seen_keys:
            updated_lines.append(f"{key}={value}")

    path.write_text("\n".join(updated_lines) + "\n", encoding="utf-8")
    print(f"Configured Conan download retries and source backups in {path}")


if __name__ == "__main__":
    update_global_conf()

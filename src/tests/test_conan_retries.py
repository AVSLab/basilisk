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

"""Test Conan retry configuration and source backups with isolated caches."""

import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from threading import Thread

import pytest


SCRIPT = Path(__file__).resolve().parents[2] / ".github/scripts/configure_conan_retries.py"


@pytest.fixture
def retry_module(tmp_path, monkeypatch):
    """Load the helper without allowing it to write to the user's Conan home."""
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path / "user"))
    monkeypatch.delenv("CONAN_HOME", raising=False)
    spec = importlib.util.spec_from_file_location("configure_conan_retries", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("custom_home", [False, True])
def test_configuration_home(retry_module, tmp_path, monkeypatch, capsys, custom_home):
    """Configure the selected home, leaving an unused default cache untouched."""
    default_conf = Path.home() / ".conan2/global.conf"
    default_conf.parent.mkdir(parents=True)
    default_conf.write_text("# Existing default configuration\n", encoding="utf-8")
    expected_conf = default_conf
    if custom_home:
        # Set this after importing to ensure selection happens when called.
        conan_home = tmp_path / "mounted cache"
        monkeypatch.setenv("CONAN_HOME", str(conan_home))
        expected_conf = conan_home / "global.conf"

    retry_module.update_global_conf()

    contents = expected_conf.read_text(encoding="utf-8")
    assert "tools.files.download:retry=1\n" in contents
    assert "tools.files.download:retry_wait=5\n" in contents
    assert "core.net.http:max_retries=1\n" in contents
    assert "core.net.http:timeout=30\n" in contents
    assert ("core.sources:download_urls=['origin', "
            "'https://hanspeterschaub.info/bskFiles/backup/conan-sources']\n") in contents
    assert str(expected_conf) in capsys.readouterr().out
    if custom_home:
        assert default_conf.read_text(encoding="utf-8") == "# Existing default configuration\n"


@pytest.mark.parametrize("conan_home", ["", "relative-cache"])
def test_invalid_home_is_rejected(retry_module, tmp_path, monkeypatch, conan_home):
    """Reject paths Conan cannot use before creating configuration files."""
    monkeypatch.setenv("CONAN_HOME", conan_home)
    monkeypatch.chdir(tmp_path)
    with pytest.raises(ValueError, match="CONAN_HOME"):
        retry_module.update_global_conf()
    assert list(tmp_path.iterdir()) == []


def test_preserves_configuration_and_explicit_path(retry_module, tmp_path, monkeypatch):
    """Preserve unrelated settings and comments while replacing duplicate keys once."""
    monkeypatch.setenv("CONAN_HOME", str(tmp_path / "unused-cache"))
    path = tmp_path / "explicit/global.conf"
    path.parent.mkdir()
    preserved = (
        "# Keep package-manager configuration\n"
        "tools.system.package_manager:mode=install\n\n"
        "tools.system.package_manager:sudo=False\n"
    )
    path.write_text(
        preserved + " tools.files.download:retry = 9\n"
        "tools.files.download:retry=8\n"
        "core.sources:download_urls=['origin']\n",
        encoding="utf-8",
    )

    retry_module.update_global_conf(path)
    first_update = path.read_text(encoding="utf-8")
    retry_module.update_global_conf(path)

    assert path.read_text(encoding="utf-8") == first_update
    assert first_update.startswith(preserved)
    assert first_update.count("tools.files.download:retry=") == 1
    assert "tools.files.download:retry=1\n" in first_update
    assert first_update.count("core.sources:download_urls=") == 1
    assert not (tmp_path / "unused-cache").exists()
    assert not Path.home().exists()


@pytest.mark.parametrize("corrupt_backup", [False, True])
def test_conan_source_backup(retry_module, tmp_path, monkeypatch, corrupt_backup):
    """Use a backup after origin failure, verify its checksum, and reuse cached sources.

    A loopback server supplies a missing origin and a checksum-addressed backup.
    The test uses real Conan without external downloads or a compiler.
    """
    pytest.importorskip("conan")
    payload = b"Basilisk source backup fixture\n"
    checksum = hashlib.sha256(payload).hexdigest()
    requests = []
    responses = {}

    class SourceHandler(BaseHTTPRequestHandler):
        """Serve only fixture sources and record each requested path."""

        def do_GET(self):
            """Return a fixture response or an HTTP 404 for the origin."""
            requests.append(self.path)
            content = responses.get(self.path)
            self.send_response(404 if content is None else 200)
            self.end_headers()
            if content is not None:
                self.wfile.write(content)

        def log_message(self, format_string, *args):
            """Keep expected HTTP failures out of test output."""

    server = ThreadingHTTPServer(("127.0.0.1", 0), SourceHandler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        server_url = f"http://127.0.0.1:{server.server_port}"
        origin_url = f"{server_url}/missing-source.txt"
        responses[f"/backup/{checksum}"] = b"corrupted" if corrupt_backup else payload
        responses[f"/backup/{checksum}.json"] = json.dumps({
            "references": {"backup-fixture/1.0": [origin_url]},
        }).encode("utf-8")
        monkeypatch.setenv("CONAN_HOME", str(tmp_path / "mounted cache"))
        monkeypatch.setenv("NO_PROXY", "127.0.0.1")
        monkeypatch.setenv("no_proxy", "127.0.0.1")
        monkeypatch.setitem(retry_module.CONAN_RETRY_SETTINGS, "core.sources:download_urls",
                            ["origin", f"{server_url}/backup"])
        # Keep deliberate failures immediate; retry timing is covered by the config test.
        monkeypatch.setitem(retry_module.CONAN_RETRY_SETTINGS, "tools.files.download:retry", 0)
        monkeypatch.setitem(retry_module.CONAN_RETRY_SETTINGS, "core.net.http:max_retries", 0)
        retry_module.update_global_conf()

        recipe = tmp_path / "conanfile.py"
        recipe.write_text(
            "from conan import ConanFile\n"
            "from conan.tools.files import download\n\n"
            "class BackupFixture(ConanFile):\n"
            "    name = 'backup-fixture'\n"
            "    version = '1.0'\n\n"
            "    def source(self):\n"
            f"        download(self, {origin_url!r}, 'source.txt', sha256={checksum!r})\n",
            encoding="utf-8",
        )
        command = [sys.executable, "-m", "conans.conan", "source", str(tmp_path)]
        result = subprocess.run(command, cwd=tmp_path, env=os.environ.copy(),
                                capture_output=True, text=True, check=False,
                                timeout=30)  # [s]
        output = result.stdout + result.stderr
        assert "/missing-source.txt" in requests, output
        assert f"/backup/{checksum}" in requests, output
        if corrupt_backup:
            assert result.returncode != 0, output
            assert "sha256 hash failed" in output.lower(), output
            assert not (tmp_path / "source.txt").exists()
        else:
            assert result.returncode == 0, output
            assert f"/backup/{checksum}.json" in requests, output
            assert (tmp_path / "source.txt").read_bytes() == payload

            (tmp_path / "source.txt").unlink()
            requests.clear()
            cached = subprocess.run(command, cwd=tmp_path, env=os.environ.copy(),
                                    capture_output=True, text=True, check=False,
                                    timeout=30)  # [s]
            assert cached.returncode == 0, cached.stdout + cached.stderr
            assert (tmp_path / "source.txt").read_bytes() == payload
            assert requests == []
        assert not Path.home().exists()
    finally:
        server.shutdown()
        thread.join(timeout=5)  # [s]
        server.server_close()

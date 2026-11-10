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

"""Check persistent Matplotlib cache selection before plotting imports."""

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import conftest
import pytest


def test_matplotlib_cache_is_reused(tmp_path, monkeypatch):
    """Repeated test sessions select the same directory without deleting its contents."""
    cache = tmp_path / "cache" / "matplotlib"
    monkeypatch.delenv("MPLCONFIGDIR", raising=False)
    conftest._configure_matplotlib_cache(cache)
    assert os.environ["MPLCONFIGDIR"] == str(cache)
    cached_file = cache / "cached-data"
    cached_file.write_text("preserved", encoding="utf-8")
    monkeypatch.delenv("MPLCONFIGDIR")
    conftest._configure_matplotlib_cache(cache)
    assert os.environ["MPLCONFIGDIR"] == str(cache)
    assert cached_file.read_text(encoding="utf-8") == "preserved"
    assert list(cache.iterdir()) == [cached_file]


@pytest.mark.parametrize("configured", ["custom-cache", ""])
def test_matplotlib_cache_preserves_environment(tmp_path, monkeypatch, configured):
    """An explicit environment setting takes precedence without creating another cache."""
    cache = tmp_path / "unused"
    monkeypatch.setenv("MPLCONFIGDIR", configured)
    conftest._configure_matplotlib_cache(cache)
    assert os.environ["MPLCONFIGDIR"] == configured
    assert not cache.exists()


@pytest.mark.parametrize("failure", ["directory", "write"])
def test_matplotlib_cache_unwritable_checkout(tmp_path, monkeypatch, failure):
    """Directory creation or write failures leave Matplotlib free to choose its cache."""
    cache = tmp_path / "matplotlib"
    monkeypatch.delenv("MPLCONFIGDIR", raising=False)
    if failure == "directory":
        cache.write_text("a file blocks directory creation", encoding="utf-8")
    else:
        def denied_write(*args, **kwargs):
            raise PermissionError("read-only cache")

        monkeypatch.setattr(conftest.tempfile, "TemporaryFile", denied_write)
    conftest._configure_matplotlib_cache(cache)
    assert "MPLCONFIGDIR" not in os.environ


def test_matplotlib_font_cache_survives_new_processes(tmp_path):
    """Fresh Python processes reuse the actual font cache independently of their cwd."""
    configuration = Path(conftest.__file__).resolve()
    cache = configuration.parents[1] / ".pytest_cache" / "matplotlib"
    try:
        cache.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryFile(dir=cache):
            pass
    except OSError:
        pytest.skip("Font-cache integration requires a writable checkout")

    script = """
import json
import runpy
import sys
from pathlib import Path

runpy.run_path(sys.argv[1])
import matplotlib
from matplotlib import font_manager

cache = Path(matplotlib.get_cachedir())
print(json.dumps({"directory": str(cache), "files": {
    path.name: path.stat().st_mtime_ns for path in cache.glob("fontlist-v*.json")
}}))
"""
    environment = os.environ.copy()
    environment.pop("MPLCONFIGDIR", None)
    environment["MPLBACKEND"] = "Agg"
    results = []
    for _ in range(2):
        completed = subprocess.run(
            [sys.executable, "-c", script, str(configuration)], cwd=tmp_path,
            env=environment, check=True, capture_output=True, text=True,
        )
        results.append(json.loads(completed.stdout.splitlines()[-1]))
    assert results[0]["directory"] == str(cache)
    assert results[0]["files"]
    assert results[1] == results[0]

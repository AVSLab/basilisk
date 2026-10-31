#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

"""
Unit tests for Basilisk supportData fetching (Pooch + local fallback).
"""

import hashlib
import importlib.util
from pathlib import Path
from unittest.mock import Mock

import pytest
import requests

from Basilisk.utilities import bskLargeData
from Basilisk.utilities.supportDataTools import dataFetcher


class TransientDownloader:
    """Fail a fixed number of times before succeeding."""

    def __init__(self, failures_before_success: int):
        self.failures_before_success = failures_before_success
        self.calls = 0

    def __call__(self, url, output_file, pooch_obj, check_only=False):
        self.calls += 1
        if self.calls <= self.failures_before_success:
            raise RuntimeError("temporary download failure")
        return None


@pytest.fixture
def fake_fetch(monkeypatch, tmp_path):
    """Patch POOCH.fetch to always return a small dummy file."""
    dummy = tmp_path / "dummy.dat"
    dummy.write_text("test")

    monkeypatch.setattr(
        dataFetcher.POOCH,
        "fetch",
        lambda key, downloader=None: str(dummy),
    )
    monkeypatch.setattr(dataFetcher, "LOCAL_SUPPORT", None)

    return dummy


@pytest.fixture
def isolated_fetcher(monkeypatch, tmp_path):
    """Import fresh fetcher state with network requests forbidden."""
    def reject_request(*args, **kwargs):
        pytest.fail("Unexpected network request while accessing support data")

    monkeypatch.setattr(requests.sessions.Session, "request", reject_request)
    monkeypatch.setenv("BSK_SUPPORT_DATA_CACHE", str(tmp_path / "cache"))
    spec = importlib.util.spec_from_file_location(
        "isolated_data_fetcher", dataFetcher.__file__
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def recorded_downloads(isolated_fetcher, monkeypatch):
    """Use real Pooch cache handling with an in-memory HTTP response."""
    content = b"test support data\n"
    urls = []

    def download(url, output_file, pooch_obj, check_only=False):
        urls.append(url)
        Path(output_file).write_bytes(content)

    downloader_type = isolated_fetcher.RetryingHTTPDownloader
    monkeypatch.setattr(
        isolated_fetcher,
        "RetryingHTTPDownloader",
        lambda **kwargs: downloader_type(
            downloader_factory=lambda **options: download, **kwargs
        ),
    )
    monkeypatch.setattr(isolated_fetcher, "LOCAL_SUPPORT", None)
    return content, urls


def test_import_does_not_access_network(isolated_fetcher):
    """Importing the fetcher must initialize its registry without HTTP requests."""
    assert isolated_fetcher.POOCH.registry


def test_local_support_file_does_not_access_network(isolated_fetcher, monkeypatch, tmp_path):
    """A checkout's support file must be usable without a GitHub tag lookup."""
    file_enum = isolated_fetcher.DataFile.MagneticFieldData.WMM
    rel = isolated_fetcher.relpath(file_enum)
    local_root = tmp_path / "supportData"
    local_file = tmp_path / rel
    local_file.parent.mkdir(parents=True)
    local_file.write_text("local data")
    monkeypatch.setattr(isolated_fetcher, "LOCAL_SUPPORT", local_root)

    assert isolated_fetcher.get_path(file_enum) == local_file


def test_cached_support_file_does_not_access_network(isolated_fetcher, monkeypatch):
    """A valid Pooch cache entry must be returned without HTTP requests."""
    file_enum = isolated_fetcher.DataFile.MagneticFieldData.WMM
    rel = isolated_fetcher.relpath(file_enum)
    cached_file = isolated_fetcher.POOCH.abspath / rel
    cached_file.parent.mkdir(parents=True)
    content = b"cached data"
    cached_file.write_bytes(content)
    monkeypatch.setattr(isolated_fetcher, "LOCAL_SUPPORT", None)
    monkeypatch.setitem(isolated_fetcher.POOCH.registry, rel, hashlib.sha256(content).hexdigest())

    assert isolated_fetcher.get_path(file_enum) == cached_file


@pytest.mark.parametrize("cache_state", ["missing", "stale"])
@pytest.mark.parametrize("tag_status", [200, 404, "timeout"])
def test_remote_fetch_resolves_tag_once(
    isolated_fetcher, recorded_downloads, monkeypatch, cache_state, tag_status
):
    """Resolve release or develop URLs on download and reuse the tag result."""
    content, downloaded_urls = recorded_downloads
    file_enum = isolated_fetcher.DataFile.MagneticFieldData.WMM
    rel = isolated_fetcher.relpath(file_enum)
    another_rel = "supportData/another.dat"
    content_hash = hashlib.sha256(content).hexdigest()
    monkeypatch.setitem(isolated_fetcher.POOCH.registry, rel, content_hash)
    monkeypatch.setitem(isolated_fetcher.POOCH.registry, another_rel, content_hash)
    cached_file = isolated_fetcher.POOCH.abspath / rel
    if cache_state == "stale":
        cached_file.parent.mkdir(parents=True)
        cached_file.write_text("outdated data")

    if tag_status == "timeout":
        head = Mock(side_effect=requests.exceptions.Timeout("tag lookup timed out"))
    else:
        response = requests.Response()
        response.status_code = tag_status
        head = Mock(return_value=response)
    monkeypatch.setattr(requests, "head", head)

    assert isolated_fetcher.get_path(file_enum).read_bytes() == content
    assert isolated_fetcher.get_path(file_enum) == cached_file
    assert Path(isolated_fetcher.fetch_support_data(another_rel)).read_bytes() == content

    version = isolated_fetcher.DATA_VERSION if tag_status == 200 else "develop"
    base_url = f"https://raw.githubusercontent.com/AVSLab/basilisk/{version}/"
    assert downloaded_urls == [base_url + rel, base_url + another_rel]
    assert head.call_count == 1


def test_external_download_does_not_check_github(isolated_fetcher, recorded_downloads):
    """Explicit external data URLs must not require Basilisk's release tag."""
    content, downloaded_urls = recorded_downloads
    file_enum = isolated_fetcher.DataFile.EphemerisData.de430
    rel = isolated_fetcher.relpath(file_enum)

    assert isolated_fetcher.get_path(file_enum).read_bytes() == content
    assert downloaded_urls == [isolated_fetcher.EXTERNAL_KERNEL_URLS[rel]]


def test_all_enum_entries_are_in_registry():
    """Every DataFile enum value must correspond to a key in REGISTRY."""
    missing = []

    for category in dataFetcher.DataFile.__dict__.values():
        if isinstance(category, type) and hasattr(category, "__members__"):
            for enum_value in category:
                key = dataFetcher.relpath(enum_value)
                if key not in dataFetcher.REGISTRY:
                    missing.append(key)

    assert not missing, f"Registry missing entries: {missing}"


def test_get_path_uses_pooch_when_not_local(fake_fetch):
    """get_path() must return a Path backed by mocked Pooch."""
    path = dataFetcher.get_path(dataFetcher.DataFile.MagneticFieldData.WMM)

    assert isinstance(path, Path)
    assert path.exists(), "get_path() should return an existing dummy file"


def test_bsk_large_data_quiet_fetch_keeps_warnings_visible(monkeypatch):
    """quiet_fetch() must keep retry and backup warnings visible."""
    logger_levels = []

    def fake_fetch(rel):
        logger_levels.append(bskLargeData.pooch.utils.get_logger().level)
        return "dummy"

    monkeypatch.setattr(bskLargeData, "fetch_support_data", fake_fetch)

    assert bskLargeData.quiet_fetch("supportData/example.dat") == "dummy"

    assert logger_levels == [bskLargeData.logging.WARNING]


def test_fetch_support_data_uses_retrying_downloader(monkeypatch, tmp_path):
    """fetch_support_data() must pass a retrying downloader to Pooch."""
    dummy = tmp_path / "dummy.dat"
    dummy.write_text("test")
    captured = {}

    def fake_pooch_fetch(key, downloader=None):
        captured["key"] = key
        captured["downloader"] = downloader
        return str(dummy)

    monkeypatch.setattr(dataFetcher.POOCH, "fetch", fake_pooch_fetch)

    path = dataFetcher.fetch_support_data("supportData/MagneticField/WMM2025.COF")

    assert path == str(dummy)
    assert captured["key"] == "supportData/MagneticField/WMM2025.COF"
    assert isinstance(captured["downloader"], dataFetcher.RetryingHTTPDownloader)


def test_retrying_http_downloader_retries_transient_errors():
    """RetryingHTTPDownloader must retry transient download failures."""
    transient_downloader = TransientDownloader(failures_before_success=2)
    retry_waits = []
    downloader = dataFetcher.RetryingHTTPDownloader(
        attempts=3,
        retry_wait=1,  # [s]
        timeout=2,  # [s]
        sleep=retry_waits.append,
        downloader_factory=lambda **kwargs: transient_downloader,
    )

    downloader("https://example.com/file.dat", "unused", None)

    assert transient_downloader.calls == 3
    assert retry_waits == [1, 1]


def test_retrying_http_downloader_raises_after_last_attempt():
    """RetryingHTTPDownloader must raise when retries are exhausted."""
    transient_downloader = TransientDownloader(failures_before_success=3)
    downloader = dataFetcher.RetryingHTTPDownloader(
        attempts=3,
        retry_wait=1,  # [s]
        timeout=2,  # [s]
        sleep=lambda wait: None,
        downloader_factory=lambda **kwargs: transient_downloader,
    )

    with pytest.raises(RuntimeError, match="temporary download failure"):
        downloader("https://example.com/file.dat", "unused", None)

    assert transient_downloader.calls == 3


def test_retrying_http_downloader_does_not_retry_not_found():
    """RetryingHTTPDownloader must not retry hard HTTP client errors."""
    response = requests.Response()
    response.status_code = 404
    not_found_error = requests.exceptions.HTTPError(response=response)
    calls = []

    def fail_with_not_found(url, output_file, pooch_obj, check_only=False):
        calls.append(url)
        raise not_found_error

    downloader = dataFetcher.RetryingHTTPDownloader(
        attempts=3,
        retry_wait=1,  # [s]
        timeout=2,  # [s]
        sleep=lambda wait: None,
        downloader_factory=lambda **kwargs: fail_with_not_found,
    )

    with pytest.raises(requests.exceptions.HTTPError):
        downloader("https://example.com/missing.dat", "unused", None)

    assert calls == ["https://example.com/missing.dat"]


def test_retrying_http_downloader_uses_backup_after_primary_failure():
    """RetryingHTTPDownloader must try backup URLs after primary failures."""
    calls = []

    def fail_primary(url, output_file, pooch_obj, check_only=False):
        calls.append(url)
        if url == "https://example.com/primary.dat":
            raise RuntimeError("primary download failure")
        return None

    downloader = dataFetcher.RetryingHTTPDownloader(
        attempts=1,
        retry_wait=1,  # [s]
        timeout=2,  # [s]
        sleep=lambda wait: None,
        downloader_factory=lambda **kwargs: fail_primary,
        backup_urls_for=lambda url: ["https://example.com/backup.dat"],
    )

    downloader("https://example.com/primary.dat", "unused", None)

    assert calls == [
        "https://example.com/primary.dat",
        "https://example.com/backup.dat",
    ]


def test_external_kernel_backup_urls_include_haslam_file():
    """The Haslam map must have a mirrored support-data fallback URL."""
    primary_url = dataFetcher.EXTERNAL_KERNEL_URLS[
        "supportData/SkyBrightnessData/haslam408_dsds_Remazeilles2014.fits"
    ]

    assert dataFetcher.EXTERNAL_KERNEL_BACKUP_URLS[primary_url] == [
        "https://hanspeterschaub.info/bskFiles/backup/"
        "supportData/SkyBrightnessData/haslam408_dsds_Remazeilles2014.fits"
    ]

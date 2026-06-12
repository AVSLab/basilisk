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

import pytest
from pathlib import Path

import requests

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

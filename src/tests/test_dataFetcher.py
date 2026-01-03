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
from unittest.mock import patch

from Basilisk.utilities.supportDataTools.dataFetcher import (
    DataFile,
    get_path,
    relpath,
    REGISTRY,
)


@pytest.fixture
def fake_fetch(monkeypatch, tmp_path):
    """Patch POOCH.fetch to always return a small dummy file."""
    dummy = tmp_path / "dummy.dat"
    dummy.write_text("test")

    monkeypatch.setattr(
        "Basilisk.utilities.supportDataTools.dataFetcher.POOCH.fetch",
        lambda key: str(dummy),
    )

    return dummy


def test_all_enum_entries_are_in_registry():
    """Every DataFile enum value must correspond to a key in REGISTRY."""
    missing = []

    for category in DataFile.__dict__.values():
        if isinstance(category, type) and hasattr(category, "__members__"):
            for enum_value in category:
                key = relpath(enum_value)
                if key not in REGISTRY:
                    missing.append(key)

    assert not missing, f"Registry missing entries: {missing}"


def test_get_path_uses_pooch_when_not_local(fake_fetch):
    """get_path() must return a Path backed by mocked Pooch."""
    path = get_path(DataFile.MagneticFieldData.WMM)

    assert isinstance(path, Path)
    assert path.exists(), "get_path() should return an existing dummy file"

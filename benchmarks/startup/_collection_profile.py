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

"""Explicit pytest plugin for collection-only startup measurements."""

import _imp
import json
import os
from pathlib import Path
import time

import pytest


STARTED = float(os.environ["BSK_STARTUP_STARTED"])
OUTPUT = Path(os.environ["BSK_STARTUP_OUTPUT"])
WORKER = os.environ.get("PYTEST_XDIST_WORKER", "controller")
NATIVE_CALLS = []
READY_SECONDS = None
SELECTED_ITEMS = None
ORIGINAL_CREATE_DYNAMIC = _imp.create_dynamic


def create_dynamic(spec, *args, **kwargs):
    """Time native initialization without changing the extension loader's behavior."""
    started = time.perf_counter()
    try:
        return ORIGINAL_CREATE_DYNAMIC(spec, *args, **kwargs)
    finally:
        NATIVE_CALLS.append({"name": spec.name, "file": spec.origin,
                             "seconds": time.perf_counter() - started})


_imp.create_dynamic = create_dynamic


@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(items):
    """Record selected collection readiness and prevent test execution."""
    global READY_SECONDS, SELECTED_ITEMS
    READY_SECONDS = time.time() - STARTED
    SELECTED_ITEMS = len(items)
    items.clear()


def pytest_sessionfinish(exitstatus):
    """Write one profile per process, preserving collection failures in the exit code."""
    data = {"worker": WORKER, "ready_seconds": READY_SECONDS,
            "selected_items": SELECTED_ITEMS, "exit_code": int(exitstatus), "native": NATIVE_CALLS}
    (OUTPUT / f"worker-{WORKER}.json").write_text(
        json.dumps(data, indent=2) + "\n", encoding="utf-8"
    )


def pytest_unconfigure():
    """Restore the native loader when the explicit benchmark plugin shuts down."""
    _imp.create_dynamic = ORIGINAL_CREATE_DYNAMIC

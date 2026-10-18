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

"""Preserve the 2.12 Numba dispatcher API while exercising the numerical fixes."""

import inspect
import os
import subprocess
import sys
import textwrap
import warnings

import numpy as np
import pytest

numba = pytest.importorskip("numba")
pytest.importorskip("scipy")

from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import RigidBodyKinematicsNumba as rbkn

pytestmark = [
    pytest.mark.filterwarnings("error::numba.core.errors.NumbaPendingDeprecationWarning"),
    pytest.mark.filterwarnings("error::numba.core.errors.NumbaDeprecationWarning"),
]

ATTITUDE = np.array([0.1, -0.2, 0.3])  # [-] MRPs


def test_dispatcher_api_preserved():
    """Keep public dispatchers, their Python functions, and compiled signatures available."""
    for name, reference in inspect.getmembers(rbk, inspect.isfunction):
        if name.startswith("_"):
            continue
        function = getattr(rbkn, name)
        assert isinstance(function, numba.core.registry.CPUDispatcher), name
        assert inspect.signature(function.py_func) == inspect.signature(reference)
    result = rbkn.MRP2C(q=ATTITUDE)
    np.testing.assert_allclose(result, rbk.MRP2C(ATTITUDE), rtol=5e-15, atol=5e-15)
    np.testing.assert_allclose(rbkn.MRP2C.py_func(ATTITUDE), result, rtol=5e-15, atol=5e-15)
    assert rbkn.MRP2C.signatures
    assert rbkn.MRP2C.nopython_signatures


@pytest.mark.parametrize("layout", ["contiguous", "strided", "reversed", "readonly"])
def test_array_storage_preserved(layout):
    """An unchanged MRP result must continue to share the caller's array storage."""
    value = ATTITUDE.copy()
    if layout == "strided":
        value = np.repeat(value, 2)[::2]
    elif layout == "reversed":
        value = value[::-1]
    elif layout == "readonly":
        value.setflags(write=False)
    original = value.copy()
    # Strided dot products may emit a performance hint; deprecations must still raise.
    with warnings.catch_warnings(record=True) as captured:
        result = rbkn.MRPswitch(value, 1.0)  # [-] norm threshold
    for warning in captured:
        assert issubclass(warning.category, numba.core.errors.NumbaPerformanceWarning)
        assert "np.dot() is faster on contiguous arrays" in str(warning.message)
    assert np.shares_memory(result, value)
    np.testing.assert_array_equal(result, original)
    np.testing.assert_array_equal(value, original)


def test_compiled_caller_cache(tmp_path):
    """Compile on a fresh worker thread and reload the caller in a second process."""
    script = tmp_path / "cached_rbk.py"
    script.write_text(textwrap.dedent('''\
        import sys
        import threading
        import warnings
        from concurrent.futures import ThreadPoolExecutor
        import numpy as np
        import numba
        from Basilisk.utilities.RigidBodyKinematicsNumba import addMRP

        warnings.simplefilter("error", numba.core.errors.NumbaPendingDeprecationWarning)
        warnings.simplefilter("error", numba.core.errors.NumbaDeprecationWarning)

        @numba.njit(cache=True)
        def compose(value):
            return addMRP(value, np.array([0.2, 0.0, 0.0]))  # [-] MRPs

        # Match the default stack of macOS simulation workers, with a cold cache.
        if sys.platform == "darwin":
            threading.stack_size(512 * 1024)  # [bytes]
        with ThreadPoolExecutor(max_workers=1) as executor:
            result = executor.submit(compose, np.zeros(3)).result()
        np.testing.assert_allclose(result, [0.2, 0.0, 0.0], rtol=5e-15, atol=0.0)  # [-]
        assert compose.nopython_signatures
    '''))
    environment = dict(os.environ, NUMBA_DEBUG_CACHE="1", NUMBA_CACHE_DIR=str(tmp_path / "cache"))
    for run in range(2):
        result = subprocess.run([sys.executable, str(script)], env=environment,
                                capture_output=True, text=True, check=True, timeout=60)
        if run == 1:
            assert "data loaded" in result.stdout

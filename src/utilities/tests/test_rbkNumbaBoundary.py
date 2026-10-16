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

"""Exercise Python and compiled call boundaries without deprecated list reflection."""

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
from Basilisk.utilities.RigidBodyKinematicsNumba import MRP2C

pytestmark = [
    pytest.mark.filterwarnings("error::numba.core.errors.NumbaPendingDeprecationWarning"),
    pytest.mark.filterwarnings("error::numba.core.errors.NumbaDeprecationWarning"),
]

ATTITUDE = np.array([0.1, -0.2, 0.3])  # [-] MRPs
DERIVATIVE = np.array([0.01, 0.02, -0.03])  # [1/s]
SECOND_DERIVATIVE = np.array([-0.003, 0.002, 0.001])  # [1/s^2]
OMEGA = np.array([0.1, 0.2, -0.3])  # [rad/s]
ACCELERATION = np.array([-0.03, 0.02, 0.01])  # [rad/s^2]


# Cover each arity, call style, and input kind without multiplying independent
# dimensions. Numerical and per-function sequence regressions live elsewhere.
@pytest.mark.parametrize("name, arguments, container, call_style", [
    pytest.param("MRP2C", (ATTITUDE,), list, "positional", id="vector-list"),
    pytest.param("MRP2C", (ATTITUDE,), tuple, "keyword", id="vector-tuple-keyword"),
    pytest.param("C2EP", (rbk.MRP2C(ATTITUDE),), list, "keyword", id="matrix-list-keyword"),
    pytest.param("C2EP", (rbk.MRP2C(ATTITUDE),), tuple, "positional", id="matrix-tuple"),
    pytest.param("subMRP", (ATTITUDE, -ATTITUDE), np.array, "mixed", id="two-arguments"),
    pytest.param("MRPswitch", (ATTITUDE, 1.0), tuple, "keyword", id="vector-and-scalar"),  # [-]
    pytest.param("ddMRP2dOmega", (ATTITUDE, DERIVATIVE, SECOND_DERIVATIVE),
                 list, "mixed", id="three-arguments"),
    pytest.param("ddMRP", (ATTITUDE, DERIVATIVE, OMEGA, ACCELERATION),
                 tuple, "keyword", id="four-arguments"),
    pytest.param("Mi", (0.3, 2), None, "keyword", id="scalar-keywords"),  # [rad], axis index
    pytest.param("Picheck", (9.0,), None, "positional", id="scalar-positional"),  # [rad]
])
def test_python_call_boundary(name, arguments, container, call_style):
    """Preserve signatures, scalar arguments, numerical results, and input contents."""
    values = [container(value.tolist()) if isinstance(value, np.ndarray) else value
              for value in arguments]
    original = [np.array(value, copy=True) for value in values]
    reference = getattr(rbk, name)
    function = getattr(rbkn, name)
    parameters = inspect.signature(reference).parameters
    assert inspect.signature(function) == inspect.signature(reference)
    keyword = dict(zip(parameters, values))
    if call_style == "positional":
        result = function(*values)
    elif call_style == "keyword":
        result = function(**dict(reversed(list(keyword.items()))))
    else:
        del keyword[next(iter(parameters))]
        result = function(values[0], **keyword)
    np.testing.assert_allclose(result, reference(*arguments), rtol=5e-15, atol=5e-15)
    for value, before in zip(values, original):
        np.testing.assert_array_equal(value, before)


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


@numba.njit
def compiled_calls(attitude, derivative, omega, acceleration, second_derivative):
    """Exercise imported aliases, keywords, scalar arguments, and internal call chains."""
    return (MRP2C(q=attitude), rbkn.C2MRP(rbkn.MRP2C(attitude)),
            rbkn.MRPswitch(s2=1.0, q=attitude),
            rbkn.ddMRP(attitude, derivative, dw=acceleration, w=omega),
            rbkn.ddMRP2dOmega(attitude, derivative, second_derivative),
            rbkn.Mi(i=2, theta=0.3), rbkn.Picheck(9.0))  # [rad] angles


def test_compiled_call_boundary():
    """Cover all arities and native container types with one compiled signature."""
    inputs = (ATTITUDE, tuple(DERIVATIVE), numba.typed.List(OMEGA),
              ACCELERATION, numba.typed.List(SECOND_DERIVATIVE))
    expected = (rbk.MRP2C(ATTITUDE), ATTITUDE, ATTITUDE,
                rbk.ddMRP(ATTITUDE, DERIVATIVE, OMEGA, ACCELERATION),
                rbk.ddMRP2dOmega(ATTITUDE, DERIVATIVE, SECOND_DERIVATIVE),
                rbk.Mi(0.3, 2), rbk.Picheck(9.0))  # [rad] angles
    for result, reference in zip(compiled_calls(*inputs), expected):
        np.testing.assert_allclose(result, reference, rtol=5e-15, atol=5e-15)
    assert compiled_calls.nopython_signatures


def test_containers_created_inside_kernel():
    """Convert a native local list and a nested tuple without Python list reflection."""
    @numba.njit
    def local_containers():
        attitude = [0.1, -0.2, 0.3]  # [-]
        matrix = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))  # [-]
        return rbkn.MRP2C(attitude), rbkn.C2EP(matrix)

    matrix, quaternion = local_containers()
    np.testing.assert_allclose(matrix, rbk.MRP2C(ATTITUDE), rtol=5e-15, atol=5e-15)
    np.testing.assert_array_equal(quaternion, [1.0, 0.0, 0.0, 0.0])


def test_reflected_outer_argument_rejected():
    """Explain that lists must be converted before entering a user-owned compiled caller."""
    @numba.njit
    def outer(attitude):
        return rbkn.MRP2C(attitude)

    with pytest.raises(numba.core.errors.TypingError, match="outermost compiled function"):
        outer(ATTITUDE.tolist())


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

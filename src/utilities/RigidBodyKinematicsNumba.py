#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
"""Numba-compatible rigid-body kinematics with Python sequence input support.

Python calls accept numeric lists, tuples, and NumPy arrays. Lists and tuples
are converted to ``float64`` arrays before entering compiled code, avoiding
Numba's deprecated Python list reflection. Existing arrays and scalar
arguments are passed through unchanged.

Calls from compiled code use registered overloads and the same mathematical
implementations. The common :ref:`rbkNumericalLimits` also apply here.
Use arrays or numeric tuples at the outermost compiled
function's boundary. A Python list passed directly to a user-defined
``@numba.njit`` function must be converted before that call; a kinematics
function inside it cannot perform that outer conversion. Native lists created
inside compiled code and ``numba.typed.List`` inputs are also supported by
the overloads.

Design
------
``RigidBodyKinematics.py`` is loaded into a private module and its functions
are compiled with ``nb.njit(cache=False, inline='always')``. Only that private
copy is patched, preserving compiled internal calls without modifying the
plain Python module. Public functions are Python wrappers with matching
signatures and Numba overloads; they are not Numba dispatcher objects.
Compiled callers bypass the Python wrappers and may use Numba's disk cache.

Requires scipy (for BLAS-backed ``np.dot`` / ``np.linalg.norm`` inside
Numba nopython).  A clear ``ImportError`` is raised at import time if missing.
"""

try:
    import scipy  # noqa: F401  – verifies BLAS is available for Numba linalg
except ImportError as _e:
    raise ImportError(
        "RigidBodyKinematicsNumba requires scipy for np.dot / "
        "np.linalg.norm inside Numba nopython kernels."
    ) from _e

import importlib.util
import inspect
from functools import wraps as _wraps
from pathlib import Path

import numba as nb
import numpy as np
from numba.extending import overload as _overload

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from Basilisk.utilities.RigidBodyKinematics import *

# Load RigidBodyKinematics into a fresh, private module (zero side effects).
# The module is registered in sys.modules so Numba's environment serialiser can
# import it by name when loading cached compilations.
import sys as _sys
_MODULE_NAME = "RigidBodyKinematicsNumba._source"
_spec = importlib.util.spec_from_file_location(
    _MODULE_NAME,
    Path(__file__).parent / "RigidBodyKinematics.py",
)
_src = importlib.util.module_from_spec(_spec)
_sys.modules[_MODULE_NAME] = _src   # register BEFORE exec so the name is stable
_spec.loader.exec_module(_src)      # populates _src.__dict__; public RBK untouched

# Apply nb.njit to every function and patch the private copy so that
# inter-function calls (e.g. C2MRP -> C2EP) resolve to jitted versions.
# cache=False: these functions live in a dynamically-loaded private module.
# When inlined into a NumbaModel, the caller's compiled form is cached by
# numbaModel's own cache; standalone calls recompile per process (fast).
_deco = nb.njit(cache=False, inline='always')


def _normalize_input(value):
    """Convert Python sequences before dispatch while preserving arrays and scalars."""
    if isinstance(value, (list, tuple)):
        return np.asarray(value, dtype=np.float64)
    return value


@_overload(_normalize_input, inline="always")
def _normalize_compiled_input(value):
    """Normalize native containers without permitting Python list reflection."""
    if isinstance(value, nb.types.List) and value.reflected:
        raise nb.core.errors.TypingError(
            "Convert Python lists to NumPy arrays before entering the outermost compiled function."
        )
    if isinstance(value, (nb.types.BaseTuple, nb.types.List, nb.types.ListType)):
        def implementation(value):
            return np.asarray(value, dtype=np.float64)
    else:
        def implementation(value):
            return value
    return implementation


def _make_wrapper(function, kernel):
    """Expose one compiled implementation through Python and Numba call boundaries."""
    @_wraps(function)
    def wrapper(*args, **kwargs):
        return kernel(*(_normalize_input(value) for value in args),
                      **{name: _normalize_input(value) for name, value in kwargs.items()})

    wrapper.__module__ = __name__
    parameters = inspect.signature(function).parameters
    arguments = ", ".join(parameters)
    converted = ", ".join(f"_normalize_input({name})" for name in parameters)
    namespace = {"_kernel": kernel, "_normalize_input": _normalize_input}
    # Numba requires matching named signatures for overload typing and implementation,
    # including keyword calls. Generate only these bridges from the trusted source
    # parameter names; the mathematical implementation remains in the private kernel.
    exec(f"def implementation({arguments}):\n"
         f"    return _kernel({converted})\n"
         f"def typing({arguments}):\n"
         f"    return implementation\n", namespace)
    _overload(wrapper, inline="always")(namespace["typing"])
    return wrapper


_functions = inspect.getmembers(_src, inspect.isfunction)
_kernels = {name: _deco(function) for name, function in _functions}
_src.__dict__.update(_kernels)
for _name, _func in _functions:
    globals()[_name] = _make_wrapper(_func, _kernels[_name])

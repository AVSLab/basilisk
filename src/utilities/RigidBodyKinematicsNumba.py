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
"""
RigidBodyKinematicsNumba.py, a Numba nopython version of RigidBodyKinematics.py

Applies ``nb.njit(cache=True, inline='always')`` to every function so they
can be called from other compiled Numba kernels (e.g. NumbaModel
``UpdateStateImpl``).

Design
------
``RigidBodyKinematics.py`` is loaded into a **private, isolated module object**
via ``importlib`` - it is never registered under the original name in
``sys.modules``.  Functions defined in that private copy have ``__globals__``
pointing to its own ``__dict__``.  Only that private copy is patched with
jitted versions; the public ``RigidBodyKinematics`` module is never imported
or modified here, so it retains full plain-Python behaviour (accepts lists,
returns plain numpy objects, raises normal Python errors).

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
from pathlib import Path

import numba as nb

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

for _name, _func in inspect.getmembers(_src, inspect.isfunction):
    _jitted = _deco(_func)
    setattr(_src, _name, _jitted)   # patch isolated copy so cross-calls work
    globals()[_name] = _jitted      # re-export from this module

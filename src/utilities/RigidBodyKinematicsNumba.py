"""
RigidBodyKinematicsNumba.py  -  Numba nopython version of RigidBodyKinematics.py

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

# ---------------------------------------------------------------------------
# Load RigidBodyKinematics into a fresh, private module - zero side effects
# ---------------------------------------------------------------------------
_spec = importlib.util.spec_from_file_location(
    "RigidBodyKinematicsNumba._source",
    Path(__file__).parent / "RigidBodyKinematics.py",
)
_src = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_src)   # populates _src.__dict__; _rbk untouched

# Apply nb.njit to every function and patch the private copy so that
# inter-function calls (e.g. C2MRP -> C2EP) resolve to jitted versions.
_deco = nb.njit(cache=True, inline='always')

for _name, _func in inspect.getmembers(_src, inspect.isfunction):
    _jitted = _deco(_func)
    setattr(_src, _name, _jitted)   # patch isolated copy - cross-calls work
    globals()[_name] = _jitted      # re-export from this module

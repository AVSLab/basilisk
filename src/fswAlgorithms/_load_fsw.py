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

"""Load an individual FSW extension from the optional combined native library."""

import importlib.util as _import_util
import sys as _sys

_library_spec = _import_util.find_spec(f"{__package__}._fswCoreNative")
if _library_spec is None or _library_spec.origin is None:
    raise ImportError(f"The combined FSW library is missing from {__package__}.")
_library_path = _library_spec.origin


def load_native_module(full_name: str) -> None:
    """Replace a generated import shim with its original SWIG extension.

    Each binding is initialized only when its Python proxy or private extension
    is imported. Python's normal import lock protects each module name.
    Reloading a private extension preserves the existing native module.

    :param full_name: Fully qualified name of the private SWIG extension.
    :raises ImportError: If the native binding cannot be loaded.
    """
    placeholder = _sys.modules[full_name]
    native_spec = getattr(placeholder, "_bsk_fsw_native_spec", None)
    if native_spec is not None:
        # reload() finds the Python shim and overwrites these attributes before
        # executing it. Restore the native metadata without initializing again.
        placeholder.__spec__ = native_spec
        placeholder.__loader__ = native_spec.loader
        placeholder.__file__ = native_spec.origin
        placeholder.__package__ = native_spec.parent
        placeholder.__dict__.pop("__cached__", None)
        return

    spec = _import_util.spec_from_file_location(full_name, _library_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load {full_name} from {_library_path}.")
    module = _import_util.module_from_spec(spec)
    # Preserve importlib's initialization flag before publishing the replacement.
    # Otherwise concurrent imports bypass the shim's module lock and can use
    # native functions before SWIG has initialized their types.
    spec._initializing = True
    try:
        _sys.modules[full_name] = module
        spec.loader.exec_module(module)
        module._bsk_fsw_native_spec = spec
    except BaseException:
        _sys.modules[full_name] = placeholder
        raise
    finally:
        spec._initializing = False

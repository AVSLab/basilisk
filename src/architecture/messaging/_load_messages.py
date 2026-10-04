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

"""Load the message bindings' separate entry points from one native library."""

import importlib.util as _import_util
import sys as _sys


# Resolve the platform's extension suffix and the installed package location
# without importing the container: its entry points retain the SWIG names.
_library_spec = _import_util.find_spec(f"{__package__}._messagingNative")
if _library_spec is None or _library_spec.origin is None:
    raise ImportError(f"The combined messaging library is missing from {__package__}.")
_library_path = _library_spec.origin


def load_message_module(module_name: str) -> None:
    """Initialize a native binding before importing its Python proxy.

    Preserve the original module names and normal module caching so direct
    imports, SWIG class registration, and recorder creation keep working.
    The generated package initializer holds Python's package import lock.
    Cache native modules only after initialization so concurrent direct imports
    wait for that initializer instead of receiving an unfinished binding.
    Failed initialization leaves no cached module.
    No process-wide import hook is installed.

    :param module_name: SWIG module name without its leading underscore.
    :raises ImportError: If the native binding cannot be loaded.
    """
    native_name = f"_{module_name}"
    full_name = f"{__package__}.{native_name}"
    module = _sys.modules.get(full_name)
    # A fallback shim can occupy this name while waiting for the package.
    # Reuse only native bindings whose initialization has completed.
    native_spec = getattr(module, "_bsk_message_native_spec", None)
    if native_spec is not None:
        # reload() can execute the fallback shim on the existing native module.
        # Restore its metadata without initializing the extension again.
        module.__spec__ = native_spec
        module.__loader__ = native_spec.loader
        module.__file__ = native_spec.origin
        module.__package__ = native_spec.parent
        module.__dict__.pop("__cached__", None)
    else:
        spec = _import_util.spec_from_file_location(full_name, _library_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Cannot load {full_name} from {_library_path}.")
        module = _import_util.module_from_spec(spec)
        # SWIG initializes its native type information during exec_module.
        # Publishing earlier lets direct imports bypass the package import lock
        # and construct payloads whose type information is still uninitialized.
        spec.loader.exec_module(module)
        module._bsk_message_native_spec = spec
        _sys.modules[full_name] = module
    # A failed package import can leave earlier bindings cached. Attach them
    # to the replacement package as well when its initializer is retried.
    setattr(_sys.modules[__package__], native_name, module)

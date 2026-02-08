/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

%module(directors="1",threads="1",package="Basilisk.architecture") sysModel

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
   #include "sys_model.h"
%}

%pythoncode %{
import sys
import traceback
from Basilisk.architecture.swig_common_model import *
%}

%include "std_string.i"
%include "swig_conly_data.i"
%include "architecture/utilities/bskLogging.h"

/* Director support for the C++ SysModel base */
%feature("director") SysModel;

/* Expose the raw SWIG proxy as _SysModel */
%rename("_SysModel") SysModel;
%include "sys_model.i"

%pythoncode %{
def logError(func):
    """Decorator that prints any exceptions raised by func, then re raises them."""
    def inner(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            traceback.print_exc()
            raise
    # Mark so we do not wrap twice
    inner._sysmodelLogged = True
    return inner


class SysModelMixin:
    """Pure Python mixin for Basilisk system models.

    Provides:
      * Automatic logError wrapping for overridden director methods
        from any SWIG C++ proxy bases in the MRO.
      * Enforcement that subclasses call super().__init__().
    """

    def __init__(self, *args, **kwargs):
        # Cooperative init; super() will eventually call the SWIG proxy __init__
        super().__init__(*args, **kwargs)
        # Mark that the mixin init has been called on this instance.
        self._sysmodelBaseInitSeen = True

    @classmethod
    def _wrapOverriddenDirectorMethods(cls):
        """Wrap all overridden director methods with logError.

        A method is considered a director method if:
          * it exists on at least one SWIG C++ proxy base
          * and it is overridden in cls with a different callable
        """
        cppBases = []
        for base in cls.mro()[1:]:
            if base is object:
                continue
            # Heuristic: SWIG proxy types have __swig_destroy__ or similar
            if hasattr(base, "__swig_destroy__"):
                cppBases.append(base)

        if not cppBases:
            return

        # Collect candidate method names from all C++ bases
        candidateNames = set()
        for base in cppBases:
            for name in dir(base):
                if name.startswith("_"):
                    continue
                if name in ("__init__",):
                    continue
                candidateNames.add(name)

        for name in candidateNames:
            subAttr = getattr(cls, name, None)
            if not callable(subAttr):
                continue

            # Find the first base that defines this name
            baseAttr = None
            for base in cppBases:
                if hasattr(base, name):
                    baseAttr = getattr(base, name)
                    break

            if baseAttr is None:
                continue

            # Only wrap if the subclass actually overrides the method
            if subAttr is baseAttr:
                continue

            # Avoid double wrapping
            if getattr(subAttr, "_sysmodelLogged", False):
                continue

            setattr(cls, name, logError(subAttr))

    def __init_subclass__(cls):
        """Hook that runs for every Python subclass of SysModelMixin.

        It:
          * wraps all overridden director methods from SWIG C++ proxy bases
            in the MRO with logError
          * enforces super().__init__() on subclasses
        """
        super().__init_subclass__()

        # Wrap overridden director methods
        cls._wrapOverriddenDirectorMethods()

        origInit = cls.__init__

        # Avoid double wrapping if someone subclasses a subclass.
        if getattr(origInit, "_sysmodelWrappedInit", False):
            return

        def wrappedInit(self, *args, **kwargs):
            # Clear flag before running user __init__.
            object.__setattr__(self, "_sysmodelBaseInitSeen", False)
            origInit(self, *args, **kwargs)
            if not getattr(self, "_sysmodelBaseInitSeen", False):
                errorMsg = (
                    "Need to call parent __init__ in SysModel based subclasses:\n"
                    f"class {cls.__name__}(...):\n"
                    "    def __init__(...):\n"
                    "        super().__init__()"
                )
                raise SyntaxError(errorMsg)

        wrappedInit._sysmodelWrappedInit = True
        cls.__init__ = wrappedInit


class SysModel(SysModelMixin, _SysModel):
    """Python friendly base class for Basilisk system models.

    Inherits:
      * SysModelMixin  (pure Python)
      * _SysModel      (SWIG proxy for C++ SysModel)

    Typical usage for new models:
        from Basilisk.architecture import sysModel

        class MyModel(sysModel.SysModel):
            def __init__(self):
                super().__init__()
                ...

            def UpdateState(self, CurrentSimNanos):
                ...

            def Reset(self, CurrentSimNanos):
                ...
    """

    bskLogger: BSKLogger = None
%}

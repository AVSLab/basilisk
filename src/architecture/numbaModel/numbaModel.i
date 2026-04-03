/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

%module(directors="1", threads="1", package="Basilisk.architecture") numbaModel

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
#include "architecture/_GeneralModuleFiles/numbaModel.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

/* std::string ↔ Python str, uint64_t, SysModel - same pattern as all BSK sim modules */
%include "std_string.i"
%include "stdint.i"
%import "architecture/_GeneralModuleFiles/py_sys_model.i"

/* uintptr_t: accept any Python int as a raw 64-bit bit-pattern */
%typemap(in) uintptr_t {
    unsigned long long tmp = PyLong_AsUnsignedLongLongMask($input);
    if (PyErr_Occurred()) SWIG_fail;
    $1 = (uintptr_t)tmp;
}

/* uintptr_t: return as a full unsigned 64-bit Python int (avoid sign-extension / truncation) */
%typemap(out) uintptr_t {
    $result = PyLong_FromUnsignedLongLong((unsigned long long)$1);
}

/* Expose ReaderSlot struct and NumbaModel class */
%feature("director") NumbaModel;
/* UpdateState is implemented in C++ and must NOT go through the Python director.
   The C++ UpdateState calls the compiled cfunc directly; routing it through SWIG's
   director mechanism (GIL acquire + Python method lookup) adds ~6 µs per tick. */
%feature("nodirector") NumbaModel::UpdateState;
%rename("_NumbaModel") NumbaModel;

/* Suppress SWIG warning for ReaderSlot (internal use only; not wrapped as a Python class) */
%ignore ReaderSlot;

%include "architecture/_GeneralModuleFiles/numbaModel.h"

%pythoncode %{
import inspect
import hashlib
import importlib.util
import marshal
import os
import pathlib
import shutil
import sys
from dataclasses import dataclass, field
from typing import Any, Optional

from platformdirs import user_cache_dir
import numpy as np
import numba as nb
from numba.experimental import jitclass as _nbJitclass

from Basilisk.architecture import messaging
from Basilisk.architecture.sysModel import SysModelMixin

_NBMODEL_CFUNC_CACHE: dict = {} # idHash -> cfunc; keeps cfunc alive (prevents GC)

def getCacheDir():
    """Return platformdirs user cache dir; None if not writable."""
    d = pathlib.Path(user_cache_dir('basilisk')) / 'numba_model'
    try:
        d.mkdir(parents=True, exist_ok=True)
        return d
    except OSError:
        return None

def clearCacheDir():
    """Remove all cached numba model functions."""
    d = getCacheDir()
    if d is None or not d.exists():
        return
    shutil.rmtree(d)

_bsklogSpec = [
    ('_level', nb.int32),
    ('_t0',    nb.types.unicode_type),
    ('_t1',    nb.types.unicode_type),
    ('_t2',    nb.types.unicode_type),
    ('_t3',    nb.types.unicode_type),
]

@_nbJitclass(_bsklogSpec)
class BskLoggerProxy:
    """Logging proxy usable inside UpdateStateImpl (nopython mode).
    Calls print(); output goes to stdout.
    Tags and min-level are read from the module's BSKLogger at Reset time.
    """
    def __init__(self, level, t0, t1, t2, t3):
        self._level = level
        self._t0 = t0; self._t1 = t1; self._t2 = t2; self._t3 = t3

    def _tag(self, level):
        if   level == 0: return self._t0
        elif level == 1: return self._t1
        elif level == 2: return self._t2
        else:            return self._t3

    def bskLog(self, level, msg):
        if level >= self._level:
            print(self._tag(level), msg)

    def bskLog1(self, level, msg, v0):
        if level >= self._level:
            print(self._tag(level), msg, v0)

    def bskLog2(self, level, msg, v0, v1):
        if level >= self._level:
            print(self._tag(level), msg, v0, v1)

    def bskLog3(self, level, msg, v0, v1, v2):
        if level >= self._level:
            print(self._tag(level), msg, v0, v1, v2)


import math as _math

# ---------------------------------------------------------------------------
# xoshiro256++ PRNG - state is uint64[4] stored in a NumbaModel userPtr array.
# The jitclass wraps the array by reference so that state persists between
# ticks without any Python overhead.
# ---------------------------------------------------------------------------

@nb.njit(cache=True)
def _rngRotl64(x, k):
    return (x << k) | (x >> (np.uint64(64) - k))


_rngSpec = [('_s', nb.uint64[:])]

@_nbJitclass(_rngSpec)
class _RngWrapper:
    """xoshiro256++ PRNG usable inside nopython kernels.

    Created from a uint64[4] array that lives in the NumbaModel userPtr
    vector.  Modifications to ``self._s`` are in-place, so state persists
    across ticks via the underlying C buffer.
    """

    def __init__(self, state):
        self._s = state

    def _next(self):
        s = self._s
        result = _rngRotl64(s[0] + s[3], np.uint64(23)) + s[0]
        t = s[1] << np.uint64(17)
        s[2] ^= s[0]
        s[3] ^= s[1]
        s[1] ^= s[2]
        s[0] ^= s[3]
        s[2] ^= t
        s[3] = _rngRotl64(s[3], np.uint64(45))
        return result

    def random(self):
        """Uniform float in [0, 1)."""
        return np.float64(self._next() >> np.uint64(11)) * (1.0 / 9007199254740992.0)

    def standard_normal(self, n):
        """Return *n* independent standard-normal samples (Box-Muller)."""
        out = np.empty(n, np.float64)
        i = 0
        while i < n:
            u1 = self.random()
            u2 = self.random()
            if u1 < 1e-300:
                u1 = 1e-300
            r = _math.sqrt(-2.0 * _math.log(u1))
            theta = 6.283185307179586 * u2   # 2 * pi
            out[i] = r * _math.cos(theta)
            if i + 1 < n:
                out[i + 1] = r * _math.sin(theta)
                i += 2
            else:
                i += 1
        return out

    def integers(self, low, high):
        """Uniform integer in [low, high)."""
        return np.int64(low) + np.int64(self._next() % np.uint64(high - low))

    def uniform(self, low, high):
        """Uniform float in [low, high)."""
        return low + (high - low) * self.random()


def _seedXoshiro256pp(seed):
    """Seed xoshiro256++ state from a 64-bit integer via splitmix64."""
    state = np.zeros(4, dtype=np.uint64)
    x = np.uint64(int(seed) & 0xFFFFFFFFFFFFFFFF)
    with np.errstate(over='ignore'):
        for i in range(4):
            x = x + np.uint64(0x9e3779b97f4a7c15)
            z = x
            z = (z ^ (z >> np.uint64(30))) * np.uint64(0xbf58476d1ce4e5b9)
            z = (z ^ (z >> np.uint64(27))) * np.uint64(0x94d049bb133111eb)
            state[i] = z ^ (z >> np.uint64(31))
    return state


class MemoryNamespace:
    """Stores user-defined persistent memory for a NumbaModel.

    Before Reset() is called, attributes can be freely set (plain Python
    values).  After Reset() is called, reads and writes are transparently
    forwarded to the structured numpy buffer that backs the C++ user-pointer
    array, so that any change made from Python is immediately visible inside
    the running cfunc.
    """

    def __setattr__(self, name, value):
        d = object.__getattribute__(self, '__dict__')
        if name.startswith('_nbm_'):
            d[name] = value
            return
        buf = d.get('_nbmBuf')
        if buf is not None:
            dt = d['_nbmDtype']
            if name not in dt.names:
                raise AttributeError(
                    f"memory.{name!r} does not exist after Reset(). "
                    f"All memory fields must be defined before Reset() "
                    f"(e.g. self.memory.{name} = 0.0 in __init__)."
                )
            if dt[name].shape:           # array field
                buf[0][name][:] = np.asarray(value)
            else:                         # scalar field
                buf[0][name] = value
        else:
            d[name] = value

    def __getattribute__(self, name):
        if name.startswith('_') or name in ('__class__', '__dict__'):
            return object.__getattribute__(self, name)
        d = object.__getattribute__(self, '__dict__')
        buf = d.get('_nbmBuf')
        if buf is not None:
            dt = d['_nbmDtype']
            if name in dt.names:
                return buf[0][name]
        if name in d:
            return d[name]
        return object.__getattribute__(self, name)


@dataclass
class NbmMsgEntry:
    """One scalar reader or writer registered with the C++ layer."""
    pName: str
    dtype: np.dtype
    idx:   int   # position in the read or write slice of allPtrs

@dataclass
class NbmMsgListEntry:
    """One list/dict reader or writer registered with the C++ layer."""
    pName:  str
    dtype:  np.dtype
    n:      int                    # element count (0 for empty containers)
    base:   int                    # first slot index in the read/write slice of allPtrs
    bufUidx: int                   # user-pointer index for the payload buffer
    keys:   Optional[tuple[str, ...]]  # dict keys, or None for plain lists
    linkedBufUidx: Optional[int] = None  # dict IsLinked buffer, or None

@dataclass
class NbmMemoryEntry:
    """Compiled-memory registration."""
    dtype: np.dtype
    uidx:  int   # user-pointer index for the memory buffer


# ---------------------------------------------------------------------------
# NbmCtx - mutable compilation context passed through _nbmCompile helpers
# ---------------------------------------------------------------------------

@dataclass
class NbmCtx:
    """Mutable compilation context threaded through all _nbmCompile helper methods.

    Created fresh at the start of every _nbmCompile call and discarded once
    _nbmBuildCfunc returns.  Every helper receives this object and mutates it
    in place; no field is written by more than one step (see pipeline below).

    Pipeline
    --------
    Step 0  _nbmGetImpl          → populates  params
    Step 1  _nbmClassifyParam    → populates  readers, writers, listReaders,
                                              listWriters, memoryCodegen,
                                              linkedSet;
                                   increments readIdx, writeIdx, userIdx
    Step 2  _nbmValidateLinks    → reads      readers, listReaders, linkedSet
                                   (read-only; raises on unsubscribed msgs)
    Step 3  _nbmRegisterMetaPtrs → populates  linkedUidx, moduleIdUidx, rngUidx;
                                   increments userIdx
    Step 4  _nbmGenerateCode     → reads all of the above;
                                   populates  g, preLines, postLines
    Step 5  _nbmBuildCfunc       → reads      g, preLines, postLines, params;
                                   compiles and registers the cfunc

    allPtrs layout (assembled by C++ finalizeAllPtrs after _nbmCompile):
        [0 .. readIdx-1]               read-payload slots   (refreshed every tick)
        [readIdx .. readIdx+writeIdx-1] write-payload slots
        [readIdx+writeIdx .. ]          user-pointer slots   (arbitrary Python objs)

    _WRITE_BASE = readIdx  and  _USER_BASE = readIdx + writeIdx  are injected into
    g as integer constants by _nbmGenerateCode so that any preLines strings added
    by step-1 handlers (before the bases are known) can reference them by name and
    have them resolved at exec() time.
    """

    # -- Step 0 ---------------------------------------------------------------

    params: list[str] = field(default_factory=list)
    # Ordered list of UpdateStateImpl parameter names, taken directly from the
    # function signature.  Drives the step-1 dispatch loop and is later joined
    # into the _impl(...) call line by _nbmBuildCfunc.

    # -- Step 1 ---------------------------------------------------------------

    readers: list[NbmMsgEntry] = field(default_factory=list)
    # One entry per scalar InMsg parameter (e.g. dataInMsgPayload).
    # Set by _nbmHandleInMsgPayload.  Each entry records the parameter name,
    # payload dtype, and its slot index in the read slice of allPtrs.
    # Used by _nbmValidateLinks (link check) and _nbmGenerateCode (binding).

    writers: list[NbmMsgEntry] = field(default_factory=list)
    # One entry per scalar OutMsg parameter (e.g. dataOutMsgPayload).
    # Set by _nbmHandleOutMsgPayload.  Each entry records the parameter name,
    # payload dtype, and its slot index in the write slice of allPtrs.
    # Used by _nbmGenerateCode to emit the write-payload binding lines.

    listReaders: list[NbmMsgListEntry] = field(default_factory=list)
    # One entry per list/dict InMsg parameter.
    # Set by _nbmHandleInMsgPayload.  Each entry records the parameter name,
    # dtype, element count n, base slot in the read slice, user-pointer index
    # of the payload buffer, and optional dict keys (None for plain lists).
    # Used by _nbmValidateLinks and _nbmGenerateCode.

    listWriters: list[NbmMsgListEntry] = field(default_factory=list)
    # One entry per list/dict OutMsg parameter.
    # Set by _nbmHandleOutMsgPayload.  Each entry mirrors listReaders but for
    # write slots.  Used by _nbmGenerateCode to emit both the pre-call binding
    # and the post-call copy-back lines (via postLines).

    memoryCodegen: Optional[NbmMemoryEntry] = None
    # Set by _nbmHandleMemory when the user declares a 'memory' parameter.
    # Holds the structured numpy dtype built from self.memory's fields and the
    # user-pointer index of the memory buffer.  None if 'memory' is not in params.
    # Used by _nbmGenerateCode to emit the memory binding line.

    linkedSet: set[str] = field(default_factory=set)
    # Set of *InMsgIsLinked parameter names declared in UpdateStateImpl.
    # Populated by _nbmHandleInMsgIsLinked (one add per IsLinked param).
    # Used in _nbmValidateLinks to skip the always-linked assertion, and in
    # _nbmGenerateCode to emit the per-slot or per-element IsLinked bindings.

    readIdx: int = 0
    # Running count of registered reader slots.  Incremented by
    # _nbmHandleInMsgPayload (by 1 for scalars, by n for list/dict).
    # Final value equals the number of read-payload slots in allPtrs, and
    # becomes _WRITE_BASE in the generated code.

    writeIdx: int = 0
    # Running count of registered writer slots.  Incremented by
    # _nbmHandleOutMsgPayload (by 1 for scalars, by n for list/dict).
    # Final value equals the number of write-payload slots in allPtrs, and
    # contributes to _USER_BASE = readIdx + writeIdx.

    userIdx: int = 0
    # Running count of registered user pointers.  Incremented by step-1
    # handlers (payload buffers, stub arrays, memory buffer) and step-3
    # (_nbmRegisterMetaPtrs: readLinked_ array, moduleID array, rng state).
    # Each increment corresponds to one addUserPointer() call on the C++ side.
    # The final value is the length of the user-pointer slice of allPtrs.

    # -- Step 4 ---------------------------------------------------------------

    g: dict[str, Any] = field(default_factory=lambda: {'nb': nb, 'np': np})
    # Global namespace for exec() / cfunc compilation.  Pre-seeded with 'nb'
    # and 'np' so generated code can reference numba and numpy by name.
    # Step-4 injects dtype constants (_rdt_*, _wdt_*, _cdt_*, _WRITE_BASE,
    # _USER_BASE, _impl, and any handler-specific objects like _RngWrapper).

    preLines: list[str] = field(default_factory=lambda: ["def _wrapper(allPtrs, CurrentSimNanos):"])
    # Lines of the generated _wrapper function body that execute *before*
    # calling _impl.  Opened with the def line; step-4 appends one or more
    # variable-binding lines for each registered parameter.

    postLines: list[str] = field(default_factory=list)
    # Lines appended after the _impl call.  Used exclusively for list/dict
    # writer copy-back: after _impl returns, each element of the payload buffer
    # is written back into the individual allPtrs write slots.

    # -- Step 3 ---------------------------------------------------------------

    linkedUidx: Optional[int] = None
    # User-pointer index of the readLinked_ uint8 array maintained by the C++
    # layer (one byte per reader slot, non-zero when the slot is linked).
    # Set by _nbmRegisterMetaPtrs only when readIdx > 0; stays None if there
    # are no readers.  Used by _nbmGenerateCode to emit IsLinked bindings that
    # index into allPtrs[_USER_BASE + linkedUidx].

    moduleIdUidx: Optional[int] = None
    # User-pointer index of the int64[1] array holding self.moduleID.
    # Set by _nbmRegisterMetaPtrs when 'moduleID' is in params; stays None
    # otherwise.  Used by _nbmGenerateCode to emit the moduleID binding line.

    rngUidx: Optional[int] = None
    # User-pointer index of the uint64[4] xoshiro256++ RNG state array.
    # Set by _nbmRegisterMetaPtrs when 'rng' is in params; stays None
    # otherwise.  The array is seeded from self.RNGSeed at Reset time and
    # mutated in place by _RngWrapper so that state persists across ticks.
    # Used by _nbmGenerateCode to emit the rng binding line.

    bskLogMinUidx: Optional[int] = None
    # User-pointer index of a np.int32[1] array holding the BSK logger minimum
    # level at the time of Reset.  Set by _nbmRegisterMetaPtrs when 'bskLogger'
    # is in params; stays None otherwise.  Storing it as a runtime userPtr
    # (rather than a compile-time global) means the log level is read each tick
    # from the pointer, so different Reset calls with different log levels share
    # the same compiled cfunc.


# ---------------------------------------------------------------------------
# NumbaModel
# ---------------------------------------------------------------------------

class NumbaModel(SysModelMixin, _NumbaModel):

    def __init__(self):
        super().__init__()
        self.memory = MemoryNamespace()

    # ------------------------------------------------------------------
    # _nbmCompile - orchestrates cfunc compilation in five steps
    # ------------------------------------------------------------------

    def _nbmCompile(self, CurrentSimNanos):
        """Introspect UpdateStateImpl, register pointers, and compile cfunc.

        Called by Reset() after the C++ base has been cleared.
        Subclasses may override individual _nbmClassifyParam / _nbmHandle*
        methods to extend which parameter names are recognised.
        """
        implFunc, params = self._nbmGetImpl()
        ctx = NbmCtx(params=params)
        for pName in params:
            self._nbmClassifyParam(pName, ctx)
        self._nbmValidateLinks(ctx)
        self._nbmRegisterMetaPtrs(ctx)
        self._nbmGenerateCode(ctx)
        self._nbmBuildCfunc(implFunc, ctx)

    # ------------------------------------------------------------------
    # Step 0 - find and validate UpdateStateImpl
    # ------------------------------------------------------------------

    def _nbmGetImpl(self):
        """Find UpdateStateImpl in the MRO, validate it, and return (func, params).

        Skips the NumbaModel stub.  Raises NotImplementedError / TypeError for
        common mistakes (missing override, not a staticmethod, uninspectable).
        """
        implDesc = None
        for klass in type(self).__mro__:
            if klass is NumbaModel:
                continue   # skip our own no-op stub
            if 'UpdateStateImpl' in klass.__dict__:
                implDesc = klass.__dict__['UpdateStateImpl']
                break

        if implDesc is None:
            raise NotImplementedError(
                f"{type(self).__name__} must override UpdateStateImpl as a @staticmethod."
            )

        if not isinstance(implDesc, staticmethod):
            raise TypeError(
                f"{type(self).__name__}.UpdateStateImpl must be a @staticmethod "
                f"(it must not have a 'self' parameter)."
            )

        implFunc = implDesc.__func__

        try:
            sig = inspect.signature(implFunc)
        except (ValueError, TypeError) as e:
            raise TypeError(
                f"Could not inspect {type(self).__name__}.UpdateStateImpl: {e}"
            ) from e

        return implFunc, list(sig.parameters.keys())

    # ------------------------------------------------------------------
    # Step 1 - classify each parameter (dispatch + per-type handlers)
    # ------------------------------------------------------------------

    def _nbmClassifyParam(self, pName, ctx):
        """Dispatch a single UpdateStateImpl parameter to its handler.

        Subclasses may override to recognise additional parameter types before
        falling through to ``super()._nbmClassifyParam(pName, ctx)``.
        """
        if pName == 'CurrentSimNanos':
            return self._nbmHandleCurrentSimNanos(pName, ctx)
        if pName.endswith('InMsgIsLinked'):
            return self._nbmHandleInMsgIsLinked(pName, ctx)
        if pName.endswith('InMsgPayload'):
            return self._nbmHandleInMsgPayload(pName, ctx)
        if pName.endswith('OutMsgPayload'):
            return self._nbmHandleOutMsgPayload(pName, ctx)
        if pName == 'moduleID':
            return self._nbmHandleModuleID(pName, ctx)
        if pName == 'memory':
            return self._nbmHandleMemory(pName, ctx)
        if pName == 'bskLogger':
            return self._nbmHandleBskLogger(pName, ctx)
        if pName == 'rng':
            return self._nbmHandleRng(pName, ctx)
        return self._nbmHandleExtraParam(pName, ctx)

    def _nbmHandleCurrentSimNanos(self, pName, ctx):
        pass   # always present in the wrapper signature; nothing to register

    def _nbmHandleInMsgIsLinked(self, pName, ctx):
        ctx.linkedSet.add(pName)

    def _nbmHandleInMsgPayload(self, pName, ctx):
        attrName = pName[:-len('Payload')]   # "dataInMsgPayload" → "dataInMsg"
        attr = getattr(self, attrName, None)
        if attr is None:
            raise AttributeError(
                f"UpdateStateImpl parameter {pName!r} implies attribute "
                f"{attrName!r}, but {type(self).__name__} has no such attribute. "
                f"Add  self.{attrName} = messaging.<Type>MsgReader()  in __init__."
            )

        if not isinstance(attr, (list, dict)):
            # Scalar reader: register via addReaderSlot so the C++ layer
            # can refresh the payload pointer and linked flag every tick.
            dt = self._nbmResolveReaderDtype(attrName, attr)
            dummy = np.zeros(1, dtype=dt)
            object.__setattr__(self, f'_{attrName}_dummy', dummy)
            self.addReaderSlot(
                attr.getPayloadPtrAddress(),
                attr.getLinkedAddress(),
                dummy.ctypes.data,
            )
            ctx.readers.append(NbmMsgEntry(pName, dt, ctx.readIdx))
            ctx.readIdx += 1
        else:
            # List / dict reader
            items = list(attr.values()) if isinstance(attr, dict) else list(attr)
            if len(items) == 0:
                # Empty container: register nothing; cfunc receives a 0-element
                # float64 array.  IsLinked is not supported for empty containers.
                lnkName = pName[:-len('Payload')] + 'IsLinked'
                if lnkName in ctx.params:
                    raise ValueError(
                        f"'{lnkName}' cannot be used with an empty {attrName!r}. "
                        f"Remove '{lnkName}' from UpdateStateImpl when the "
                        f"list/dict is empty."
                    )
                dt = np.dtype('float64')  # placeholder; no elements accessed
                _stub = np.zeros(1, dtype=np.uint8)
                bufUidx = ctx.userIdx
                self.addUserPointer(_stub.ctypes.data)
                ctx.userIdx += 1
                object.__setattr__(self, f'_{attrName}_stub', _stub)
                if isinstance(attr, dict):
                    object.__setattr__(self, f'_{attrName}Keys', ())
                ctx.listReaders.append(NbmMsgListEntry(pName, dt, 0, ctx.readIdx, bufUidx, None))
                return

            dt = self._nbmResolveReaderDtype(attrName, items[0])
            for k, item in enumerate(items[1:], 1):
                dtK = self._nbmResolveReaderDtype(attrName, item)
                if dtK != dt:
                    raise TypeError(
                        f"All messages in {attrName!r} must share the same "
                        f"payload dtype. Item 0 has dtype {dt}, "
                        f"item {k} has {dtK}."
                    )
            n    = len(items)
            base = ctx.readIdx
            # Register each item as a reader slot so C++ can refresh
            # payload pointers and linked flags dynamically every tick.
            dummyBuf = np.zeros(n, dtype=dt)
            for k, item in enumerate(items):
                elemAddr = dummyBuf.ctypes.data + k * dt.itemsize
                self.addReaderSlot(
                    item.getPayloadPtrAddress(),
                    item.getLinkedAddress(),
                    elemAddr,
                )
            ctx.readIdx += n

            # For dict attrs, build a container dtype so the cfunc can
            # expose payloads by string key: payload['a'].field
            linkedBufUidx = None
            if isinstance(attr, dict):
                keysOrNone = tuple(attr.keys())
                containerDt = np.dtype([(k, dt) for k in keysOrNone])
                payloadBuf  = np.zeros(1, dtype=containerDt)
                linkedDt    = np.dtype([(k, np.bool_) for k in keysOrNone])
                linkedBuf   = np.zeros(1, dtype=linkedDt)
            else:
                keysOrNone = None
                payloadBuf  = np.zeros(n, dtype=dt)

            bufUidx = ctx.userIdx
            self.addUserPointer(payloadBuf.ctypes.data)
            ctx.userIdx += 1
            if keysOrNone is not None:
                linkedBufUidx = ctx.userIdx
                self.addUserPointer(linkedBuf.ctypes.data)
                ctx.userIdx += 1

            # Keep alive (prevent GC)
            object.__setattr__(self, f'_{attrName}_dummy', dummyBuf)
            object.__setattr__(self, f'_{attrName}_buf',   payloadBuf)
            if keysOrNone is not None:
                object.__setattr__(self, f'_{attrName}Keys', keysOrNone)
                object.__setattr__(self, f'_{attrName}_linkedBuf', linkedBuf)

            ctx.listReaders.append(
                NbmMsgListEntry(pName, dt, n, base, bufUidx, keysOrNone, linkedBufUidx)
            )

    def _nbmHandleOutMsgPayload(self, pName, ctx):
        attrName = pName[:-len('Payload')]   # "dataOutMsgPayload" → "dataOutMsg"
        attr = getattr(self, attrName, None)
        if attr is None:
            raise AttributeError(
                f"UpdateStateImpl parameter {pName!r} implies attribute "
                f"{attrName!r}, but {type(self).__name__} has no such attribute. "
                f"Add  self.{attrName} = messaging.<Type>Msg()  in __init__."
            )

        if not isinstance(attr, (list, dict)):
            # Scalar writer
            dt = self._nbmResolveWriterDtype(attrName, attr)
            self.addWritePayloadPtr(attr.getPayloadAddress())
            self.addWriteHeaderPtr(attr.getHeaderAddress())
            ctx.writers.append(NbmMsgEntry(pName, dt, ctx.writeIdx))
            ctx.writeIdx += 1
        else:
            # List / dict writer
            items = list(attr.values()) if isinstance(attr, dict) else list(attr)
            if len(items) == 0:
                # Empty container: register nothing; cfunc receives a 0-element array.
                dt = np.dtype('float64')  # placeholder; no elements written
                _stub = np.zeros(1, dtype=np.uint8)
                bufUidx = ctx.userIdx
                self.addUserPointer(_stub.ctypes.data)
                ctx.userIdx += 1
                object.__setattr__(self, f'_{attrName}_stub', _stub)
                if isinstance(attr, dict):
                    object.__setattr__(self, f'_{attrName}Keys', ())
                ctx.listWriters.append(NbmMsgListEntry(pName, dt, 0, ctx.writeIdx, bufUidx, None))
                return

            dt = self._nbmResolveWriterDtype(attrName, items[0])
            for k, item in enumerate(items[1:], 1):
                dtK = self._nbmResolveWriterDtype(attrName, item)
                if dtK != dt:
                    raise TypeError(
                        f"All messages in {attrName!r} must share the same "
                        f"payload dtype. Item 0 has dtype {dt}, "
                        f"item {k} has {dtK}."
                    )
            n    = len(items)
            base = ctx.writeIdx
            for item in items:
                self.addWritePayloadPtr(item.getPayloadAddress())
                self.addWriteHeaderPtr(item.getHeaderAddress())
            ctx.writeIdx += n

            if isinstance(attr, dict):
                keysOrNone = tuple(attr.keys())
                containerDt = np.dtype([(k, dt) for k in keysOrNone])
                payloadBuf  = np.zeros(1, dtype=containerDt)
                object.__setattr__(self, f'_{attrName}Keys', keysOrNone)
            else:
                keysOrNone = None
                payloadBuf  = np.zeros(n, dtype=dt)

            bufUidx = ctx.userIdx
            self.addUserPointer(payloadBuf.ctypes.data)
            ctx.userIdx += 1

            object.__setattr__(self, f'_{attrName}_buf', payloadBuf)

            ctx.listWriters.append(NbmMsgListEntry(pName, dt, n, base, bufUidx, keysOrNone))

    def _nbmHandleModuleID(self, pName, ctx):
        pass

    def _nbmHandleMemory(self, pName, ctx):
        mem = object.__getattribute__(self, 'memory')
        # Sync the live buffer back to __dict__ so that cfunc writes and Python
        # writes are treated identically: the buffer is always the source of truth.
        memD = object.__getattribute__(mem, '__dict__')
        if '_nbmBuf' in memD:
            old_buf = memD['_nbmBuf']
            old_dt  = memD['_nbmDtype']
            for fname in old_dt.names:
                if fname == '_':
                    continue
                memD[fname] = np.array(old_buf[0][fname]) if old_dt[fname].shape else old_buf[0][fname].item()
        raw = {k: v for k, v in vars(mem).items() if not k.startswith('_')}
        memFields = []
        memInit   = {}
        for fname, fval in raw.items():
            arr = np.asarray(fval)
            if arr.ndim == 0:
                memFields.append((fname, arr.dtype))
                memInit[fname] = (arr.item(), False)
            else:
                arr = np.ascontiguousarray(arr)
                memFields.append((fname, arr.dtype, arr.shape))
                memInit[fname] = (arr, True)
        # Empty memory: use a 1-byte placeholder so Numba gets a valid Record.
        if not memFields:
            memFields = [('_', np.uint8)]
        memoryDt  = np.dtype(memFields)
        memoryBuf = np.zeros(1, dtype=memoryDt)
        for fname, (fval, isArr) in memInit.items():
            if isArr:
                memoryBuf[0][fname][:] = fval
            else:
                memoryBuf[0][fname] = fval
        # Activate the MemoryNamespace redirect (write directly to __dict__ to
        # avoid MemoryNamespace.__setattr__ interpreting these as user fields)
        memD['_nbmBuf']   = memoryBuf
        memD['_nbmDtype'] = memoryDt
        memoryUidx = ctx.userIdx
        self.addUserPointer(memoryBuf.ctypes.data)
        ctx.userIdx += 1
        object.__setattr__(self, '_memoryBuf', memoryBuf)  # keep alive
        ctx.memoryCodegen = NbmMemoryEntry(memoryDt, memoryUidx)

    def _nbmHandleBskLogger(self, pName, ctx):
        pass

    def _nbmHandleRng(self, pName, ctx):
        pass

    def _nbmHandleExtraParam(self, pName, ctx):
        """Called when no built-in handler matches pName.

        Raises AttributeError using _nbmValidParamPatterns().
        Subclasses should override _nbmClassifyParam (to add new handlers)
        and _nbmValidParamPatterns (to extend the error message), rather
        than overriding this method directly.
        """
        patterns = self._nbmValidParamPatterns()
        raise AttributeError(
            f"UpdateStateImpl parameter {pName!r} does not match any known pattern.\n"
            f"Valid patterns:\n" + "\n".join(f"  {p}" for p in patterns)
        )

    def _nbmValidParamPatterns(self):
        """Return a list of human-readable pattern descriptions for error messages.

        Subclasses should override this and prepend their own patterns before
        calling ``super()._nbmValidParamPatterns()``.
        """
        return [
            "'<name>InMsgPayload'   : ReadFunctor or list/dict of ReadFunctors",
            "'<name>InMsgIsLinked'  : bool flag(s) for the '<name>InMsg' reader",
            "'<name>OutMsgPayload'  : Message or list/dict of Messages",
            "'CurrentSimNanos'      : simulation time (uint64)",
            "'moduleID'             : this model's unique integer ID (int64)",
            "'memory'               : persistent state "
            "(set self.memory.* attributes in __init__)",
            "'bskLogger'            : logging proxy "
            "(bskLog/bskLog1/bskLog2/bskLog3)",
            "'rng'                  : per-module numpy Generator "
            "(seeded from self.RNGSeed at Reset)",
        ]

    # ------------------------------------------------------------------
    # Step 1 helpers - dtype resolution (shared by In/Out handlers)
    # ------------------------------------------------------------------

    @staticmethod
    def _nbmResolveReaderDtype(attrName, item):
        if not hasattr(item, 'subscribeTo'):
            raise TypeError(
                f"Attribute {attrName!r} must contain ReadFunctor(s) (InMsg), "
                f"e.g. messaging.CModuleTemplateMsgReader(), "
                f"but got {type(item).__name__!r}."
            )
        swigType = type(item).__name__
        if not swigType.endswith('Reader'):
            raise TypeError(
                f"Cannot derive payload class from {attrName!r}: SWIG type "
                f"{swigType!r} does not end with 'Reader'. "
                f"Ensure {attrName!r} is a SWIG-generated ReadFunctor."
            )
        pclsName = swigType[:-len('Reader')] + 'Payload'
        return NumbaModel._nbmLookupPayloadDtype(attrName, swigType, pclsName)

    @staticmethod
    def _nbmResolveWriterDtype(attrName, item):
        if not hasattr(item, 'addSubscriber'):
            raise TypeError(
                f"Attribute {attrName!r} must contain Message(s) (OutMsg), "
                f"e.g. messaging.CModuleTemplateMsg(), "
                f"but got {type(item).__name__!r}."
            )
        swigType = type(item).__name__
        pclsName = swigType + 'Payload'
        return NumbaModel._nbmLookupPayloadDtype(attrName, swigType, pclsName)

    @staticmethod
    def _nbmLookupPayloadDtype(attrName, swigType, pclsName):
        """Look up messaging.<pclsName> and return its __dtype__, with clear errors."""
        PayloadClass = getattr(messaging, pclsName, None)
        if PayloadClass is None:
            raise AttributeError(
                f"messaging.{pclsName} does not exist. "
                f"Derived from SWIG type {swigType!r}; check that the message "
                f"type is generated and imported correctly."
            )
        if PayloadClass.__dtype__ is None:
            raise TypeError(
                f"messaging.{pclsName}.__dtype__ is None - this payload contains "
                f"C++ types (e.g. std::vector) that cannot be represented as a "
                f"numpy dtype. NumbaModel cannot handle it."
            )
        return PayloadClass.__dtype__

    # ------------------------------------------------------------------
    # Step 2 - validate IsLinked constraints
    # ------------------------------------------------------------------

    def _nbmValidateLinks(self, ctx):
        """Assert that non-guarded readers are subscribed."""
        # Declaring *InMsgPayload without *InMsgIsLinked asserts the reader is
        # always connected; raise a clear error at Reset time if it is not.
        for e in ctx.readers:
            lnk = e.pName[:-len('Payload')] + 'IsLinked'
            if lnk not in ctx.linkedSet:
                attrName = e.pName[:-len('Payload')]
                attr = getattr(self, attrName)
                if not attr.isLinked():
                    raise RuntimeError(
                        f"UpdateStateImpl declares {e.pName!r} without a corresponding "
                        f"'{lnk}' guard, but self.{attrName} is not subscribed.\n"
                        f"Either call  self.{attrName}.subscribeTo(...)  before "
                        f"Reset(), or add '{lnk}' to UpdateStateImpl to handle "
                        f"the unlinked case gracefully."
                    )

        for e in ctx.listReaders:
            if e.n == 0:
                continue   # empty container, nothing to check
            lnk = e.pName[:-len('Payload')] + 'IsLinked'
            if lnk not in ctx.linkedSet:
                attrName = e.pName[:-len('Payload')]
                attr = getattr(self, attrName)
                itemsCheck = list(attr.values()) if isinstance(attr, dict) else list(attr)
                for k, item in enumerate(itemsCheck):
                    if not item.isLinked():
                        keyDesc = (repr(list(attr.keys())[k])
                                    if isinstance(attr, dict) else str(k))
                        raise RuntimeError(
                            f"UpdateStateImpl declares {e.pName!r} without a "
                            f"corresponding '{lnk}' guard, but "
                            f"self.{attrName}[{keyDesc}] is not subscribed.\n"
                            f"Either subscribe it before Reset(), or add '{lnk}' "
                            f"to UpdateStateImpl to handle the unlinked case."
                        )

    # ------------------------------------------------------------------
    # Step 3 - register post-loop C++ user pointers
    # ------------------------------------------------------------------

    def _nbmRegisterMetaPtrs(self, ctx):
        """Register the readLinked_ array, moduleID, and rng user pointers.

        Must run after all addReaderSlot() calls (step 1) so that
        readLinked_.data() is stable.
        """
        # readLinked_ array - one uint8 per registered reader slot
        if ctx.readIdx > 0:
            ctx.linkedUidx = ctx.userIdx
            self.addUserPointer(self.getReadLinkedPtr())
            ctx.userIdx += 1

        # moduleID scalar (int64)
        if 'moduleID' in ctx.params:
            moduleIdArr = np.array([self.moduleID], dtype=np.int64)
            object.__setattr__(self, '_moduleIdArr', moduleIdArr)
            ctx.moduleIdUidx = ctx.userIdx
            self.addUserPointer(moduleIdArr.ctypes.data)
            ctx.userIdx += 1

        # xoshiro256++ RNG state (uint64[4])
        if 'rng' in ctx.params:
            rngState = _seedXoshiro256pp(self.RNGSeed)
            object.__setattr__(self, '_rng_state', rngState)   # keep alive
            ctx.rngUidx = ctx.userIdx
            self.addUserPointer(rngState.ctypes.data)
            ctx.userIdx += 1

        # bskLogger minimum level (int32[1]) — runtime value, not compile-time constant
        if 'bskLogger' in ctx.params:
            bskLogMinArr = np.array([np.int32(self.bskLogger.getLogLevel())], dtype=np.int32)
            object.__setattr__(self, '_bskLogMinArr', bskLogMinArr)  # keep alive
            ctx.bskLogMinUidx = ctx.userIdx
            self.addUserPointer(bskLogMinArr.ctypes.data)
            ctx.userIdx += 1

    # ------------------------------------------------------------------
    # Step 4 - build the _wrapper source and compile globals
    # ------------------------------------------------------------------

    def _nbmGenerateCode(self, ctx):
        """Append variable-binding lines to ctx.preLines / ctx.postLines."""
        g         = ctx.g
        preLines = ctx.preLines
        postLines = ctx.postLines

        # Absolute base offsets into the flat allPtrs array.
        # _WRITE_BASE and _USER_BASE are stored as integer constants in g so
        # that code added to preLines by subclass handlers (before this method
        # runs) can reference them by name and have them resolved at exec() time.
        g['_WRITE_BASE'] = ctx.readIdx
        g['_USER_BASE']  = ctx.readIdx + ctx.writeIdx

        # Scalar readers
        for e in ctx.readers:
            g[f'_rdt_{e.idx}'] = e.dtype
            preLines.append(
                f"    {e.pName} = nb.carray(allPtrs[{e.idx}], 1, _rdt_{e.idx})[0]"
            )
            lnk = e.pName[:-len('Payload')] + 'IsLinked'
            if lnk in ctx.linkedSet:
                preLines.append(
                    f"    {lnk} = nb.carray(allPtrs[_USER_BASE + {ctx.linkedUidx}], {ctx.readIdx}, np.uint8)[{e.idx}] != np.uint8(0)"
                )

        # List / dict readers
        for e in ctx.listReaders:
            g[f'_rdt_{e.base}'] = e.dtype
            if e.n > 0:
                preLines.append(
                    f"    _{e.pName}_lnk = nb.carray(allPtrs[_USER_BASE + {ctx.linkedUidx}], {ctx.readIdx}, np.uint8)[{e.base}:{e.base+e.n}]"
                )
            if e.keys is None:
                preLines.append(
                    f"    _{e.pName}_buf = nb.carray(allPtrs[_USER_BASE + {e.bufUidx}], ({e.n},), _rdt_{e.base})"
                )
                if e.n > 0:
                    preLines.append(f"    for _i_{e.base} in range({e.n}):")
                    preLines.append(f"        if _{e.pName}_lnk[_i_{e.base}]:")
                    preLines.append(
                        f"            _{e.pName}_buf[_i_{e.base}] = "
                        f"nb.carray(allPtrs[{e.base} + _i_{e.base}], 1, _rdt_{e.base})[0]"
                    )
                preLines.append(f"    {e.pName} = _{e.pName}_buf")
            else:
                g[f'_cdt_{e.base}'] = np.dtype([(k, e.dtype) for k in e.keys])
                preLines.append(
                    f"    _{e.pName}_ctbuf = nb.carray(allPtrs[_USER_BASE + {e.bufUidx}], 1, _cdt_{e.base})"
                )
                for k_i, k in enumerate(e.keys):
                    preLines.append(f"    if _{e.pName}_lnk[{k_i}]:")
                    preLines.append(
                        f"        _{e.pName}_ctbuf[0]['{k}'] = "
                        f"nb.carray(allPtrs[{e.base + k_i}], 1, _rdt_{e.base})[0]"
                    )
                preLines.append(f"    {e.pName} = _{e.pName}_ctbuf[0]")
            lnk = e.pName[:-len('Payload')] + 'IsLinked'
            if lnk in ctx.linkedSet:
                if e.keys is not None:
                    g[f'_lcdt_{e.base}'] = np.dtype([(k, np.bool_) for k in e.keys])
                    preLines.append(
                        f"    _lnkbuf_{e.base} = nb.carray("
                        f"allPtrs[_USER_BASE + {e.linkedBufUidx}], 1, _lcdt_{e.base})"
                    )
                    for k_i, k in enumerate(e.keys):
                        preLines.append(
                            f"    _lnkbuf_{e.base}[0]['{k}'] = "
                            f"_{e.pName}_lnk[{k_i}] != np.uint8(0)"
                        )
                    preLines.append(f"    {lnk} = _lnkbuf_{e.base}[0]")
                else:
                    preLines.append(f"    {lnk} = _{e.pName}_lnk")

        # Scalar writers
        for e in ctx.writers:
            g[f'_wdt_{e.idx}'] = e.dtype
            preLines.append(
                f"    {e.pName} = nb.carray(allPtrs[_WRITE_BASE + {e.idx}], 1, _wdt_{e.idx})[0]"
            )

        # List / dict writers - postLines carries copy-back after _impl
        for e in ctx.listWriters:
            g[f'_wdt_{e.base}'] = e.dtype
            if e.keys is not None:
                g[f'_cdtW{e.base}'] = np.dtype([(k, e.dtype) for k in e.keys])
                preLines.append(
                    f"    _{e.pName}_ctbuf = nb.carray(allPtrs[_USER_BASE + {e.bufUidx}], 1, _cdtW{e.base})"
                )
                preLines.append(f"    {e.pName} = _{e.pName}_ctbuf[0]")
                for k_i, k in enumerate(e.keys):
                    postLines.append(
                        f"    nb.carray(allPtrs[_WRITE_BASE + {e.base + k_i}], 1, _wdt_{e.base})[0]"
                        f" = _{e.pName}_ctbuf[0]['{k}']"
                    )
            else:
                preLines.append(
                    f"    _{e.pName}_buf = nb.carray(allPtrs[_USER_BASE + {e.bufUidx}], ({e.n},), _wdt_{e.base})"
                )
                preLines.append(f"    {e.pName} = _{e.pName}_buf")
                if e.n > 0:
                    postLines.append(f"    for _j_{e.base} in range({e.n}):")
                    postLines.append(
                        f"        nb.carray(allPtrs[_WRITE_BASE + {e.base} + _j_{e.base}], 1, _wdt_{e.base})[0]"
                        f" = _{e.pName}_buf[_j_{e.base}]"
                    )

        # Memory
        if ctx.memoryCodegen is not None:
            g['_memoryDt'] = ctx.memoryCodegen.dtype
            preLines.append(
                f"    memory = nb.carray(allPtrs[_USER_BASE + {ctx.memoryCodegen.uidx}], 1, _memoryDt)[0]"
            )

        # moduleID
        if ctx.moduleIdUidx is not None:
            preLines.append(
                f"    moduleID = nb.carray(allPtrs[_USER_BASE + {ctx.moduleIdUidx}], 1, np.int64)[0]"
            )

        # rng - xoshiro256++ state backed by userPtr; instance created per tick
        if ctx.rngUidx is not None:
            g['_RngWrapper'] = _RngWrapper
            preLines.append(
                f"    rng = _RngWrapper(nb.carray(allPtrs[_USER_BASE + {ctx.rngUidx}], 4, np.uint64))"
            )

        # bskLogger, log level read from userPtr at runtime; tags are compile-time
        if 'bskLogger' in ctx.params:
            _lmap = self.bskLogger.logLevelMap
            g['BskLoggerProxy'] = BskLoggerProxy
            g['_bsklog_t0']     = _lmap[0]
            g['_bsklog_t1']     = _lmap[1]
            g['_bsklog_t2']     = _lmap[2]
            g['_bsklog_t3']     = _lmap[3]
            preLines.append(
                f"    bskLogger = BskLoggerProxy("
                f"nb.carray(allPtrs[_USER_BASE + {ctx.bskLogMinUidx}], 1, np.int32)[0],"
                " _bsklog_t0, _bsklog_t1, _bsklog_t2, _bsklog_t3)"
            )

    # ------------------------------------------------------------------
    # Step 5 - compile and register the cfunc
    # ------------------------------------------------------------------

    def _nbmBuildCfunc(self, implFunc, ctx):
        ctx.preLines.append(f"    _impl({', '.join(ctx.params)})")
        _co = implFunc.__code__
        _implCanCache = (not _co.co_filename.startswith('<')
                         and os.path.isfile(_co.co_filename))
        ctx.g['_impl'] = nb.njit(cache=_implCanCache)(implFunc)

        # Content-based fingerprint: (implCodeHash, dtypeHash, rawSrc).
        # implCodeHash covers identity (co_filename, co_qualname) AND body (co_code,
        # co_consts) — so moving the file, renaming the class, OR changing the body
        # all produce a new fp and trigger a fresh compile.
        rawSrc = "\n".join(ctx.preLines + ctx.postLines)

        implCodeHash = hashlib.sha256(marshal.dumps(implFunc.__code__)).hexdigest()[:16]

        dtypeParts = ([str(e.dtype) for e in ctx.readers + ctx.writers] +
                      [f"{e.n}:{e.dtype}" for e in ctx.listReaders + ctx.listWriters])
        if ctx.memoryCodegen:
            dtypeParts.append(str(ctx.memoryCodegen.dtype))
        dtypeHash = hashlib.md5('|'.join(dtypeParts).encode()).hexdigest()

        fp = hashlib.sha256(
            '\n'.join([implCodeHash, dtypeHash, rawSrc]).encode()
        ).hexdigest()[:16]

        # Path 1: in-process cache hit
        if fp in _NBMODEL_CFUNC_CACHE:
            self._cfunc = _NBMODEL_CFUNC_CACHE[fp]
            self.setStateUpdateFunc(self._cfunc.address)
            return

        # Path 2+3: disk cache (with fallback to raw compile)
        cfunc = None
        cacheDir = getCacheDir()
        if cacheDir is not None:
            try:
                srcPath = cacheDir / f'nbm_{fp}.py'
                modName = f'_nbm_{fp}'

                if not srcPath.exists():
                    tmp = srcPath.with_suffix('.tmp')
                    tmp.write_text(rawSrc, encoding='utf-8')
                    os.replace(tmp, srcPath)   # atomic on Linux + Windows

                spec = importlib.util.spec_from_file_location(modName, srcPath)
                mod  = importlib.util.module_from_spec(spec)
                mod.__dict__.update(ctx.g)
                spec.loader.exec_module(mod)
                sys.modules[modName] = mod   # required before cfunc(cache=True)

                cfuncSig = nb.types.void(nb.types.CPointer(nb.types.voidptr), nb.types.uint64)
                cfunc = nb.cfunc(cfuncSig, cache=True)(mod._wrapper)
            except Exception:
                cfunc = None

        if cfunc is None:   # fallback: read-only location or I/O failure
            exec(rawSrc, ctx.g)
            cfuncSig = nb.types.void(nb.types.CPointer(nb.types.voidptr), nb.types.uint64)
            cfunc = nb.cfunc(cfuncSig)(ctx.g['_wrapper'])

        self._cfunc = cfunc
        _NBMODEL_CFUNC_CACHE[fp] = cfunc
        self.setStateUpdateFunc(cfunc.address)

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def Reset(self, CurrentSimNanos=0):
        # C++ Reset clears all pointer vectors
        super().Reset(CurrentSimNanos)
        self._nbmCompile(CurrentSimNanos)
        # Assemble flat allPtrs_ from [reads | writes | user] staging vectors
        self.finalizeAllPtrs()

    @staticmethod
    def UpdateStateImpl(*a, **k):
        """Override with a @staticmethod that numba can compile."""
        pass
%}

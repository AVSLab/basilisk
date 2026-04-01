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

%module(directors="1", threads="1", package="Basilisk.simulation") StatefulNumbaModel

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
#include "statefulNumbaModel.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

%include "std_string.i"
%include "stdint.i"

/* Import parent SWIG modules so SWIG knows about base class types */
%import "architecture/numbaModel/numbaModel.i"
%import "simulation/mujocoDynamics/_GeneralModuleFiles/pyStatefulSysModel.i"

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

/* Free helper functions to access StateData's public Eigen data pointers.
 * stateData.h is transitively included via statefulNumbaModel.h → StatefulSysModel.h
 * → dynParamManager.h → stateData.h, so StateData is already visible. */
%inline %{
    uintptr_t getStateDataPtr(StateData* sd) {
        return reinterpret_cast<uintptr_t>(sd->state.data());
    }
    uintptr_t getDerivDataPtr(StateData* sd) {
        return reinterpret_cast<uintptr_t>(sd->stateDeriv.data());
    }
%}

/* Director: allow Python to override registerStates.
   UpdateState must NOT go through the director - C++ calls the cfunc directly. */
%feature("director") StatefulNumbaModel;
%feature("nodirector") StatefulNumbaModel::UpdateState;
%rename("_StatefulNumbaModel") StatefulNumbaModel;

%include "statefulNumbaModel.h"

/* Re-expose NumbaModel methods directly on StatefulNumbaModel so that
 * Python calls go through _pyStatefulNumbaModel.so (which uses StatefulNumbaModel*)
 * rather than _numbaModel.so (which requires a cross-module type cast that may
 * silently operate on the wrong C++ subobject in some SWIG runtime configurations).
 * _StatefulNumbaModel appears before _NumbaModel in the Python MRO, so these
 * %extend methods take priority over the inherited _NumbaModel wrappers. */
%extend StatefulNumbaModel {
    void addReaderSlot(uintptr_t payloadPtrAddr, uintptr_t linkedAddr, uintptr_t dummyAddr) {
        self->NumbaModel::addReaderSlot(payloadPtrAddr, linkedAddr, dummyAddr);
    }
    void addWritePayloadPtr(uintptr_t ptr) {
        self->NumbaModel::addWritePayloadPtr(ptr);
    }
    void addWriteHeaderPtr(uintptr_t ptr) {
        self->NumbaModel::addWriteHeaderPtr(ptr);
    }
    void addUserPointer(uintptr_t ptr) {
        self->NumbaModel::addUserPointer(ptr);
    }
    void setStateUpdateFunc(uintptr_t funcPtr) {
        self->NumbaModel::setStateUpdateFunc(funcPtr);
    }
    uintptr_t getReadLinkedPtr() {
        return self->NumbaModel::getReadLinkedPtr();
    }
    void finalizeAllPtrs() {
        self->NumbaModel::finalizeAllPtrs();
    }
}

%pythoncode %{
import numpy as np
import numba as nb
from Basilisk.architecture.numbaModel import NumbaModel
from Basilisk.architecture.sysModel import SysModelMixin

class StatefulNumbaModel(NumbaModel, _StatefulNumbaModel):
    """A SysModel with JIT-compiled UpdateStateImpl and continuous-time states.

    Combines NumbaModel (Numba cfunc compilation) with StatefulSysModel
    (continuous-time states integrated by MJScene's RK4 solver).

    Usage
    -----
    class MyModel(StatefulNumbaModel):
        def registerStates(self, registerer):
            self.posState = registerer.registerState(3, 1, "pos")
            self.velState = registerer.registerState(3, 1, "vel")

        def Reset(self, CurrentSimNanos=0):
            super().Reset(CurrentSimNanos)
            self.memory.omega = 2.0

        @staticmethod
        def UpdateStateImpl(posState, posStateDeriv, velState, velStateDeriv, memory):
            posStateDeriv[:, 0] = velState[:, 0]
            velStateDeriv[:, 0] = -memory.omega**2 * posState[:, 0]

    Parameter naming in UpdateStateImpl
    ------------------------------------
    All NumbaModel parameters are supported, plus:

    '<name>State'
        Read-only current state value.  Resolved from self.<name>State
        (a StateData*).  Type: float64[nRow, nCol] Fortran-order 2-D array.

    '<name>StateDeriv'
        Writable state derivative.  Resolved from self.<name>State
        (same StateData* as '<name>State').
        Type: float64[nRow, nCol] Fortran-order 2-D array.

    States are registered in registerStates(), which is called by MJScene
    before Reset().  The Eigen MatrixXd backing store is column-major
    (Fortran order), so arr[i, j] = M(i, j).  Even 1-D states are exposed
    as 2-D arrays (e.g. a 3x1 column vector: posState[i, 0]).
    """

    def _nbmHandleExtraParam(self, pName, ctx):
        # 'StateDeriv' is checked before 'State' so that the longer suffix wins
        # (e.g. 'posStateDeriv' must not be mis-handled as 'posStateDeriv'[:-5]='posState').
        if pName.endswith('StateDeriv'):
            attrName = pName[:-len('Deriv')]   # "posStateDeriv" → "posState"
            stateObj = getattr(self, attrName, None)
            if stateObj is None:
                raise AttributeError(
                    f"UpdateStateImpl parameter {pName!r} implies attribute "
                    f"{attrName!r}, but {type(self).__name__} has no such attribute.\n"
                    f"Ensure self.{attrName} was assigned in registerStates() "
                    f"before Reset()."
                )
            ptr = getDerivDataPtr(stateObj)
        elif pName.endswith('State'):
            attrName = pName                   # "posState" → "posState"
            stateObj = getattr(self, attrName, None)
            if stateObj is None:
                raise AttributeError(
                    f"UpdateStateImpl parameter {pName!r} implies attribute "
                    f"{attrName!r}, but {type(self).__name__} has no such attribute.\n"
                    f"Ensure self.{attrName} was assigned in registerStates() "
                    f"before Reset()."
                )
            ptr = getStateDataPtr(stateObj)
        else:
            super()._nbmHandleExtraParam(pName, ctx)
            return
        nRow = stateObj.getRowSize()
        nCol = stateObj.getColumnSize()
        self.addUserPointer(ptr)
        ctx.preLines.append(
            f"    {pName} = nb.farray(allPtrs[_USER_BASE + {ctx.userIdx}], ({nRow}, {nCol}), np.float64)"
        )
        ctx.userIdx += 1

    def _nbmValidParamPatterns(self):
        return [
            "'<name>State'          : read-only state value "
            "(float64[nRow, nCol] Fortran-order)",
            "'<name>StateDeriv'     : writable derivative "
            "(float64[nRow, nCol] Fortran-order)",
        ] + super()._nbmValidParamPatterns()

    def registerStates(self, registerer):
        """Override to register continuous-time states on the given registerer."""
        pass
%}

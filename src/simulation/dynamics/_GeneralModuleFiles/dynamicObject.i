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

%module(package="Basilisk.simulation") dynamicObject

%{
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
%}

%include "architecture/utilities/bskException.swg"
%include "simulation/dynamics/_GeneralModuleFiles/dynParamManagerImport.swg"
%include "sys_model.i"

// Directors and native lifecycle checks must translate exceptions at this boundary.
%default_bsk_exception(catch (const std::exception& error) {
    SWIG_exception(SWIG_RuntimeError, error.what());
});
%ignore StateVecIntegrator::integrate;
%ignore StateVecIntegrator::getDynamics;
%include "simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h"

// Track proxies handed to C++ or returned as borrowed access. The SWIG thisown
// flag alone cannot distinguish these from a new, explicitly disowned integrator.
// Keep the marker on each proxy even after rejection, replacement or destruction;
// checking it does not dereference a potentially dangling native pointer.
%pythonprepend DynamicObject::setIntegrator %{
    if getattr(newIntegrator, "_bsk_integrator_borrowed", False):
        current_integrator = self.getIntegrator()
        if current_integrator is None or int(current_integrator.this) != int(newIntegrator.this):
            from Basilisk.architecture.bskLogging import BasiliskError
            raise BasiliskError("New integrator is already owned, borrowed, or no longer valid")
    # Mark before calling C++ because it also destroys newly rejected integrators.
    if hasattr(newIntegrator, "thisown"):
        object.__setattr__(newIntegrator, "_bsk_integrator_borrowed", True)
%}

%pythonappend DynamicObject::getIntegrator %{
    if val is not None:
        object.__setattr__(val, "_bsk_integrator_borrowed", True)
%}

// Keep Python-owned secondaries alive without making them own the primary.
// All references stay visible to Python's garbage collector. Retain distinct
// proxies too: a borrowed alias must not replace an existing owning proxy.
// A borrowed primary proxy can disappear while the native primary is still alive,
// so reject it before C++ creates a connection whose retention would be temporary.
%pythonprepend DynamicObject::syncDynamicsIntegration %{
    if not self.thisown:
        from Basilisk.architecture.bskLogging import BasiliskError
        raise BasiliskError(
            "Configure synchronized dynamics through the owning Python primary object; "
            "borrowed primary proxies cannot retain connections"
        )
%}
%pythonappend DynamicObject::syncDynamicsIntegration %{
    synchronized_dynamics = getattr(self, "_bsk_synced_dynamics", None)
    if synchronized_dynamics is None:
        synchronized_dynamics = []
        object.__setattr__(self, "_bsk_synced_dynamics", synchronized_dynamics)
    if not any(secondary is dynPtr for secondary in synchronized_dynamics):
        synchronized_dynamics.append(dynPtr)
%}

// Transfer Python ownership to the C++ unique_ptr so dropping the Python
// reference later does not free the integrator a second time.
// C++ consumes newly supplied integrators, including rejected replacements.
%apply SWIGTYPE *DISOWN { StateVecIntegrator* newIntegrator };
%include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
%clear StateVecIntegrator* newIntegrator;

%pythoncode %{
DynamicObject.integrator = property(DynamicObject.getIntegrator, DynamicObject.setIntegrator)
DynamicObject.isDynamicsSynced = property(lambda self: self.getIntegrationOwner() is not None)

_COMPATIBILITY_EXPORTS = (
    "SysModel",
    "DynamicObject",
    "StateVecIntegrator",
)

def _exportCompatibilityAPI(namespace):
    for name in _COMPATIBILITY_EXPORTS:
        namespace[name] = globals()[name]

import sys as _sys
from Basilisk.architecture.swig_common_model import protectAllClasses
protectAllClasses(_sys.modules[__name__])
del _sys
%}

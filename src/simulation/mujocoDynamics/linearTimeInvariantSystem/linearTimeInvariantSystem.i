/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab

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

%module(directors="1",threads="1") MJLinearTimeInvariantSystem

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
  // C++ includes visible to the generated wrapper
  #include "linearTimeInvariantSystem.h"
  #include "forceAtSiteLTI.h"
  #include "singleActuatorLTI.h"
%}

/* Python helpers consistent with your other modules */
%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

/* Common SWIG helpers */
%include "swig_eigen.i"
%include "std_string.i"
%include "exception.i"

/* Basilisk system-model base and helpers */
%import "simulation/mujocoDynamics/_GeneralModuleFiles/pyStatefulSysModel.i"

/* ============================
   Base class: LinearTimeInvariantSystem
   ============================ */

/* Enable directors so Python subclasses can override virtual methods
   (readInput, writeOutput, etc.). */
%feature("director") LinearTimeInvariantSystem;

%rename("_LinearTimeInvariantSystem") LinearTimeInvariantSystem;
%include "linearTimeInvariantSystem.h"

/* ============================
   SingleActuatorLTI
   ============================ */

%ignore SingleActuatorLTI::readInput;
%ignore SingleActuatorLTI::writeOutput;
%include "singleActuatorLTI.h"

%include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
struct SingleActuatorMsgPayload_C;

/* ============================
   ForceAtSiteLTI
   ============================ */

%ignore ForceAtSiteLTI::readInput;
%ignore ForceAtSiteLTI::writeOutput;
%include "forceAtSiteLTI.h"

%include "architecture/msgPayloadDefC/ForceAtSiteMsgPayload.h"
struct ForceAtSiteMsgPayload_C;

/* Final Python-side wrapper for the base LTI class */
%pythoncode %{
from Basilisk.architecture.sysModel import SysModelMixin

class LinearTimeInvariantSystem(SysModelMixin, _LinearTimeInvariantSystem):
    """Python wrapper for the C++ LinearTimeInvariantSystem."""
%}

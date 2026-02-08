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


%module NBodyGravity
%{
   #include "NBodyGravity.h"
   #include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
   #include "simulation/mujocoDynamics/_GeneralModuleFiles/MJInterpolators.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "swig_eigen.i"
%include "std_string.i"
%include "exception.i"

%import "simulation/mujocoDynamics/_GeneralModuleFiles/mujoco.i"
%import "simulation/dynamics/_GeneralModuleFiles/gravityModel.i"

%include "sys_model.i"
%include "NBodyGravity.h"

%include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
struct SpicePlanetStateMsgPayload_C;
%include "architecture/msgPayloadDefC/SCMassPropsMsgPayload.h"
struct SCMassPropsMsgPayload_C;
%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsgPayload_C;
%include "architecture/msgPayloadDefC/ForceAtSiteMsgPayload.h"
struct ForceAtSiteMsgPayload_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

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


%module MJPIDControllers
%{
   #include "JointPIDController.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "swig_eigen.i"
%include "std_string.i"
%include "exception.i"

%include "sys_model.i"
%include "simulation/mujocoDynamics/_GeneralModuleFiles/StatefulSysModel.h"
%include "simulation/mujocoDynamics/_GeneralModuleFiles/PIDController.h"

// JointPIDController
%template(_JointPIDControllerBase) PIDController<ScalarJointStateMsgPayload, ScalarJointStateMsgPayload, SingleActuatorMsgPayload>;

%ignore JointPIDController::readMeasuredPosition;
%ignore JointPIDController::readMeasuredVelocity;
%ignore JointPIDController::readDesiredPosition;
%ignore JointPIDController::readDesiredVelocity;
%ignore JointPIDController::writeOutput;

%include "JointPIDController.h"

// Messages
%include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
struct ScalarJointStateMsgPayload_C;
%include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
struct SingleActuatorMsgPayload_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

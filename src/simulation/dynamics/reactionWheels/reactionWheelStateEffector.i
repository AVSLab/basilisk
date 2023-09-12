/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
%module reactionWheelStateEffector
%{
   #include "reactionWheelStateEffector.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_eigen.i"
%include "swig_conly_data.i"

%include "sys_model.i"
%include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
%include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
%include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
%include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
%include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
%include "simulation/dynamics/reactionWheels/reactionWheelSupport.h"
%include "reactionWheelStateEffector.h"
%include "architecture/utilities/macroDefinitions.h"

%include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
struct RWSpeedMsg_C;
%include "architecture/msgPayloadDefC/RWCmdMsgPayload.h"
struct RWCmdMsg_C;
%include "architecture/msgPayloadDefCpp/RWConfigMsgPayload.h"
%include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
struct RWConfigLogMsg_C;
%include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
struct ArrayMotorTorqueMsg_C;

%include "std_vector.i"
namespace std {
    %template(RWConfigPointerVector) vector<RWConfigMsgPayload *>;
}

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

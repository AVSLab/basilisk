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
from Basilisk.simulation.swig_common_model import *
%}
%include "std_string.i"
%include "swig_eigen.i"
%include "swig_conly_data.i"

%include "sys_model.h"
%include "../_GeneralModuleFiles/stateData.h"
%include "../_GeneralModuleFiles/stateEffector.h"
%include "../_GeneralModuleFiles/dynamicEffector.h"
%include "../_GeneralModuleFiles/dynParamManager.h"
%include "../_GeneralModuleFiles/dynamicObject.h"
%include "dynamics/reactionWheels/reactionWheelSupport.h"

%include "reactionWheelStateEffector.h"
%include "utilities/macroDefinitions.h"

%include "msgPayloadDefC/RWSpeedMsgPayload.h"
struct RWSpeedMsg_C;
%include "msgPayloadDefC/RWCmdMsgPayload.h"
struct RWCmdMsg_C;
%include "msgPayloadDefCpp/RWConfigMsgPayload.h"
struct RWConfigMsg_C;
%include "msgPayloadDefC/RWConfigLogMsgPayload.h"
struct RWConfigLogMsg_C;
%include "msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
struct ArrayMotorTorqueMsg_C;

%include "std_vector.i"
namespace std {
    %template(RWConfigVector) vector<RWConfigMsgPayload>;
	%template(RWCmdVector) vector<RWCmdMsgPayload>;
}

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

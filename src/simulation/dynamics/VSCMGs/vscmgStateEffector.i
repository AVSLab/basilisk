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
%module vscmgStateEffector
%{
   #include "vscmgStateEffector.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_eigen.i"
%include "swig_conly_data.i"

%include "sys_model.i"
%include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.i"
%include "../_GeneralModuleFiles/stateEffector.h"
%include "../_GeneralModuleFiles/dynamicEffector.h"
%include "vscmgStateEffector.h"

%include "architecture/msgPayloadDefC/VSCMGCmdMsgPayload.h"
struct VSCMGCmdMsg_C;
%include "architecture/msgPayloadDefC/VSCMGSpeedMsgPayload.h"
struct VSCMGSpeedMsg_C;
%include "architecture/msgPayloadDefC/VSCMGArrayTorqueMsgPayload.h"
struct VSCMGArrayTorqueMsg_C;
%include "architecture/msgPayloadDefCpp/VSCMGConfigMsgPayload.h"

%include "architecture/utilities/macroDefinitions.h"

%include "std_vector.i"

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

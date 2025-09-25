/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

%module jointedArmThrFiringMotor

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "jointedArmThrFiringMotor.h"
%}

%include "std_vector.i"
%template(DoubleVector) std::vector<double>;

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"

%include "sys_model.i"
%include "jointedArmThrFiringMotor.h"

%include "architecture/msgPayloadDefC/MJSysMassMatrixMsgPayload.h"
struct MJSysMassMatrixMsg_C;
%include "architecture/msgPayloadDefC/MJNonActuatorForcesMsgPayload.h"
struct MJNonActuatorForcesMsg_C;
%include "architecture/msgPayloadDefC/CmdForceBodyMsgPayload.h"
struct CmdForceBodyMsg_C;
%include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
struct CmdTorqueBodyMsg_C;
%include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
struct ArrayMotorTorqueMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

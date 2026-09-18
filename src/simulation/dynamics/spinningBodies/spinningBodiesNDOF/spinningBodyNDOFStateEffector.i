/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


%module spinningBodyNDOFStateEffector

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "spinningBodyNDOFStateEffector.h"
    #include <memory>
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

%include "std_string.i"
%include "attribute.i"
%attribute(SpinningBodyNDOFStateEffector, std::string, nameOfThetaState, getNameOfThetaState, setNameOfThetaState);
%rename(getNameOfThetaState) SpinningBodyNDOFStateEffector::getNameOfThetaState() const;
%rename(setNameOfThetaState) SpinningBodyNDOFStateEffector::setNameOfThetaState;
%attribute(SpinningBodyNDOFStateEffector, std::string, nameOfThetaDotState, getNameOfThetaDotState, setNameOfThetaDotState);
%rename(getNameOfThetaDotState) SpinningBodyNDOFStateEffector::getNameOfThetaDotState() const;
%rename(setNameOfThetaDotState) SpinningBodyNDOFStateEffector::setNameOfThetaDotState;
%attribute(SpinningBody, std::string, nameOfInertialPositionProperty, getNameOfInertialPositionProperty, setNameOfInertialPositionProperty);
%attribute(SpinningBody, std::string, nameOfInertialVelocityProperty, getNameOfInertialVelocityProperty, setNameOfInertialVelocityProperty);
%attribute(SpinningBody, std::string, nameOfInertialAttitudeProperty, getNameOfInertialAttitudeProperty, setNameOfInertialAttitudeProperty);
%attribute(SpinningBody, std::string, nameOfInertialAngVelocityProperty, getNameOfInertialAngVelocityProperty, setNameOfInertialAngVelocityProperty);

%include "std_vector.i"
%include "swig_conly_data.i"
%include "swig_eigen.i"

%include <std_shared_ptr.i>
%shared_ptr(SpinningBody)

%include "sys_model.i"
%include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.i"
%include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
%include "spinningBodyNDOFStateEffector.h"

%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;
%include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
struct ArrayMotorTorqueMsg_C;
%include "architecture/msgPayloadDefC/ArrayEffectorLockMsgPayload.h"
struct ArrayEffectorLockMsg_C;
%include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
struct HingedRigidBodyMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

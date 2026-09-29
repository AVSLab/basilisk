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

%{
   #include "MJJoint.h"
%}

%include "MJObject.swg"
%template_mujoco_object(mjsJoint)

// Retain the owner on borrowed results using the generated Python methods.
%pythonappend MJJoint::getBody %{
    if val is not None:
        val._swig_bsk_owner = getattr(self, "_swig_bsk_owner", self)
%}
%pythonappend MJScalarJoint::getConstrainedEquality %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScalarJoint::getPositionState %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJScalarJoint::getVelocityState %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJBallJoint::getPositionState %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJBallJoint::getVelocityState %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJFreeJoint::getTranslationPositionState %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJFreeJoint::getTranslationVelocityState %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJFreeJoint::getAttitudeState %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJFreeJoint::getAttitudeRateState %{
    if val is not None:
        val._swig_bsk_owner = self
%}

%ignore MJJoint::MJJoint;
%ignore MJJoint::stageConfiguration;
%ignore MJJoint::commitConfiguration;
%ignore MJJoint::discardStagedConfiguration;
%ignore MJJoint::registerPositionStates;
%ignore MJJoint::registerVelocityStates;
%ignore MJJoint::setPositionDerivativeFromMujoco;
%ignore MJJoint::clearStateHandles;
%ignore MJJoint::validateStateLayout;

%ignore MJScalarJoint::MJScalarJoint;
%ignore MJScalarJoint::stageConfiguration;
%ignore MJScalarJoint::commitConfiguration;
%ignore MJScalarJoint::discardStagedConfiguration;
%ignore MJScalarJoint::updateConstrainedEquality;
%ignore MJScalarJoint::writeJointStateMessage;


%include "MJJoint.h"



%include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
struct ScalarJointStateMsgPayload_C;

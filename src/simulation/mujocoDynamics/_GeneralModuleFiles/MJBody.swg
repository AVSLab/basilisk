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
   #include "MJBody.h"
   #include "MJScene.h" // needed to support the class extension below
%}

%include "MJObject.swg"
%template_mujoco_object(mjsBody)

// Retain the owner on borrowed results using the generated Python methods.
%pythonappend MJBody::getSite %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJBody::getCenterOfMass %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJBody::getOrigin %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJBody::getScalarJoint %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJBody::getBallJoint %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJBody::getFreeJoint %{
    if val is not None:
        val._swig_bsk_owner = self
%}
%pythonappend MJBody::getScene %{
    if val is not None:
        val._swig_bsk_owner = getattr(self, "_swig_bsk_owner", self)
%}
%pythonappend MJBody::getSpec %{
    if val is not None:
        val._swig_bsk_owner = getattr(self, "_swig_bsk_owner", self)
%}

%ignore MJBody::MJBody;
%ignore MJBody::stageConfiguration;
%ignore MJBody::commitConfiguration;
%ignore MJBody::discardStagedConfiguration;
%ignore MJBody::writeFwdKinematicsMessages;
%ignore MJBody::writeStateDependentOutputMessages;
%ignore MJBody::registerJointPositionStates;
%ignore MJBody::registerJointVelocityStates;
%ignore MJBody::setJointPositionDerivativesFromMujoco;
%ignore MJBody::clearJointStateHandles;
%ignore MJBody::validateJointStateLayout;
%ignore MJBody::updateMujocoModelFromMassProps;
%ignore MJBody::updateMassPropsDerivative;
%ignore MJBody::updateConstrainedEqualityJoints;


%extend MJBody {
MJScene& getScene() { return $self->getSpec().getScene(); }
}

%include "MJBody.h"



%include "architecture/msgPayloadDefC/SCMassPropsMsgPayload.h"
struct SCMassPropsMsgPayload_C;

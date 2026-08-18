/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

%module twoHingeDamper

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "twoHingeDamper.h"
%}

%pythoncode %{
from Basilisk.architecture.swig_common_model import *
%}

%include "std_string.i"

// Keep the dynamics module independent of MuJoCo while providing a concise
// Python-side adapter for the common two-hinge use case.
%extend TwoHingeDamper {
%pythoncode %{
    def applyTo(self, phiJoint, thetaJoint):
        """
        Connect this damper to two MuJoCo hinge joints.

        The first joint is the pendulum's ``phi`` coordinate and the second is
        its ``theta`` coordinate. Both joints must belong to the same scene.
        The helper connects their state messages, creates one torque actuator
        per joint, and routes the generalized damping torques to those
        actuators. The caller remains responsible for adding the damper to the
        scene's dynamics task.

        Parameters
        ----------
        phiJoint : MJScalarJoint
            First pendulum scalar joint. For this damping law it must be a hinge.
        thetaJoint : MJScalarJoint
            Second pendulum scalar joint. For this damping law it must be a hinge.

        Returns
        -------
        tuple[MJSingleActuator, MJSingleActuator]
            The actuators for the ``phi`` and ``theta`` joints.
        """
        try:
            from Basilisk.simulation import mujoco
        except ImportError:
            raise RuntimeError(
                "TwoHingeDamper.applyTo requires a Basilisk build with MuJoCo enabled."
            )

        for name, joint in (("phiJoint", phiJoint), ("thetaJoint", thetaJoint)):
            if not isinstance(joint, mujoco.MJScalarJoint):
                raise TypeError(
                    f"TwoHingeDamper.applyTo: {name} must be an MJScalarJoint; "
                    f"got {type(joint)!r}"
                )
            if not joint.isHinge():
                raise ValueError(
                    f"TwoHingeDamper.applyTo: {name} must be a hinge joint."
                )

        phiScene = phiJoint.getBody().getScene()
        thetaScene = thetaJoint.getBody().getScene()
        if phiScene.this != thetaScene.this:
            raise ValueError(
                "TwoHingeDamper.applyTo: both joints must belong to the same scene."
            )

        self.thetaInMsg.subscribeTo(thetaJoint.stateOutMsg)
        self.phiDotInMsg.subscribeTo(phiJoint.stateDotOutMsg)
        self.thetaDotInMsg.subscribeTo(thetaJoint.stateDotOutMsg)

        tag = self.ModelTag or "twoHingeDamper"
        phiActuator = phiScene.addJointSingleActuator(
            tag + "_" + phiJoint.getName(), phiJoint
        )
        thetaActuator = phiScene.addJointSingleActuator(
            tag + "_" + thetaJoint.getName(), thetaJoint
        )
        phiActuator.actuatorInMsg.subscribeTo(self.phiTorqueOutMsg)
        thetaActuator.actuatorInMsg.subscribeTo(self.thetaTorqueOutMsg)
        return phiActuator, thetaActuator
%}
}

%include "sys_model.i"
%include "twoHingeDamper.h"

%include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
struct ScalarJointStateMsgPayload_C;
%include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
struct SingleActuatorMsgPayload_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

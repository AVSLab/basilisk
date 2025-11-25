/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

%module cannonballDrag

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();

%{
    #include "cannonballDrag.h"
%}

%pythoncode %{
    from Basilisk.architecture.swig_common_model import *
%}
%include "std_string.i"
%include "swig_conly_data.i"

// We add these helper functions on the Python side so that the module
// itself doesn't depend on any MuJoCo imports.
%extend CannonballDrag {
%pythoncode %{
    def applyTo(self, target):
        """
        Bind this CannonballDrag model to a MuJoCo object and route the computed
        aerodynamic wrench to the simulation.

        Parameters
        ----------
        target : MJBody | MJSite | MJForceTorqueActuator
            The MuJoCo object to which the drag model should be attached:

            - MJBody:
            The drag force and torque are applied at the body's center of mass.
            Internally, this resolves to the body's center-of-mass site.

            - MJSite:
            The drag force is applied at the given site. A new
            ``MJForceTorqueActuator`` is created in the owning scene and connected
            to this model.

            - MJForceTorqueActuator:
            The model outputs are connected to an existing actuator, allowing
            advanced users to control actuator placement, naming, or reuse.

        Returns
        -------
        MJForceTorqueActuator
            The force-torque actuator that receives the drag wrench. This is either
            the newly created actuator (for ``MJBody`` or ``MJSite`` targets) or the
            same actuator instance passed in.

        Notes
        -----
        Required message connections depend on the chosen target:

        - MJBody or MJSite:
        ``dragGeometryInMsg`` and ``atmoDensInMsg`` must be connected when calling
        this method.

        - MJForceTorqueActuator:
        ``dragGeometryInMsg``, ``atmoDensInMsg``, and ``referenceFrameStateInMsg``
        must be connected when calling this method.


        Examples
        --------
        Apply drag at a body's center of mass::

            drag = cannonballDrag.CannonballDrag()
            drag.applyTo(body)

        Apply drag at a specific site::

            drag.applyTo(site)

        Reuse an existing actuator::

            act = scene.addForceTorqueActuator('dragAct', site)
            drag.applyTo(act)
        """
        try:
            from Basilisk.simulation import mujoco
        except ImportError:
            raise RuntimeError("This method can only be called when Basilisk is compiled with MuJoCo enabled.")

        # Dispatch and implement the equivalent logic you showed, but in Python
        if isinstance(target, mujoco.MJBody):
            site = target.getCenterOfMass()
            return self.applyTo(site)

        if isinstance(target, mujoco.MJSite):
            site = target

            self.referenceFrameStateInMsg.subscribeTo(site.stateOutMsg)

            scene = site.getBody().getScene()
            actuator = scene.addForceTorqueActuator(
                self.ModelTag + "_cannonballDrag_" + site.getName(),
                site
            )

            actuator.forceInMsg.subscribeTo(self.forceOutMsg)
            actuator.torqueInMsg.subscribeTo(self.torqueOutMsg)
            return actuator

        if isinstance(target, mujoco.MJForceTorqueActuator):
            actuator = target
            actuator.forceInMsg.subscribeTo(self.forceOutMsg)
            actuator.torqueInMsg.subscribeTo(self.torqueOutMsg)
            return actuator

        raise TypeError(
            "CannonballDrag.applyTo(target): target must be MJBody, MJSite, or MJForceTorqueActuator; "
            f"got {type(target)!r}"
        )
%}
}

%include "sys_model.i"
%include "cannonballDrag.h"

%include "architecture/msgPayloadDefC/DragGeometryMsgPayload.h"
struct DragGeometryMsg_C;
%include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"
struct AtmoPropsMsg_C;
%include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
struct SCStatesMsg_C;
%include "architecture/msgPayloadDefC/ForceAtSiteMsgPayload.h"
struct ForceAtSiteMsg_C;
%include "architecture/msgPayloadDefC/TorqueAtSiteMsgPayload.h"
struct TorqueAtSiteMsg_C;

%pythoncode %{
import sys
protectAllClasses(sys.modules[__name__])
%}

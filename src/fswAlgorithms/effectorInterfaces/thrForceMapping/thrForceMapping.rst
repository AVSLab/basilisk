Executive Summary
-----------------

This module is reads in a desired attitude control torque vector and maps it onto a set of thrusters.

The module works for both on-pulsing (nominal thruster state is off such as with RCS thrusters) and off-pulsing (nominal thruster state in such as with DV thrusters). More information can be found in the
:download:`PDF Description </../../src/fswAlgorithms/effectorInterfaces/thrForceMapping/_Documentation/Basilisk-ThrusterForces-20160627.pdf>`.

Message Connection Descriptions
-------------------------------
The following diagram and table list all the module input and output messages.  The module message connection is
set by the user from Python.  The message type contains a link to the message structure definition, while the
description provides information on what this message is used for.

.. bsk-module-io:: thrForceMapping
    :caption: Module I/O Messages

    output thrForceCmdOutMsg THRArrayCmdForceMsgPayload
        thruster force output message
    input cmdTorqueInMsg CmdTorqueBodyMsgPayload
        commanded attitude control torque vector input message
    input thrConfigInMsg THRArrayConfigMsgPayload
        Thruster array configuration input message
    input vehConfigInMsg VehicleConfigMsgPayload
        spacecraft configuration input message

Module Assumptions and Limitations
----------------------------------
The variable ``controlAxes_B`` holds up to three orthonormal control axes :math:`\hat{\mathbf{c}}_j`, stored row by
row in body frame components as the matrix :math:`[C]`.  Unused rows are zero-padded.  ``Reset()`` normalizes each
axis, and logs a ``BSK_WARNING`` if any two axes are not orthogonal, as the math below then no longer holds.

Let :math:`{}^{\cal B}\mathbf{L}_{\mathrm{cmd}}` be the input torque request and
:math:`\mathbf{F}_{\mathrm{nom}}` the nominal thruster forces.  For on-pulsing (``thrForceSign = +1``),
:math:`\mathbf{F}_{\mathrm{nom}}` is zero and the commands :math:`\mathbf{F}` are nonnegative forces.
For off-pulsing (``thrForceSign = -1``), :math:`\mathbf{F}_{\mathrm{nom}} = \mathbf{F}_{\max}` and
:math:`\mathbf{F}` represents nonpositive changes from those nominal forces: a zero command leaves a
thruster fully on, while :math:`F_i = -F_{\max,i}` requests that it remain off for the control period.
The allocator subtracts the nominal thruster torque before projecting the request:

.. math::

    {}^{\cal B}\mathbf{L}_r = {}^{\cal B}\mathbf{L}_{\mathrm{cmd}} - [D]\mathbf{F}_{\mathrm{nom}}.

This adjusted target is reduced onto the control axes, written in body frame components, as

.. math::

    {}^{\cal B}\bar{\mathbf{L}}_r = [C]^T [C] \, {}^{\cal B}\mathbf{L}_r

The thruster forces :math:`\mathbf{F}` are then found such that :math:`[C][D]\mathbf{F} = [C]\,{}^{\cal B}\mathbf{L}_r`.

The magnitudes of the commanded force changes :math:`\mathbf{F}` can exceed the individual thruster limits
:math:`F_{\max,i}`.  The angle calculation uses the saturation-clipped commands
:math:`\mathbf{F}_{\mathrm{clip}}` and their torque contribution:

.. math::

    F_{\mathrm{clip},i} = \min\left(F_{\max,i}, \max\left(-F_{\max,i}, F_i\right)\right),
    \qquad {}^{\cal B}\boldsymbol{\tau} = [D]\mathbf{F}_{\mathrm{clip}}.

The corresponding ideal average net thruster torque is
:math:`{}^{\cal B}\boldsymbol{\tau}_{\mathrm{net}} = [D](\mathbf{F}_{\mathrm{nom}} + \mathbf{F}_{\mathrm{clip}})`.
In off-pulsing mode, :math:`\boldsymbol{\tau}` is the change from nominal torque, and differs from
:math:`\boldsymbol{\tau}_{\mathrm{net}}` when the nominal thrust produces a nonzero torque.

The output ``outTorqAngErr`` is the angle between this torque contribution and the reduced adjusted target
:math:`{}^{\cal B}\bar{\mathbf{L}}_r`, both in body frame components:

.. math::

    \theta = \arccos \left( \frac{\boldsymbol{\tau} \cdot \bar{\mathbf{L}}_r}{|\boldsymbol{\tau}| \, |\bar{\mathbf{L}}_r|} \right)

This angle is invariant under reordering the control axes or choosing another orthonormal basis
for the same controlled subspace.  Changing the controlled subspace can change the angle and force solution.

If this angle is larger than ``angErrThresh`` [rad] and any commanded force exceeds its limit,
all commanded forces are scaled by the same positive factor to satisfy every thruster limit.
The resulting torque contribution preserves the direction of the unscaled contribution :math:`[D]\mathbf{F}`.
It aligns with :math:`\bar{\mathbf{L}}_r` only if that unscaled torque was already aligned; residual torque
outside the controlled subspace is also scaled and can leave a nonzero angle error.
In off-pulsing mode, these direction statements concern the change from nominal torque; the net torque
also includes the unchanged nominal contribution :math:`[D]\mathbf{F}_{\max}`.
The module recomputes ``outTorqAngErr`` after scaling.  If the angle is at or below the threshold,
the output force commands can remain above their limits even though the angle calculation uses clipped forces.
The default value of ``angErrThresh`` is 0 rad.  Setting it to a value larger than :math:`\pi` rad disables scaling.

Thruster locations, force directions, and maximum thrust values are read from ``thrConfigInMsg`` during
``Reset()`` and remain cached until the next reset.  The spacecraft center of mass is read from
``vehConfigInMsg`` on every update, and the module recomputes the thruster moment arms using the current
``CoM_B``.  Center-of-mass changes therefore do not require a reset.

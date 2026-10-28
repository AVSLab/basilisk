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
row in body frame components as the matrix :math:`[C]`.  Unused rows are zero-padded.  The commanded torque
:math:`{}^{\cal B}\mathbf{L}_r` is reduced onto these control axes, written in body frame components, as

.. math::

    {}^{\cal B}\bar{\mathbf{L}}_r = [C]^T [C] \, {}^{\cal B}\mathbf{L}_r

The thruster forces :math:`\mathbf{F}` are then found such that :math:`[C][D]\mathbf{F} = [C]\,{}^{\cal B}\mathbf{L}_r`.

The output ``outTorqAngErr`` is the angle between the reduced torque :math:`{}^{\cal B}\bar{\mathbf{L}}_r` and the
produced thruster torque :math:`{}^{\cal B}\boldsymbol{\tau} = [D]\mathbf{F}`, both in body frame components:

.. math::

    \theta = \arccos \left( \frac{\boldsymbol{\tau} \cdot \bar{\mathbf{L}}_r}{|\boldsymbol{\tau}| \, |\bar{\mathbf{L}}_r|} \right)

This angle does not depend on the order or orientation of the control axes in ``controlAxes_B``.

If this angle is larger than ``angErrThresh`` [rad], then the thruster forces are scaled such that no thruster
exceeds its maximum thrust.  This reduces the torque magnitude, but keeps the torque direction aligned with
:math:`\bar{\mathbf{L}}_r`.  The default value of ``angErrThresh`` is 0, i.e. this scaling is on by default.  Setting
``angErrThresh`` to a value larger than 180 deg disables the scaling.

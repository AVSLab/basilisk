
Executive Summary
-----------------

This class is an instantiation of the stateEffector class and is a `N`-hinged rigid body effector. This effector is a rigid body attached to the hub through a torsional spring and damper that approximates a flexible appendage. See Allard, Schaub, and Piggott paper: `General Hinged Solar Panel Dynamics Approximating First-Order Spacecraft Flexing <http://dx.doi.org/10.2514/1.A34125>`__ for a detailed description of this model. A hinged rigid body has 2 states: theta and thetaDot

The module
:download:`PDF Description </../../src/simulation/dynamics/NHingedRigidBodies/_Documentation/Basilisk-NHINGEDRIGIDBODYSTATEEFFECTOR-20180103.pdf>`
contains further information on this module's function,
how to run it, as well as testing.

.. important::

    The equations of motion are derived for a chain of identical panels. Every panel must carry the
    same positive ``mass`` and the same hinge to center of mass distance ``d``, and each hinge sits
    ``2d`` from the one before it. The panel inertia ``IPntS_S`` may differ from panel to panel.
    Initialization rejects a chain that violates the mass or distance requirement. An uneven chain
    is modeled with :ref:`dualHingedRigidBodyStateEffector` for two panels, or with
    :ref:`spinningBodyNDOFStateEffector` for an arbitrary number.



Message Connection Descriptions
-------------------------------
The following table lists all the module output messages. Each is a vector carrying one message per
panel, ordered outward from the hub.

.. bsk-module-io:: nHingedRigidBodyStateEffector
    :caption: Module I/O Messages

    output nHingedRigidBodyOutMsgs HingedRigidBodyMsgPayload
        Output vector of messages containing the panel angle and angle rate.
    output nHingedRigidBodyConfigLogOutMsgs SCStatesMsgPayload
        Output vector of messages containing the panel inertial states. The position and velocity are
        those of the panel center of mass, and the attitude and angular velocity are those of the
        panel frame S.

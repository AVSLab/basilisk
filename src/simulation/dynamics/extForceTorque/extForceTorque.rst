Executive Summary
-----------------

Class used to provide a direct external force and torque on body. This class is used to simulate an external for or torque acting on the body.
For example, this module can be used to simulate the external disturbance due to
outgasing or a thruster, or be used to directly apply requested control forces or
torques.

The module
:download:`PDF Description </../../src/simulation/dynamics/extForceTorque/_Documentation/Basilisk-extForceTorque-20161103.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: extForceTorque
    :caption: Module I/O Messages

    input cmdTorqueInMsg CmdTorqueBodyMsgPayload
        commanded torque input msg.
    input cmdForceBodyInMsg CmdForceBodyMsgPayload
        commanded force input msg in B frame.
    input cmdForceInertialInMsg CmdForceInertialMsgPayload
        commanded force input msg in N frame.


Attaching to a State Effector
-----------------------------
This effector supports the branching described in :ref:`bskPrinciples-11`, so its load can be
carried by an appendage instead of the hub. Attach it to the parent state effector rather than
to the spacecraft::

    stateEff.addDynamicEffector(extFTObject, segment)

This effector reads no kinematics of its own, so the body frame force and torque it is given are
interpreted in the parent segment's frame and the torque is applied about that segment's frame
origin. An inertial frame force is unaffected by the choice of parent. The ``segment`` argument is
omitted for a parent with a single degree of freedom.

Both the parent and the child are still added to the task in the usual way, the same as when the
child is attached to the hub.

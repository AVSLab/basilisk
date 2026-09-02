
Executive Summary
-----------------

Class to represent a solar array of two panels. The first panel is hinged on a single axis to the spacecraft body. The second panel is hinged to the first panel by a parallel axis on the opposite end of the first panel from the spacecraft body.)


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: dualHingedRigidBodyStateEffector
    :caption: Module I/O Messages

    input motorTorqueInMsg ArrayMotorTorqueMsgPayload
        (Optional) Input message of the two hinge motor torque values.
    output dualHingedRigidBodyOutMsgs HingedRigidBodyMsgPayload
        vector of output message containing the panel 1 and 2 hinge state angle and angle rate.
    output dualHingedRigidBodyConfigLogOutMsgs SCStatesMsgPayload
        vector of output messages containing the panel 1 and 2 inertial position and attitude states.


Detailed Module Description
---------------------------

The module :download:`PDF Module Description </../../src/simulation/dynamics/dualHingedRigidBodies/_Documentation/Basilisk-DUALHINGEDRIGIDBODYSTATEEFFECTOR-20180102.pdf>` contains further information on this module's function, as well as testing.


User Guide
----------
This section is to outline the steps needed to setup a Hinged Rigid Body State Effector in python using Basilisk.

#. Import the dualHingedRigidBodyStateEffector class::

    from Basilisk.simulation import dualHingedRigidBodyStateEffector

#. Create an instantiation of a Dual Hinged Rigid body::

    panel1 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()

#. Define all physical parameters for a Dual Hinged Rigid Body. For example::

    IPntS1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]

   Do this for all of the parameters for a Dual Hinged Rigid Body seen in the public variables in the .h file.

#. Define the initial conditions of the states::

    panel1.theta1Init = 5*numpy.pi/180.0
    panel1.theta1DotInit = 0.0
    panel1.theta2Init = 5*numpy.pi/180.0
    panel1.theta2DotInit = 0.0

#. (Optional) Define a unique name for each state::

    panel1.nameOfTheta1State = "dualHingedRigidBodyTheta1"
    panel1.nameOfTheta1DotState = "dualHingedRigidBodyThetaDot1"
    panel1.nameOfTheta2State = "dualHingedRigidBodyTheta2"
    panel1.nameOfTheta2DotState = "dualHingedRigidBodyThetaDot2"

#. Define an optional motor torque input message with ``panel1.motorTorqueInMsg``

#. The module creates two output messages with the panel angular states.
   The messages are stored in the vector ``dualHingedRigidBodyOutMsgs``.

#. The module creates two output messages with each panel inertial position and attitude states.
   The messages are stored in the vector ``dualHingedRigidBodyConfigLogOutMsgs``.

#. Add the panel to your spacecraft::

    scObject.addStateEffector(panel1)

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, panel1)


Hosting a Dynamic Effector
--------------------------
This effector supports the branching described in :ref:`bskPrinciples-11`, so a compatible
dynamic effector can be carried by one of the panels rather than by the hub::

    panel1.addDynamicEffector(childEffector, segment)

Here ``segment`` is the one-based panel number, counting outward from the hub, so ``1`` is the
panel attached to the hub.

The child then reads this effector's inertial position, velocity, attitude, and angular velocity
in place of the hub's, and any geometry given to the child is expressed in that panel's frame
rather than the hub body frame. Both this effector and the child are still added to the task in
the usual way.

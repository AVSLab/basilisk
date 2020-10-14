
Executive Summary
-----------------

Class to represent a solar array of two panels. The first panel is hinged on a single axis to the spacecraft body. The second panel is hinged to the first panel by a parallel axis on the opposite end of the first panel from the spacecraft body.)


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. _ModuleIO_DualHingedRigidBodyStateEffector:
.. figure:: /../../src/simulation/dynamics/dualHingedRigidBodies/_Documentation/Images/moduleIODualHingedRigidBodyStateEffector.svg
    :align: center

    Figure 1: ``DualHingedRigidBodyStateEffector()`` Module I/O Illustration

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - motorTorqueInMsgName
      - :ref:`ArrayMotorTorqueIntMsg`
      - (Optional) Input message of the two hinge motor torque values
    * - dualHingedRigidBodyOutMsgName + ``_OutputStates0``
      - :ref:`HingedRigidBodySimMsg`
      - Output message containing the panel 1 hinge state angle and angle rate
    * - dualHingedRigidBodyOutMsgName + ``_OutputStates1``
      - :ref:`HingedRigidBodySimMsg`
      - Output message containing the panel 2 hinge state angle and angle rate
    * - dualHingedRigidBodyConfigLogOutMsgName + ``_InertialStates0``
      - :ref:`SCPlusStatesSimMsg`
      - Output message containing the panel 1 inertial position and attitude states
    * - dualHingedRigidBodyConfigLogOutMsgName + ``_InertialStates1``
      - :ref:`SCPlusStatesSimMsg`
      - Output message containing the panel 2 inertial position and attitude states


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

#. Define a unique name for each state::

    panel1.nameOfTheta1State = "dualHingedRigidBodyTheta1"
    panel1.nameOfTheta1DotState = "dualHingedRigidBodyThetaDot1"
    panel1.nameOfTheta2State = "dualHingedRigidBodyTheta2"
    panel1.nameOfTheta2DotState = "dualHingedRigidBodyThetaDot2"

#. Define an optional motor torque input message::

    panel1.motorTorqueInMsgName = "motorTorque"

#. The module creates two output messages with the panel angular states.
   The message name is composed of the string ``dualHingedRigidBodyOutMsgName`` and appended with
   either ``_OutputStates0`` or ``_OutputStates1``.  If this base string is not set, then the module
   ``ModuleTag`` string is used as the base string.  An example of setting the base msg string is::

    panel1.dualHingedRigidBodyOutMsgName = "panel1Angles"

#. The module creates two output messages with each panel inertial position and attitude states.
   The message name is composed of the string ``dualHingedRigidBodyConfigLogOutMsgName`` and appended with
   either ``_InertialStates0`` or ``_InertialStates1``.  If this base string is not set, then the module
   ``ModuleTag`` string is used as the base string.  An example of setting the base msg string is::

    panel1.dualHingedRigidBodyConfigLogOutMsgName = "panel1Log"

#. Add the panel to your spacecraftPlus::

    scObject.addStateEffector(panel1)

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, panel1)




Executive Summary
-----------------

The two-degree-of-freedom spinning body class is an instantiation of the state effector abstract class. The integrated test is validating the interaction between the 2-DoF spinning body module and the rigid body hub that it is attached to. In this case, a 2-DoF spinning body system has two masses and two inertia tensors. The lower axis is attached to the hub and the upper axis is attached to the lower body. This module can represent multiple different effectors, such as a dual-hinged solar array, a control moment gyroscope or a dual-gimbal. A spring and damper can be included in each axis, and an optional motor torque can be applied on each spinning axis.

Nominally, two rigid bodies can be defined, which would represent a chain of 1D rotary joints. However, by setting the mass and the inertia of the lower spinning body to 0, the module can simulate a single rigid component rotating about two axis instead.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - spinningBodyOutMsgs
      - :ref:`HingedRigidBodyMsgPayload`
      - Output vector of messages containing the spinning body state angle and angle rate
    * - motorTorqueInMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - (Optional) Input message of the motor torque value
    * - motorLockInMsg
      - :ref:`ArrayEffectorLockMsgPayload`
      - (Optional) Input message for locking the axis
    * - spinningBodyRefInMsgs
      - :ref:`HingedRigidBodyMsgPayload`
      - (Optional) Input array of messages for prescribing the angles and angle rates
    * - spinningBodyConfigLogOutMsgs
      - :ref:`SCStatesMsgPayload`
      - Output vector of messages containing the spinning body inertial position and attitude states


Detailed Module Description
---------------------------

A 2 DoF spinning body has 4 states: ``theta1``, ``theta2``, ``theta1Dot`` and ``theta2Dot``.

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
See the following conference paper for a detailed description of this model.

.. note::

    J. Vaz Carneiro, C. Allard and H. Schaub, `"Rotating Rigid Body Dynamics
    Architecture for Spacecraft Simulation Software Implementation" <https://hanspeterschaub.info/Papers/VazCarneiro2023.pdf>`_,
    AAS Rocky Mountain GN&C Conference, Breckenridge, CO, Feb. 2–8, 2023

User Guide
----------
This section is to outline the steps needed to setup a Spinning Body 2 DoF State Effector in Python using Basilisk.

#. Import the spinningBody2DOFStateEffector class::

    from Basilisk.simulation import spinningBody2DOFStateEffector

#. Create an instantiation of a Spinning body::

    spinningBody = spinningBody2DOFStateEffector.SpinningBody2DOFStateEffector()

#. Define all physical parameters for both spinning bodies. For example::

    spinningBody.mass1 = 100.0
    spinningBody.mass2 = 50.0
    spinningBody.IS1PntSc1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    spinningBody.IS2PntSc2_S2 = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    spinningBody.dcm_S10B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.dcm_S20S1 = [[0.0, -1.0, 0.0], [0.0, .0, -1.0], [1.0, 0.0, 0.0]]
    spinningBody.r_Sc1S1_S1 = [[2.0], [-0.5], [0.0]]
    spinningBody.r_Sc2S2_S2 = [[1.0], [0.0], [-1.0]]
    spinningBody.r_S1B_B = [[-2.0], [0.5], [-1.0]]
    spinningBody.r_S2S1_S1 = [[0.5], [-1.5], [-0.5]]
    spinningBody.s1Hat_S1 = [[0], [0], [1]]
    spinningBody.s2Hat_S2 = [[0], [-1], [0]]

#. (Optional) Define initial conditions of the effector.  Default values are zero states::

    spinningBody.theta1Init = 0 * macros.D2R
    spinningBody.theta1DotInit = 0.1 * macros.D2R
    spinningBody.theta2Init = 5 * macros.D2R
    spinningBody.theta2DotInit = -0.5 * macros.D2R

#. (Optional) Define spring and damper coefficients.  Default values are zero states::

    spinningBody.k1 = 1.0
    spinningBody.c1 = 0.1
    spinningBody.k2 = 2.0
    spinningBody.c2 = 0.5

#. (Optional) Define a unique name for each state.  If you have multiple spinning bodies, they each must have a unique name.  If these names are not specified, then the default names are used which are incremented by the effector number::

    spinningBody.nameOfTheta1State = "spinningBodyTheta1"
    spinningBody.nameOfTheta1DotState = "spinningBodyTheta1Dot"
    spinningBody.nameOfTheta2State = "spinningBodyTheta2"
    spinningBody.nameOfTheta2DotState = "spinningBodyTheta2Dot"

#. (Optional) Connect a command torque message::

    cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    cmdArray.motorTorque = [cmdTorque1, cmdTorque2]  # [Nm]
    cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    spinningBody.motorTorqueInMsg.subscribeTo(cmdMsg)

#. (Optional) Connect an axis-locking message (0 means the axis is free to rotate and 1 locks the axis)::

    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.motorTorque = [1, 0]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    spinningBody.motorLockInMsg.subscribeTo(lockMsg)

#. (Optional) Connect angle and angle rate reference messages::

    angle1Ref = messaging.HingedRigidBodyMsgPayload()
    angle1Ref.theta = theta1Ref
    angle1Ref.thetaDot = theta1DotRef
    angle1RefMsg = messaging.HingedRigidBodyMsg().write(angle1Ref)
    spinningBody.spinningBodyRefInMsgs[0].subscribeTo(angle1RefMsg)

    angle2Ref = messaging.HingedRigidBodyMsgPayload()
    angle2Ref.theta = theta2Ref
    angle2Ref.thetaDot = theta2DotRef
    angle2RefMsg = messaging.HingedRigidBodyMsg().write(angle2Ref)
    spinningBody.spinningBodyRefInMsgs[1].subscribeTo(angle2RefMsg)

#. The angular states of the body are created using an output vector of messages ``spinningBodyOutMsgs``.

#. The spinning body config log state output messages is ``spinningBodyConfigLogOutMsgs``.

#. Add the effector to your spacecraft::

    scObject.addStateEffector(spinningBody)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, spinningBody)

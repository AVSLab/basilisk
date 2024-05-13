
Executive Summary
-----------------

The N-degree-of-freedom spinning body class is an instantiation of the state effector abstract class. The integrated test is validating the interaction between the N-DoF spinning body module and the rigid body hub that it is attached to. In this case, an N-DoF spinning body system has four masses and four inertia tensors. The lower axis is attached to the hub and the upper axis is attached to the lower body. This module can represent multiple different effectors, with any number of degrees of freedom. A spring and damper can be included in each axis, and an optional motor torque can be applied on each spinning axis.

Nominally, each degree of freedom corresponds to an additional rigid body link. However, by setting the mass and the inertia of to 0, the module can simulate multiple degrees of freedom for the same spinning body instead.


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
    * - spinningBodyConfigLogOutMsgs
      - :ref:`SCStatesMsgPayload`
      - Output vector of messages containing the spinning body inertial position and attitude states


Detailed Module Description
---------------------------

An N-DoF spinning body has 2N states: ``theta``, ``thetaDot`` for each degree of freedom.

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
See the following conference paper for a detailed description of this model.

.. note::

    J. Vaz Carneiro, C. Allard and H. Schaub, `"Effector Dynamics For Sequentially Rotating Rigid Body Spacecraft Components" <https://hanspeterschaub.info/Papers/VazCarneiro2023a.pdf>`_,
    AAS Astrodynamics Specialist Conference, Bog Sky, MT, Aug. 13-17, 2023

User Guide
----------
This section is to outline the steps needed to setup a Spinning Body N DoF State Effector in Python using Basilisk.

#. Import the spinningBodyNDOFStateEffector class::

    from Basilisk.simulation import spinningBodyNDOFStateEffector

#. Create an instantiation of a Spinning body::

    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()

#. Define all physical parameters for each spinning body. For example::

    spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody.setMass(25.0)
    spinningBody.setISPntSc_S([[50, 0.0, 0.0],
                                [0.0, 40, 0.0],
                                [0.0, 0.0, 30]])
    spinningBody.setDCM_S0P([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    spinningBody.setR_ScS_S([[0.2],
                              [-0.3],
                              [0.1]])
    spinningBody.setR_SP_P([[-0.05],
                             [0.0],
                             [0.1]])
    spinningBody.setSHat_S([[0], [0], [1]])
    spinningBodyEffector.addSpinningBody(spinningBody)

#. (Optional) Define initial conditions of the effector.  Default values are zero states::

    spinningBody.setThetaInit(10.0 * macros.D2R)
    spinningBody.setThetaDotInit(-1.0 * macros.D2R)

#. (Optional) Define spring and damper coefficients.  Default values are zero states::

    spinningBody.setK(100)
    spinningBody.setC(20)

#. (Optional) Define a unique name for each state.  If you have multiple spinning bodies, they each must have a unique name.  If these names are not specified, then the default names are used which are incremented by the effector number::

    spinningBodyEffector.setNameOfThetaState = "spinningBodyTheta"
    spinningBodyEffector.setNameOfThetaDotState = "spinningBodyThetaDot"

#. (Optional) Connect a command torque message::

    cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    cmdArray.motorTorque = [cmdTorque]  # [Nm]
    cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    spinningBody.motorTorqueInMsg.subscribeTo(cmdMsg)

#. (Optional) Connect an axis-locking message (0 means the axis is free to rotate and 1 locks the axis)::

    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.motorTorque = [1]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    spinningBody.motorLockInMsg.subscribeTo(lockMsg)

#. The angular states of the body are created using an output vector of messages ``spinningBodyOutMsgs``.

#. The spinning body config log state output messages is ``spinningBodyConfigLogOutMsgs``.

#. Add the effector to your spacecraft::

    scObject.addStateEffector(spinningBody)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, spinningBody)

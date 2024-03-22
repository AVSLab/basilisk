
Executive Summary
-----------------

The linear translation body class is an instantiation of the state effector abstract class. The integrated test is validating the interaction between the linear translation body module and the rigid body hub that it is attached to. In this case, a 1-DoF linear translation body has an inertia tensor and is attached to the hub by a single degree of freedom axis. The spinning axis is fixed in the body frame and the effector is rigid, which means that its center of mass location does not move in the F frame. An optional motor force can be applied on the spinning axis, and the user can also lock the axis through a command.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - translatingBodyOutMsg
      - :ref:`LinearTranslationRigidBodyMsgPayload`
      - Output message containing the linear translation body state displacement and displacement rate
    * - motorForceInMsg
      - :ref:`ArrayMotorForceMsgPayload`
      - (Optional) Input message of the motor force value
    * - motorLockInMsg
      - :ref:`ArrayEffectorLockMsgPayload`
      - (Optional) Input message for locking the axis
    * - translatingBodyRefInMsg
      - :ref:`LinearTranslationRigidBodyMsgPayload`
      - (Optional) Input message for prescribing the displacement and displacement rate
    * - translatingBodyConfigLogOutMsg
      - :ref:`SCStatesMsgPayload`
      - Output message containing the translating body inertial position and attitude states


Detailed Module Description
---------------------------

A 1 DoF translating body has 2 states: ``rho`` and ``rhoDot``. The displacemet and displacement rate can change due to the interaction with the hub, but also because of applied forces (control, spring and damper). The displacement remains fixed and the displacement rate is set to zero when the axis is locked.

Mathematical Modeling
^^^^^^^^^^^^^^^^^^^^^
See the following tech report for a detailed description of this model.

.. note::

    P. Johnson and J. Vaz Carneiro, "`Single Axis Translating Effector <http://hanspeterschaub.info/Papers/singleAxisTranslatingEffector.pdf>`_,"
    Technical Note, University of Colorado, Autonomous Vehicle Systems (AVS) Lab, Boulder, CO, March 9, 2024.

User Guide
----------
This section is to outline the steps needed to setup a Translating Body State Effector in Python using Basilisk.

#. Import the linearTranslatingBodyOneDOFStateEffector class::

    from Basilisk.simulation import linearTranslatingBodyOneDOFStateEffector

#. Create an instantiation of a Translating body::

    translatingBody = linearTranslatingBodyOneDOFStateEffector.linearTranslatingBodyOneDOFStateEffector()

#. Define all physical parameters for a Translating Body. For example::

    translatingBody.setMass(20.0)
    translatingBody.setFHat_B([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody.setR_FcF_F([[-1.0], [1.0], [0.0]])
    translatingBody.setR_F0B_B([[-5.0], [4.0], [3.0]])
    translatingBody.setIPntFc_F([[50.0, 0.0, 0.0],
                                 [0.0, 80.0, 0.0],
                                 [0.0, 0.0, 60.0]])
    translatingBody.setDCM_FB([[0.0, -1.0, 0.0],
                               [0.0, 0.0, -1.0],
                               [1.0, 0.0, 0.0]])

#. (Optional) Define initial conditions of the effector.  Default values are zero states::

    translatingBody.setRhoInit(1.0)
    translatingBody.setRhoDotInit(0.05)

#. (Optional) Define spring and damper coefficients.  Default values are zero::

    translatingBody.setK(100.0)
    translatingBody.setC(0.0)

#. (Optional) Define a unique name for each state.  If you have multiple translating bodies, they each must have a unique name.  If these names are not specified, then the default names are used which are incremented by the effector number::

    translatingBody.nameOfThetaState = "translatingBodyRho"
    translatingBody.nameOfThetaDotState = "translatingBodyRhoDot"

#. (Optional) Connect a command force message::

    cmdArray = messaging.ArrayMotorForceMsgPayload()
    cmdArray.motorForce = [cmdForce]  # [Nm]
    cmdMsg = messaging.ArrayMotorForceMsg().write(cmdArray)
    translatingBody.motorForceInMsg.subscribeTo(cmdMsg)

#. (Optional) Connect an axis-locking message (0 means the axis is free to move and 1 locks the axis)::

    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.effectorLockFlag = [1]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    translatingBody.motorLockInMsg.subscribeTo(lockMsg)

#. (Optional) Connect a displacement and displacement rate reference message::

    translationRef = messaging.LinearTranslationRigidBodyMsgPayload()
    translationRef.rho = 0.2
    translationRef.rhoDot = 0.0
    translationRefMsg = messaging.LinearTranslationRigidBodyMsg().write(translationRef)
    translatingBody.translatingBodyRefInMsg.subscribeTo(translationRefMsg)

#. The linear states of the body are created using an output message ``translatingBodyOutMsg``.

#. The translating body config log state output message is ``translatingBodyConfigLogOutMsg``.

#. Add the effector to your spacecraft::

    scObject.addStateEffector(translatingBody)

   See :ref:`spacecraft` documentation on how to set up a spacecraft object.

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, translatingBody)


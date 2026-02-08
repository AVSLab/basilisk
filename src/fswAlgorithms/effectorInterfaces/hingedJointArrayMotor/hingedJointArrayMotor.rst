Executive Summary
-----------------
The ``hingedJointArrayMotor`` module determines the motor torques for an array of hinged joints based on commanded angles and the current system configuration.

.. note::
  This module is designed to work for systems with multiple spacecraft, however, each spacecraft must be comprised of six degree-of-freedom rigid hub with attached hinged joints only. Additionally, the math assumes the reaction torques experienced by the hub due to the hinged joint motion are cancelled.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - massMatrixInMsg
      - :ref:`MJSysMassMatrixMsgPayload`
      - System mass matrix input msg
    * - reactionForcesInMsg
      - :ref:`MJJointReactionsMsgPayload`
      - Joint reaction forces and torques input msg
    * - desJointStatesInMsg
      - :ref:`JointArrayStateMsgPayload`
      - Desired joint states input msg
    * - jointStatesInMsg
      - :ref:`ScalarJointStateMsgPayload`
      - Vector of joint state input msgs
    * - jointStateDotsInMsg
      - :ref:`ScalarJointStateMsgPayload`
      - Vector of joint state derivative input msgs
    * - motorTorquesOutMsg
      - :ref:`SingleActuatorMsgPayload`
      - Vector of joint motor torque output msgs

User Guide
----------
This section is to outline the steps needed to setup the ``hingedJointArrayMotor`` module in Python using Basilisk.

#. Import the hingedJointArrayMotor class::

    from Basilisk.fswAlgorithms import hingedJointArrayMotor

#. Create an instance of hingedJointArrayMotor::

    module = hingedJointArrayMotor.HingedJointArrayMotor()

#. Set the control gains::

    Ktheta = 10.0 * np.eye(numJoints * numSpacecraft)
    Ptheta = 2.0 * np.sqrt(10.0) * np.eye(numJoints * numSpacecraft)
    module.setKtheta(Ktheta.flatten().tolist())
    module.setPtheta(Ptheta.flatten().tolist())

#. (Optional) set the maximum motor torque values::

    uMax = [0.03] * (numJoints * numSpacecraft)
    module.setUMax(uMax)

#. For each hinged joint in the system, add a hinged joint to the module::

    module.addHingedJoint()

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, module)

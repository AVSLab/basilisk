Executive Summary
-----------------
The ``thrJointCompensation`` module determines the motor torques for an array of hinged joints as required to ensure zero acceleration of the joints during thruster firing.

.. note::
  This module is designed to work for systems with multiple spacecraft with multiple thruster arms, however, each spacecraft must be comprised of six degree-of-freedom rigid hub with attached arms comprised of hinged joints and pure force thrusters only.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from Python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - armConfigInMsg
      - :ref:`THRArmConfigMsgPayload`
      - static spacecraft configuration input msg
    * - massMatrixInMsg
      - :ref:`MJSysMassMatrixMsgPayload`
      - system mass matrix input msg
    * - reactionForcesInMsg
      - :ref:`MJJointReactionsMsgPayload`
      - joint reaction forces and torques input msg
    * - jointStatesInMsgs
      - :ref:`ScalarJointStateMsgPayload`
      - vector of joint state input msgs
    * - thrForcesInMsgs
      - :ref:`SingleActuatorMsgPayload`
      - vector of thruster force input msgs
    * - motorTorquesOutMsgs
      - :ref:`SingleActuatorMsgPayload`
      - vector of joint motor torque output msgs

User Guide
----------
This section is to outline the steps needed to set up the ``thrJointCompensation`` module in Python using Basilisk.

#. Import the thrJointCompensation class::

    from Basilisk.fswAlgorithms import thrJointCompensation

#. Create an instance of thrJointCompensation::

    module = thrJointCompensation.ThrJointCompensation()

#. (Optional) set the maximum motor torque values::

    uMax = [1e-6] * (numJoints * numArms * numSpacecraft)
    module.setUMax(uMax)

#. For each hinged joint in the system, add a hinged joint to the module::

    module.addHingedJoint()

#. For each thruster in the system, add a thruster to the module::

    module.addThruster()

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, module)

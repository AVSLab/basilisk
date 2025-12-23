Executive Summary
-----------------
The ``jointMotionCompensator`` module determines the hub torques required to negate the effect of hinged joint motor torques on the spacecraft hub motion.

.. note::
  This module is designed to work for systems with multiple spacecraft, however, each spacecraft must be comprised of six degree-of-freedom rigid hub with attached hinged joints only.

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
      - system mass matrix input msg
    * - reactionForcesInMsg
      - :ref:`MJJointReactionsMsgPayload`
      - joint reaction forces and torques input msg
    * - jointTorqueInMsgs
      - :ref:`SingleActuatorMsgPayload`
      - vector of joint motor torque input msgs
    * - hubTorqueOutMsgs
      - :ref:`SingleActuatorMsgPayload`
      - vector of hub torque output msgs

User Guide
----------
This section is to outline the steps needed to setup the ``jointMotionCompensator`` module in Python using Basilisk.

#. Import the jointMotionCompensator class::

    from Basilisk.fswAlgorithms import jointMotionCompensator

#. Create an instance of jointMotionCompensator::

    module = jointMotionCompensator.JointMotionCompensator()

#. (Optional) Set the maximum hub torque values::

    uMax = [0.03] * (3 * numSpacecraft)
    module.setUMax(uMax)

#. For each spacecraft in the system, add a spacecraft to the module::

    module.addSpacecraft()

#. For each hinged joint in the system, add a hinged joint to the module::

    module.addHingedJoint()

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, module)

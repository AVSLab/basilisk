Executive Summary
-----------------
The ``MJJointReactionForces`` module extracts the reaction forces and torques acting on the joints from the MuJoCo scene. It also indexes the kinematic tree, parent body, type, and starting degree-of-freedom for each joint.

.. note::
  If a free joint is used as part of a kinematic tree, it must be used as the first joint of that tree.

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
    * - reactionForcesOutMsg
      - :ref:`MJJointReactionsMsgPayload`
      - joint reaction forces and torques output msg

User Guide
----------
This section is to outline the steps needed to setup the ``MJJointReactionForces`` module in Python using Basilisk.

#. Import the MJJointReactionForces class::

    from Basilisk.simulation import MJJointReactionForces

#. Enable extra EOM call when building the MuJoCo scene::

    scene.extraEoMCall = True

#. Create an instance of MJJointReactionForces::

    module = MJJointReactionForces.MJJointReactionForces()

#. Set the scene the module is attached to::

    module.scene = scene

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, module)

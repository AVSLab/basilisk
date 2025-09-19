Executive Summary
-----------------
The ``MJSystemCoM`` module extracts the total system CoM position and velocity from a Mujoco scene with a single spacecraft.

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
    * - comStatesOutMsg
      - :ref:`SCStatesMsgPayload`
      - spacecraft CoM states C++ output msg
    * - comStatesOutMsgC
      - :ref:`SCStatesMsgPayload`
      - spacecraft CoM states C output msg

User Guide
----------
This section is to outline the steps needed to setup the ``MJSystemCoM`` module in Python using Basilisk.

#. Import the MJSystemCoM class::

    from Basilisk.simulation import MJSystemCoM

#. Enable extra EOM call when building the Mujoco scene::

    scene.extraEoMCall = True

#. Create an instance of MJSystemCoM::

    module = MJSystemCoM.MJSystemCoM()

#. Set the scene the module is attached to::

    module.scene = scene

#. The MJSystemCoM output message is ``comStatesOutMsg``.

#. Add the module to the Mujoco scene dynamics task::

    scene.AddModelToDynamicsTask(module)

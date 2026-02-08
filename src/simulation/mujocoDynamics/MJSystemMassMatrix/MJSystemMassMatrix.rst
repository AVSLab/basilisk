Executive Summary
-----------------
The ``MJSystemMassMatrix`` module extracts the full system mass matrix. It also stores the number of spacecraft in the scene, indexes their starting joint, and the types of joints used.

.. note::
    This module assumes that each body in the MuJoCo scene that is a direct child of the world body is a separate spacecraft so long as it has at least one joint.
    If a free joint is used as part of a spacecraft, it must be used as the first joint of that spacecraft.

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
    * - massMatrixOutMsg
      - :ref:`MJSysMassMatrixMsgPayload`
      - system mass matrix C ++ output msg in generalized coordinates

User Guide
----------
This section is to outline the steps needed to setup the ``MJSystemMassMatrix`` module in Python using Basilisk.

#. Import the MJSystemMassMatrix class::

    from Basilisk.simulation import MJSystemMassMatrix

#. Enable extra EOM call when building the Mujoco scene::

    scene.extraEoMCall = True

#. Create an instance of MJSystemMassMatrix::

    module = MJSystemMassMatrix.MJSystemMassMatrix()

#. Set the scene the module is attached to::

    module.scene = scene

#. The MJSystemMassMatrix output message is ``massMatrixOutMsg``.

#. Add the module to the task list::

    unitTestSim.AddModelToTask(unitTaskName, module)

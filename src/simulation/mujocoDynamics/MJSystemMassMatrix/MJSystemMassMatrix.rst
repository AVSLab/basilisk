Executive Summary
-----------------
The ``MJSystemMassMatrix`` module extracts the full system mass matrix as well as stores the degrees of freedom from a Mujoco scene with a single spacecraft.

.. warning::
    This module is designed to be used with a Mujoco object that is a single spacecraft that has a rigid hub and a set of multi-jointed arms. The values extracted from
    the Mujoco scene are in generalized coordinates so other spacecraft layouts may result in unexpected behavior.

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
    * - massMatrixOutMsgC
      - :ref:`MJSysMassMatrixMsgPayload`
      - system mass matrix C output msg in generalized coordinates

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

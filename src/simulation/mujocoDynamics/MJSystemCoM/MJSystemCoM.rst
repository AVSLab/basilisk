Executive Summary
-----------------
The ``MJSystemCoM`` module extracts the total system CoM position and velocity
from a MuJoCo scene with a single spacecraft. For variable-mass bodies, the
reported velocity is the time derivative of the reported center-of-mass
position,

.. math::

    \dot{\mathbf r}_C =
    \frac{\sum_i m_i\mathbf v_i}{M}
    + \frac{\sum_i\dot m_i(\mathbf r_i-\mathbf r_C)}{M}.

The second term is read from each body's
``derivativeMassPropertiesInMsg``. It is a mass-property bookkeeping term,
not an additional generalized force in the MuJoCo equations of motion.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: MJSystemCoM

   output comStatesOutMsg SCStatesMsgPayload
      spacecraft CoM states C++ output msg

   output comStatesOutMsgC SCStatesMsgPayload
      spacecraft CoM states C output msg

User Guide
----------
This section is to outline the steps needed to setup the ``MJSystemCoM`` module in Python using Basilisk.

#. Import the MJSystemCoM class::

    from Basilisk.simulation import MJSystemCoM

#. Enable extra EOM call when building the MuJoCo scene::

    scene.extraEoMCall = True

#. Create an instance of MJSystemCoM::

    module = MJSystemCoM.MJSystemCoM()

#. Set the scene the module is attached to::

    module.scene = scene

#. The MJSystemCoM output message is ``comStatesOutMsg``.

#. Add the module to the MuJoCo scene dynamics task::

    scene.AddModelToDynamicsTask(module)

Executive Summary
-----------------
This module converts magnetic torque bar dipoles to body torques.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: MtbEffector
    :caption: Module I/O Messages

    input mtbCmdInMsg MTBCmdMsgPayload
        input msg for commanded Magnetic Torque Bar (MTB) dipole array in the magnetic torque bar frame `T`.
    input magInMsg MagneticFieldMsgPayload
        input msg for magnetic field data in inertial frame `N`.
    input mtbParamsInMsg MTBArrayConfigMsgPayload
        input msg for layout of magnetic torque bars.
    output mtbOutMsg MTBMsgPayload
        output message containing net torque produced by the torque bars in body frame `B` components.


User Guide
----------
Note that the MTB input configuration message variable ``GtMatrix_B`` must be provided in a row major format.

Initialization and Reset
------------------------
When the spacecraft links this effector to the hub attitude state, initialization verifies that ``mtbCmdInMsg``,
``magInMsg``, and ``mtbParamsInMsg`` are connected.  This validation occurs even if only the spacecraft is added
to a task.

Torque evaluation is driven by the spacecraft dynamics and reads the input messages directly.  Add the effector
to a task when ``mtbOutMsg`` must be updated, because the effector publishes that message from ``UpdateState()``.

When the effector is scheduled, its ``Reset()`` repeats the required-message checks and
zeros all external force and torque outputs.  The cached input-message data is not cleared.

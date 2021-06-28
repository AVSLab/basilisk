Executive Summary
-----------------
This module converts magnetic torque bar dipoles to body torques.

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
    * - mtbCmdInMsg
      - :ref:`MTBCmdMsgPayload`
      - input msgfor commanded mtb dipole array in the magnetic torque bar frame T
    * - magInMsg
      - :ref:`MagneticFieldMsgPayload`
      - input msg for magnetic field data in inertial frame N
    * - cmdTorqueInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - input msg for commanded torque in B frame
    * - cmdTorqueOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - output msg for commanded force in B frame


User Guide
----------
Note that the MTB input configuration message variable ``GtMatrix_B`` must be provided in a row major format.


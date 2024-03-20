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
      - input msg for commanded Magnetic Torque Bar (MTB) dipole array in the magnetic torque bar frame `T`
    * - magInMsg
      - :ref:`MagneticFieldMsgPayload`
      - input msg for magnetic field data in inertial frame `N`
    * - mtbParamsInMsg
      - :ref:`MTBArrayConfigMsgPayload`
      - input msg for layout of magnetic torque bars
    * - mtbOutMsg
      - :ref:`MTBMsgPayload`
      - output message containing net torque produced by the torque bars in body frame `B` components


User Guide
----------
Note that the MTB input configuration message variable ``GtMatrix_B`` must be provided in a row major format.


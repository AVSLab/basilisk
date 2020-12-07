Executive Summary
-----------------

Interface module to convert RW input voltage to a motor torque output.

For more information see the :download:`PDF Module Description </../../src/simulation/deviceInterface/rwVoltageInterface/_Documentation/Basilisk-rwVoltageInterface-20170129.pdf>`.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - rwVoltageInMsg
      - :ref:`RWArrayVoltageMsgPayload`
      - Message that contains RW voltage input states
    * - rwMotorTorqueOutMsg
      - :ref:`ArrayMotorTorqueMsgPayload`
      - Output Message for RW motor torques

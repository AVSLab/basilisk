Executive Summary
-----------------

Class used to provide a direct external force and torque on body. This class is used to simulate an external for or torque acting on the body.
For example, this module can be used to simulate the external disturbance due to
outgasing or a thruster, or be used to directly apply requested control forces or
torques.

The module
:download:`PDF Description </../../src/simulation/dynamics/extForceTorque/_Documentation/Basilisk-extForceTorque-20161103.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


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
    * - cmdTorqueInMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - commanded torque input msg
    * - cmdForceBodyInMsg
      - :ref:`CmdForceBodyMsgPayload`
      - commanded force input msg in B frame
    * - cmdForceInertialInMsg
      - :ref:`CmdForceInertialMsgPayload`
      - commanded force input msg in N frame


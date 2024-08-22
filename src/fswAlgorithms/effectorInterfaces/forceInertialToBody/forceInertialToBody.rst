Executive Summary
-----------------

This module does the very naive task of rotating the force output in inertial frame to body frame using navigation output. This allows 
to connect the output of meanOEFeedback module with foceTorqueThrForceMapping module.


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
    * - dataForceInertialInMsg
      - :ref:`CmdForceInertialMsgPayload`
      - Force in inertial frame. Requires forceRequestInertial.
    * - dataNavAttMsg
      - :ref:`NavAttMsgPayload`
      - Navigation Attitude message. Only requires sigma_BN.
    * - ForceBodyMsg
      - :ref: `CmdForceBodyMsgPayload`
      - outputs forceRequestBody.

Executive Summary
-----------------
This module estimates relative spacecraft position and velocity with respect to the body,attitude and attitude rate of the body wrt. the inertial frame, and the attitude and attituderate of the spacecraft with respect to the inertial frame

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
    * - navTransInMsg
      - :ref:`NavTransMsgPayload`
      - Translational nav input message
    * - navAttInMsg
      - :ref:`NavAttMsgPayload`
      - Attitude nav input message
    * - asteroidEphemerisInMsg
      - :ref:`EphemerisMsgPayload`
      - Small body ephemeris input message
    * - sunEphemerisInMsg
      - :ref:`EphemerisMsgPayload`
      - Sun ephemeris input message
    * - rwSpeedInMsg
      - :ref:`RWSpeedMsgPayload`
      - Reaction wheel speed input message
    * - navTransOutMsg
      - :ref:`NavTransMsgPayload`
      - Translational nav output message
    * - navAttOutMsg
      - :ref:`NavAttMsgPayload`
      - Attitude nav output message
    * - smallBodyNavOutMsg
      - :ref:`SmallBodyNavMsgPayload`
      - Small body nav output msg - states and covariances
    * - asteroidEphemerisOutMsg
      - :ref:`EphemerisMsgPayload`
      - Small body ephemeris output message


Executive Summary
-----------------

This module filters incoming star tracker measurements and reaction wheel data in order to get the best possible
inertial attitude estimate. The filter used is an unscented Kalman filter using the Modified Rodrigues Parameters
(MRPs) as a non-singular attitude measure. Measurements can be coming in from several camera heads and several
gyroscopes.

More information on can be found in the
:download:`PDF Description </../../src/fswAlgorithms/attDetermination/InertialUKF/_Documentation/Basilisk-inertialUKF-20190402.pdf>`

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 15 15 70
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - navAttitudeOutputMsg
      - :ref:`NavAttMsgPayload`
      - navigation output message
    * - inertialFilterOutputMsg
      - :ref:`InertialFilterMsgPayload`
      - name of the output filter data message
    * - starTrackerResidualMsg
      - :ref:`FilterResidualsMsgPayload`
      - name of the output residual data message for star measurements
    * - gyroResidualMsg
      - :ref:`FilterResidualsMsgPayload`
      - name of the output residual data message for gyro measurements
    * - vehicleConfigMsg
      - :ref:`VehicleConfigMsgPayload`
      - spacecraft vehicle configuration input message
    * - rwArrayConfigMsg
      - :ref:`RWArrayConfigMsgPayload`
      - reaction wheel parameter input message.  Can be an empty message if no RW are included.
    * - rwSpeedMsg
      - :ref:`RWSpeedMsgPayload`
      - reaction wheel speed input message.  Can be an empty message if no RW are included.
    * - accelDataMsg
      - :ref:`AccDataMsgPayload`
      - rate gyro input message
